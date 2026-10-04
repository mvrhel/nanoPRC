/* Copyright (C) 2023-2026 CascadiaVoxel LLC

    nanoPRC is free software: you can redistribute it and/or modify it under
    the terms of the GNU Affero General Public License as published by the
    Free Software Foundation, either version 3 of the License, or (at your
    option) any later version.

    nanoPRC is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public
    License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with nanoPRC. If not, see <https://www.gnu.org/licenses/>.
*/

/* Methods to create wire and tessellation structures from exact geometry */

#include "prc_data.h"
#include "prc_parse_common.h"
#include "prc_diag_env.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

//#define PRC_DEBUG_EARCLIP 1
#define CHECK_SURFACE_PROJECTION 0

#define CURVE_SAMPLES 32
#define SURFACE_SAMPLES 32
/* Temporarily loosened for debugging (was 1e-3) to get a coarser boundary
   loop mesh while diagnosing the periodic-loop trim work -- restore before
   shipping */
#define CURVE_PRECISION 1e-1
#define SURFACE_PRECISION 1e-4
#define SURFACE_MAX_SAMPLES 1024
#define CYLINDER_SURFACE_PRECISION 1e-2
#define CYLINDER_MAX_SAMPLES 128
#define CONE_SURFACE_PRECISION 1e-2
#define CONE_MAX_SAMPLES 128
#define SPHERE_SURFACE_PRECISION 1e-2
#define SPHERE_MAX_SAMPLES 128
#define TORUS_SURFACE_PRECISION 1e-2
#define TORUS_MAX_SAMPLES 128
#define CYLINDRICAL_SURFACE_PRECISION 1e-2
#define CYLINDRICAL_MAX_SAMPLES 128
#define EXTRUSION_SURFACE_PRECISION 1e-1
#define EXTRUSION_MAX_SAMPLES 128
#define REVOLUTION_SURFACE_PRECISION 1e-1
#define REVOLUTION_MAX_SAMPLES 128
#define PLANE_MAX_SAMPLES 4
#define PLANE_SURFACE_PRECISION 1e-2
#define NURBS_SURFACE_PRECISION 1e-4
#define NURBS_MAX_SAMPLES 1024
/* Largest B-spline degree the fixed-size basis function buffers below can hold */
#define PRC_BSPLINE_MAX_DEGREE 15
/* Safety cap on prc_subdivide_curved_triangle's recursion (3^depth leaf
   triangles worst case per boundary ear-clip triangle). Kept low for now
   while debugging -- raise once the trim shape itself is confirmed correct */
#define PRC_CURVED_LOOP_SUBDIVIDE_MAX_DEPTH 3
/* Safety cap on prc_subdivide_trim_boundary_triangle's recursion. In
   practice only the one branch of the fan-split that keeps straddling the
   trim curve recurses this deep (siblings resolve to all-in/all-out after
   a level or two), so cost is roughly linear in boundary cell count, not
   3^depth */
#define PRC_TRIM_BOUNDARY_SUBDIVIDE_MAX_DEPTH 7

static int prc_tessellate_surface(prc_context *ctx, prc_data *data,
    uint32_t shell_index, uint32_t face_index, prc_topo_face *topo_face,
    uint8_t orientation, prc_nano_brep_ref_data *brep_ref_data,
    prc_topo_context *topo_context);

/* A standard type for curve sampling */
typedef prc_vec3 (*curve_func)(prc_context *ctx, void *params, double input);

/* And for surfaces */
typedef prc_vec3 (*surface_func)(prc_context *ctx, void *params, double u, double v);

/* Something to hold loop samples that should be sufficiently fine to use
   in the tessellation of a surface. These will be boundary points that should
   be included in the tessellation of the surface */
typedef struct prc_loop_samples_s
{
    uint32_t num_samples;
    prc_vec3 *samples;
    prc_vec2 *uv_samples;
    uint8_t is_outer_loop;
    /* Full 2*pi turns this loop winds around each periodic uv axis before
       closing back up (0 if it is a genuine simple polygon within one tile).
       Set by prc_map_loops_to_surface, consulted by prc_tessellate_surface
       to pick between the closed-loop and seam-cut periodic tessellation
       paths */
    int32_t wind_u;
    int32_t wind_v;
    /* Set by prc_sample_loop when every sample coincides (e.g. a loop
       trimmed down to a cone's apex): such a loop bounds zero area and must
       not be fed to the hole/ear-clip machinery or an angular (atan2-based)
       uv mapping, both of which are numerically unstable or meaningless for
       a degenerate single point */
    uint8_t is_point_loop;
} prc_loop_samples;

typedef struct prc_coedge_samples_s
{
    uint32_t num_samples;
    prc_vec3 *samples;
} prc_coedge_samples;

typedef struct prc_surface_sampling_info_s
{
    uint8_t u_periodic;
    uint8_t v_periodic;
    uint8_t u_linear;
    uint8_t v_linear;
    double u_period;
    double v_period;

    double start_u;
    double end_u;
    double start_v;
    double end_v;
    uint32_t max_samples_u;
    uint32_t max_samples_v;

    double precision_u;
    double precision_v;

    uint32_t num_samples_u;
    uint32_t num_samples_v;
} prc_surface_sampling_info;

typedef struct prc_curve_sampling_info_s
{
    double start;
    double end;
    uint32_t num_samples;
    void *curve_params;
    curve_func curve_eval_func;
} prc_curve_sampling_info;

typedef struct prc_surface_params_s
{
    void *surface_params;
    uint32_t num_loops;
    prc_loop_samples *loop_samples;
} prc_surface_params;

/* Forward declaration - populates sampling_info (including the valid parametric domain)
   for any prc_type_surf; needed early by the Blend02 bound-projection helpers */
static int prc_get_surface_data(prc_context *ctx, prc_type_surf *surface,
    prc_surface_sampling_info *sampling_info);

/* Forward declaration - used by prc_evaluate_composite before the function body appears */
static int prc_get_curve_sample_info(prc_context *ctx, prc_ptr_curve *ptr_curve,
    prc_curve_sampling_info *sample_info);

/* Forward declaration - used in curves before surfaces occur */
static int prc_get_surface_eval_func(prc_context *ctx, prc_type_surf *surface,
    surface_func *eval_func, void **params);

/* Forward declaration */
static int prc_get_hcg_circle_data(prc_context *ctx, prc_hcg_circle *hcg_circle,
    prc_hcg_circle_information *info, prc_nano_brep_compressed_data *compressed_data);

/* Invert the transform associated with the surface. This is applied to the loop
   samples */
static int
prc_invert_exact_transform(prc_context *ctx, prc_exact_geom_transform *transform_in,
    prc_exact_geom_transform *inverse_transform_out)
{
    const double *m = transform_in->matrix;
    double *out;
    double a, b, c, d, e, f, g, h, i;
    double inv00, inv01, inv02, inv10, inv11, inv12, inv20, inv21, inv22;
    double det, inv_det;
    double tx, ty, tz;

    if (transform_in->is_identity)
    {
        memset(inverse_transform_out->matrix, 0, sizeof(double) * 16);
        inverse_transform_out->matrix[0] = 1.0;
        inverse_transform_out->matrix[5] = 1.0;
        inverse_transform_out->matrix[10] = 1.0;
        inverse_transform_out->matrix[15] = 1.0;
        inverse_transform_out->is_identity = 1;
        return 0;
    }

    /* Column-major 4x4: column c, row r lives at m[c * 4 + r] (see
       prc_api_transform_point). The upper-left 3x3 is the linear part. */
    a = m[0]; d = m[1]; g = m[2];
    b = m[4]; e = m[5]; h = m[6];
    c = m[8]; f = m[9]; i = m[10];

    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (det == 0.0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Singular matrix in prc_invert_exact_transform\n");
        return PRC_ERROR_INTERNAL;
    }
    inv_det = 1.0 / det;

    inv00 = (e * i - f * h) * inv_det;
    inv01 = (c * h - b * i) * inv_det;
    inv02 = (b * f - c * e) * inv_det;
    inv10 = (f * g - d * i) * inv_det;
    inv11 = (a * i - c * g) * inv_det;
    inv12 = (c * d - a * f) * inv_det;
    inv20 = (d * h - e * g) * inv_det;
    inv21 = (b * g - a * h) * inv_det;
    inv22 = (a * e - b * d) * inv_det;

    tx = m[12]; ty = m[13]; tz = m[14];

    out = inverse_transform_out->matrix;
    memset(out, 0, sizeof(double) * 16);
    out[0] = inv00; out[1] = inv10; out[2] = inv20;
    out[4] = inv01; out[5] = inv11; out[6] = inv21;
    out[8] = inv02; out[9] = inv12; out[10] = inv22;
    /* Inverted translation is -(Inverse linear part) * original translation */
    out[12] = -(inv00 * tx + inv01 * ty + inv02 * tz);
    out[13] = -(inv10 * tx + inv11 * ty + inv12 * tz);
    out[14] = -(inv20 * tx + inv21 * ty + inv22 * tz);
    out[15] = 1.0;

    inverse_transform_out->is_identity = 0;

    return 0;
}

/* A version of the 3D transform that we use for exact geometry. This one is limited
   to Identity, Translate, Rotate and Scale */
static int
prc_exact_geom_set_transform(prc_context *ctx, prc_exact_geom_transform *exact_geom_trans,
    const prc_trans_3d *prc_trans)
{
    if (exact_geom_trans == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "exact_geom_trans is NULL in prc_exact_geom_set_transform\n");
        return PRC_ERROR_MEMORY;
    }
    uint8_t other_flags_set = 0;
    double *matrix = exact_geom_trans->matrix;
    char behavior = prc_trans->behavior;

    memset(matrix, 0, sizeof(double) * 16);
    exact_geom_trans->is_identity = 0;

    /* Identity */
    matrix[0] = 1.0;
    matrix[5] = 1.0;
    matrix[10] = 1.0;
    matrix[15] = 1.0;

    if (behavior == 0)
    {
        exact_geom_trans->is_identity = 1;
        return 0;
    }

    if (behavior & PRC_TRANSFORMATION_Translate)
    {
        matrix[12] = prc_trans->translation.x;
        matrix[13] = prc_trans->translation.y;
        matrix[14] = prc_trans->translation.z;
    }

    if (behavior & PRC_TRANSFORMATION_Rotate)
    {
        prc_vec3 z_axis;

        if (behavior & PRC_TRANSFORMATION_Mirror)
        {
            /* Z is cross product of Y and X */
            prc_vec_cross(prc_trans->rotation[1],
                prc_trans->rotation[0], &z_axis);
        }
        else
        {
            prc_vec_cross(prc_trans->rotation[0],
                prc_trans->rotation[1], &z_axis);
        }
        /* Now load the matrix */
        matrix[0] = prc_trans->rotation[0].x;
        matrix[1] = prc_trans->rotation[0].y;
        matrix[2] = prc_trans->rotation[0].z;
        matrix[4] = prc_trans->rotation[1].x;
        matrix[5] = prc_trans->rotation[1].y;
        matrix[6] = prc_trans->rotation[1].z;
        matrix[8] = z_axis.x;
        matrix[9] = z_axis.y;
        matrix[10] = z_axis.z;

        other_flags_set = 1;
    }

    if (behavior & PRC_TRANSFORMATION_Scale)
    {
        for (int i = 0; i < 11; i++)
        {
            matrix[i] = matrix[i] * prc_trans->scale;
        }
    }

    /* Check if resulting matrix is still identity for performance */
    {
        static const double identity[16] = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
        const double eps = 1e-12;
        int is_identity = 1;

        for (int i = 0; i < 16; i++)
        {
            if (fabs(matrix[i] - identity[i]) > eps)
            {
                is_identity = 0;
                break;
            }
        }

        if (is_identity)
            exact_geom_trans->is_identity = 1;
    }

    return 0;
}

static prc_vec3
prc_exact_geom_apply_transform(prc_context *ctx,
    const prc_exact_geom_transform *exact_geom_trans, prc_vec3 point)
{
    prc_vec3 transformed = point;
    const double *m;

    if (exact_geom_trans == NULL || exact_geom_trans->is_identity)
        return transformed;

    m = exact_geom_trans->matrix;

    transformed.x = (m[0] * point.x) + (m[4] * point.y) + (m[8] * point.z) + m[12];
    transformed.y = (m[1] * point.x) + (m[5] * point.y) + (m[9] * point.z) + m[13];
    transformed.z = (m[2] * point.x) + (m[6] * point.y) + (m[10] * point.z) + m[14];

    return transformed;
}

/* A method to get the periodicity of a curve. Needed to determine the periodicity
   of a surface that makes use of this curve. */
static void
prc_get_curve_periodicity(prc_context *ctx, prc_ptr_curve *curve,
    uint8_t *is_periodic, double *period)
{
    *is_periodic = 0;
    *period = 0.0;

    switch (curve->curve_type)
    {
        case PRC_TYPE_CRV_Circle:
        case PRC_TYPE_CRV_Ellipse:
            *is_periodic = 1;
            *period = 2.0 * PRC_PI;
            break;

        default:
            *is_periodic = 0;
            *period = 0;
    }
}

/* Line is defined by a start and a direction.  Project point onto this line */
static prc_vec3
prc_project_point_onto_line(prc_context *ctx, prc_vec3 point, prc_vec3 origin,
    prc_vec3 vector)
{
    double temp1;
    prc_vec3 output;
    prc_vec3 b;
    prc_vec3 projected_point;

    temp1 = prc_vec_dot_product(vector, vector);
    if (temp1 == 0.0)
    {
        /* Quiet compiler */
        output.x = 0;
        output.y = 0;
        output.z = 0;
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid projection\n");
        return output;
    }

    prc_vec_sub(point, origin, &b);
    temp1 = prc_vec_dot_product(b, vector) / temp1;
    prc_vec_scale(temp1, &vector);
    prc_vec_add(origin, vector, &projected_point);

    return projected_point;
}

/* The NURBS Book (Piegl & Tiller) Algorithm A2.1 - find the knot span containing u */
static uint32_t
prc_bspline_find_span(uint32_t degree, uint32_t num_ctrl_pts, double u, const double *knots)
{
    uint32_t n = num_ctrl_pts - 1;
    uint32_t low, high, mid;

    if (u >= knots[n + 1])
        return n;
    if (u <= knots[degree])
        return degree;

    low = degree;
    high = n + 1;
    mid = (low + high) / 2;
    while (u < knots[mid] || u >= knots[mid + 1])
    {
        if (u < knots[mid])
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

/* The NURBS Book (Piegl & Tiller) Algorithm A2.2 - the degree+1 nonzero basis functions at u */
static void
prc_bspline_basis_funs(uint32_t span, double u, uint32_t degree, const double *knots, double *N)
{
    double left[PRC_BSPLINE_MAX_DEGREE + 1];
    double right[PRC_BSPLINE_MAX_DEGREE + 1];
    uint32_t j, r;

    N[0] = 1.0;
    for (j = 1; j <= degree; j++)
    {
        double saved = 0.0;

        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        for (r = 0; r < j; r++)
        {
            double denom = right[r + 1] + left[j - r];
            double temp = (denom != 0.0) ? N[r] / denom : 0.0;

            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
}

/* Evaluate parabola at a single point.

   The published type-0 formula is wrong; this uses the corrected one. We
   derived a correction independently, and it turned out to be algebraically
   identical to the one proposed in pdf-issues #782 -- two implementations
   arriving at the same evaluation from different starting points.

   The arrangement below is #782's rather than ours: y = 2*sqrt(x*f) instead of
   the equivalent y = 2*f*sqrt(x/f). Identical for any f > 0, but it avoids
   dividing by the focal length and taking the square root of a quotient, so it
   stays well behaved as f becomes small and needs no separate guard against a
   zero focal length -- where the old form evaluated 0/0, this one gives 0. We
   said on that issue that we would adopt it. */
static prc_vec3
prc_evaluate_parabola(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_parabola *parabola = (prc_crv_parabola *)params;
    double focal_length = parabola->focal_length;
    output.z = 0;

    if (parabola->type == 0)
    {
        double param2 = input * input;
        double p2 = param2 / sqrt(16.0 * focal_length * focal_length + param2);
        output.x = p2;
        output.y = 2.0 * sqrt(output.x * focal_length);
        if (input < 0.0)
        {
            output.y = -output.y;
        }
    }
    else
    {
        output.x = focal_length * input * input;
        output.y = 2.0 * focal_length * input;
    }

    if (parabola->has_transform && !parabola->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &parabola->exact_geom_transform, output);
    }

    return output;
}

/* Evaluate line at a single point */
static prc_vec3
prc_evaluate_line(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_line *line = (prc_crv_line *)params;

    output.x = input;
    output.y = 0;
    output.z = 0;

    if (line->has_transform && !line->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &line->exact_geom_transform, output);
    }

    return output;
}

/* For the compressed cirlce, we will always be running from zero to one
   and falling along the circle arc.  We should always have center and
   the start and end point and the normal vector */
static prc_vec3
prc_evaluate_circle_compressed(prc_context *ctx, void *params, double input)
{
    prc_vec3 output = { 0 };
    prc_hcg_circle *circle = (prc_hcg_circle *)params;
    prc_hcg_circle_information *info = &circle->circle_data;
    prc_vec3 basis_vector1;
    prc_vec3 basis_vector2;
    prc_vec3 normal;
    prc_vec3 sum;
    double scale1, scale2;
    int code;

    if (info->theta)
    {
        return output;
    }

    /* Get the basis vectors for our parameterization */
    prc_vec_sub(info->start_point, info->center, &basis_vector1);
    
    /* Make sure the normal vector is normalized */
    prc_vec_copy(info->normal, &normal, 0);
    code = prc_vec_normalize(&normal);
    if (code < 0)
    {
        return output;
    }
    prc_vec_cross(info->normal, basis_vector1, &basis_vector2);

    scale1 = cos(input * info->theta);
    scale2 = sin(input * info->theta);

    prc_vec_scale(scale1, &basis_vector1);
    prc_vec_scale(scale2, &basis_vector2);
    prc_vec_add(basis_vector1, basis_vector2, &sum);
    prc_vec_add(info->center, sum, &output);

    return output;
}

static prc_vec3
prc_evaluate_hermite_compressed(prc_context *ctx, void *params, double input)
{
    prc_vec3 output = { 0 };
    prc_hcg_bspline_hermite_curve *curve = (prc_hcg_bspline_hermite_curve *)params;
    prc_vec3 start_point;
    prc_vec3 end_point;
    prc_vec3 *key_points = NULL;
    prc_vec3 *key_tangents = NULL;
    prc_vec3 segment_delta;
    prc_vec3 p0, p3, p1, p2;
    prc_vec3 t0, t1;
    double segment_length;
    double segment_u;
    uint32_t segment_index;
    uint32_t segment_count;
    uint32_t i;
    double b0, b1, b2, b3;

    if (curve == NULL)
    {
        return output;
    }

    start_point = curve->start_end_data.start_point.point;
    end_point = curve->start_end_data.end_point.point;

    if (input <= 0.0)
    {
        return start_point;
    }
    if (input >= 1.0)
    {
        return end_point;
    }

    if (curve->number_points < 2)
    {
        return start_point;
    }

    key_points = (prc_vec3 *)prc_calloc(ctx, curve->number_points, sizeof(prc_vec3));
    if (key_points == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate key_points in prc_evaluate_hermite_compressed\n");
        return start_point;
    }

    key_points[0] = start_point;
    for (i = 0; i < curve->number_points - 2; i++)
    {
        if (curve->points != NULL)
        {
            prc_vec_add(key_points[i], curve->points[i], &key_points[i + 1]);
        }
        else
        {
            key_points[i + 1] = key_points[i];
        }
    }
    key_points[curve->number_points - 1] = end_point;

    if (curve->tangents != NULL)
    {
        /* Per spec Table 238, only the points (Ptc) are stored as a running delta;
           Tgtc[i] is used directly as the tangent at key point i (P1/P2/P4 formulas
           reference Tgtc[0..2] and Tgtc[3..5] with no cumulative sum). Accumulating
           the tangents compounds quantization noise across the curve, producing
           high-frequency wiggles. */
        key_tangents = curve->tangents;
    }

    segment_count = curve->number_points - 1;
    segment_u = input * (double)segment_count;
    segment_index = (uint32_t)segment_u;
    if (segment_index >= segment_count)
    {
        segment_index = segment_count - 1;
    }
    segment_u = segment_u - (double)segment_index;

    p0 = key_points[segment_index];
    p3 = key_points[segment_index + 1];

    if (curve->tangents == NULL)
    {
        prc_free(ctx, key_points);
        return p0;
    }

    t0 = key_tangents[segment_index];
    t1 = key_tangents[segment_index + 1];

    prc_vec_sub(p3, p0, &segment_delta);
    segment_length = prc_vec_length(segment_delta);
    if (segment_length <= CURVE_PRECISION)
    {
        prc_free(ctx, key_points);
        return p0;
    }

    if (prc_vec_length(t0) > CURVE_PRECISION)
    {
        prc_vec_scale(segment_length / 3.0 / prc_vec_length(t0), &t0);
    }
    if (prc_vec_length(t1) > CURVE_PRECISION)
    {
        prc_vec_scale(segment_length / 3.0 / prc_vec_length(t1), &t1);
    }

    prc_vec_add(p0, t0, &p1);
    prc_vec_sub(p3, t1, &p2);

    b0 = (1.0 - segment_u) * (1.0 - segment_u) * (1.0 - segment_u);
    b1 = 3.0 * (1.0 - segment_u) * (1.0 - segment_u) * segment_u;
    b2 = 3.0 * (1.0 - segment_u) * segment_u * segment_u;
    b3 = segment_u * segment_u * segment_u;

    output.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
    output.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
    output.z = b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z;

    prc_free(ctx, key_points);
    return output;
}

/* For the compressed line, which has the start_end_data as its parameters
   and we just run from zero to one.  We really should only be here if the
   data is given as a point not a vertex. */
static prc_vec3
prc_evaluate_line_compressed(prc_context *ctx, void *params, double input)
{
    prc_vec3 output = { 0 };
    prc_start_end_data *line = (prc_start_end_data *)params;
    prc_vec3 point1;
    prc_vec3 point2;

    if (input <= 0)
    {
        output = line->start_point.point;
        return output;
    }
    if (input >= 1)
    {
        output = line->end_point.point;
        return output;
    }

    point1 = line->start_point.point;
    point2 = line->start_point.point;
    prc_vec_scale(1 - input, &point1);
    prc_vec_scale(input, &point2);
    prc_vec_add(point1, point2, &output);

    return output;
}

/* Evaluate hyperbola at a single point */
static prc_vec3
prc_evaluate_hyperbola(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_hyperbola *hyperbola = (prc_crv_hyperbola *)params;
    double semi_axis_image = hyperbola->semi_axis_image;
    double semi_axis = hyperbola->semi_axis;
    double temp;

    output.z = 0;

    if (hyperbola->type == 0)
    {
        if (semi_axis_image == 0.0)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid hyperbola semi_axis_image in prc_evaluate_hyperbola\n");
            return output;
        }
        temp = input / semi_axis;
        output.x = semi_axis_image * sqrt(1.0 + temp * temp);
        output.y = input;
    }
    else
    {
        output.x = semi_axis_image * cosh(input);
        output.y = semi_axis * sinh(input);
    }

    if (hyperbola->has_transform && !hyperbola->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &hyperbola->exact_geom_transform, output);
    }

    return output;
}

/* Evaluate an offset curve at a single point */
static prc_vec3
prc_evaluate_offset_curve(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_offset *offset = (prc_crv_offset *)params;
    prc_vec3 base_point;
    prc_vec3 offset_dir;
    curve_func base_func;
    int code;

    if (offset->base_func == NULL || offset->base_params == NULL)
    {
        output.x = 0.0;
        output.y = 0.0;
        output.z = 0.0;
        prc_error(ctx, PRC_ERROR_INTERNAL, "Missing base curve evaluator in prc_evaluate_offset_curve\n");
        return output;
    }

    base_func = (curve_func)offset->base_func;
    base_point = base_func(ctx, offset->base_params, input);
    offset_dir.x = 0.0;
    offset_dir.y = 0.0;
    offset_dir.z = 0.0;

    /* The base evaluator is also used for the finite-difference derivative. */
    {
        double h = 1e-5;
        double before_input = input - h;
        double after_input = input + h;
        prc_vec3 before;
        prc_vec3 after;
        prc_vec3 base_deriv;

        if (before_input < offset->parameterization.interval.min_value)
            before_input = offset->parameterization.interval.min_value;
        if (after_input > offset->parameterization.interval.max_value)
            after_input = offset->parameterization.interval.max_value;
        before = base_func(ctx, offset->base_params, before_input);
        after = base_func(ctx, offset->base_params, after_input);
        if (after_input == before_input)
            return base_point;
        base_deriv.x = (after.x - before.x) / (after_input - before_input);
        base_deriv.y = (after.y - before.y) / (after_input - before_input);
        base_deriv.z = (after.z - before.z) / (after_input - before_input);
        prc_vec_cross(base_deriv, offset->offset_plane_normal, &offset_dir);
    }

    code = prc_vec_normalize(&offset_dir);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Degenerate offset direction in prc_evaluate_offset_curve\n");
        return base_point;
    }

    output.x = base_point.x + offset->offset * offset_dir.x;
    output.y = base_point.y + offset->offset * offset_dir.y;
    output.z = base_point.z + offset->offset * offset_dir.z;

    if (offset->has_transform && !offset->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &offset->exact_geom_transform, output);
    }

    return output;
}

/* Evaluate circle at a single point */
static prc_vec3
prc_evaluate_circle(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_circle *circle = (prc_crv_circle *)params;
    double radius = circle->radius;

    output.z = 0;
    output.x = radius * cos(input);
    output.y = radius * sin(input);

    if (circle->has_transform && !circle->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &circle->exact_geom_transform, output);
    }

    return output;
}

/* Evaluate a polyline at a single point */
static prc_vec3
prc_evaluate_polyline(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_polyline *polyline = (prc_crv_polyline *)params;
    uint32_t num_points = polyline->number_of_points;
    uint8_t is_3d = polyline->curve_data.is_3d_flag;

    if (num_points < 2)
    {
        /* Quiet compiler */
        output.x = 0;
        output.y = 0;
        output.z = 0;
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid polyline with less than 2 points in prc_evaluate_polyline\n");
        return output;
    }

    /* Clamp input to the range of the polyline */
    if (input < 0.0)
        input = 0.0;
    if (input > (double)(num_points - 1))
        input = (double)(num_points - 1);
    uint32_t index = (uint32_t)input;
    double t = input - (double)index;
    if (index >= num_points - 1)
    {
        if (is_3d)
        {
            output = polyline->points[num_points - 1].point_3d;
        }
        else
        {
            output.x = polyline->points[num_points - 1].point_2d.x;
            output.y = polyline->points[num_points - 1].point_2d.y;
            output.z = 0.0;
        }
    }
    else
    {
        if (is_3d)
        {
            prc_vec3 p0 = polyline->points[index].point_3d;
            prc_vec3 p1 = polyline->points[index + 1].point_3d;
            output.x = (1.0 - t) * p0.x + t * p1.x;
            output.y = (1.0 - t) * p0.y + t * p1.y;
            output.z = (1.0 - t) * p0.z + t * p1.z;
        }
        else
        {
            prc_vec2 p0 = polyline->points[index].point_2d;
            prc_vec2 p1 = polyline->points[index + 1].point_2d;
            output.x = (1.0 - t) * p0.x + t * p1.x;
            output.y = (1.0 - t) * p0.y + t * p1.y;
            output.z = 0.0;
        }
    }

    if (polyline->has_transform && !polyline->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &polyline->exact_geom_transform, output);
    }

    return output;
}

/* Evaluate an ellipse at a single point */
static prc_vec3
prc_evaluate_ellipse(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_ellipse *ellipse = (prc_crv_ellipse *)params;
    double rx = ellipse->rx;
    double ry = ellipse->ry;

    output.z = 0;
    output.x = rx * cos(input);
    output.y = ry * sin(input);

    if (ellipse->has_transform && !ellipse->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &ellipse->exact_geom_transform, output);
    }

    return output;
}

/* Evaluate a helix at a single point */
static prc_vec3
prc_evaluate_helix(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
    prc_crv_helix01 *helix = (prc_crv_helix01 *)params;
    uint8_t type = helix->type;

    if (type == 0)
    {
        double radius_evolution = helix->type0_helix.radius;
        double radius;
        double pitch = helix->type0_helix.pitch;
        prc_vec3 origin, z_axis, start, origin_on_axis, b, x_axis;
        double temp1;

        origin.x = helix->type0_helix.origin_0;
        origin.y = helix->type0_helix.origin_1;
        origin.z = helix->type0_helix.origin_2;

        z_axis.x = helix->type0_helix.direction_0;
        z_axis.y = helix->type0_helix.direction_1;
        z_axis.z = helix->type0_helix.direction_2;

        start = helix->start;

        /* Project the start onto the z axis which is defined
           by the origin and direction */
        origin_on_axis = prc_project_point_onto_line(ctx, start, origin, z_axis);

        prc_vec_sub(start, origin_on_axis, &x_axis);
        radius = prc_vec_length(x_axis) + input * radius_evolution;

        if (helix->orientation == 1)
        {
            output.x = radius * cos(input);
            output.y = radius * sin(input);
            output.z = pitch * input;
        }
        else
        {
            output.x = radius * cos(-input);
            output.y = radius * sin(-input);
            output.z = pitch * input;
        }
    }
    else
    {
        /* Todo implement this */
        output.x = 0;
        output.y = 0;
        output.z = 0;
    }

    if (helix->has_transform && !helix->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &helix->exact_geom_transform, output);
    }

    return output;
}

static prc_vec3
prc_evaluate_crv_nurbs(prc_context *ctx, void *params, double u)
{
    prc_crv_nurbs *nurbs = (prc_crv_nurbs *)params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    double N[PRC_BSPLINE_MAX_DEGREE + 1];
    uint32_t num_ctrl = nurbs->highest_index_of_control_points + 1;
    uint32_t span, i;
    double x = 0.0, y = 0.0, z = 0.0, weight_sum = 0.0;

    if (nurbs->d > PRC_BSPLINE_MAX_DEGREE)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "NURBS curve degree exceeds supported maximum\n");
        return output;
    }

    span = prc_bspline_find_span(nurbs->d, num_ctrl, u, nurbs->u);
    prc_bspline_basis_funs(span, u, nurbs->d, nurbs->u, N);

    for (i = 0; i <= nurbs->d; i++)
    {
        uint32_t ctrl = span - nurbs->d + i;
        prc_control_points_nurbs_crv *cp = &nurbs->p[ctrl];
        double weight = nurbs->is_rational ? cp->w : 1.0;
        double basis = N[i] * weight;

        x += basis * cp->x;
        y += basis * cp->y;
        z += basis * cp->z;
        weight_sum += basis;
    }

    if (weight_sum != 0.0)
    {
        output.x = x / weight_sum;
        output.y = y / weight_sum;
        output.z = z / weight_sum;
    }

    if (output.x > 400.0 || output.y > 400.0 || output.x < -400.0 || output.y < -400.0)
    {
        fprintf(stderr,
            "[NURBS eval debug] u=%g span=%u weight_sum=%g\n"
            "  output=(%g,%g,%g)\n",
            u, span, weight_sum,
            output.x, output.y, output.z);
        for (i = 0; i <= nurbs->d; i++)
        {
            uint32_t ctrl = span - nurbs->d + i;
            prc_control_points_nurbs_crv *cp = &nurbs->p[ctrl];
            fprintf(stderr,
                "  ctrl[%u]=(x=%g,y=%g,z=%g,w=%g) N=%g weighted=%g\n",
                ctrl, cp->x, cp->y, cp->z, cp->w,
                N[i], N[i] * (nurbs->is_rational ? cp->w : 1.0));
        }
    }

    return output;
}

static prc_vec3
prc_evaluate_onsurf(prc_context *ctx, void *params, double w)
{
    prc_crv_onsurf *onsurf = (prc_crv_onsurf *)params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    curve_func base_curve_eval;
    prc_vec3 curve_output;
    surface_func base_surf_eval;

    if (onsurf == NULL || onsurf->base_curve_func == NULL ||
        onsurf->base_curve_params == NULL || onsurf->base_surface_func == NULL ||
        onsurf->base_surface_params == NULL)
    {
        return output;
    }

    /* Evaluate w on the curve to get the u, v position */
    base_curve_eval = (curve_func) onsurf->base_curve_func;
    curve_output = base_curve_eval(ctx, onsurf->base_curve_params, w);

    /* Evaluate u, v on the surface to get the XYZ position */
    base_surf_eval = (surface_func) onsurf->base_surface_func;
    output = base_surf_eval(ctx, onsurf->base_surface_params, curve_output.x, curve_output.y);

    return output;
}

static prc_vec3
prc_evaluate_composite(prc_context *ctx, void *params, double u)
{
    prc_crv_composite *composite = (prc_crv_composite *)params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    double implicit_parameter;
    uint32_t subcurve_index;
    double delta;
    double local_param;
    prc_curve_sampling_info subcurve_info;
    prc_composite_subcurve *subcurve;
    curve_func subcurve_eval;
    void *subcurve_params;
    int code;

    if (composite == NULL || composite->subcurves == NULL || composite->number_of_subcurves == 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid composite curve in prc_evaluate_composite\n");
        return output;
    }

    if (composite->parameterization.coeff_a != 0.0)
    {
        implicit_parameter = (u - composite->parameterization.coeff_b) / composite->parameterization.coeff_a;
    }
    else
    {
        implicit_parameter = u;
    }

    if (implicit_parameter < 0.0)
        implicit_parameter = 0.0;
    else if (implicit_parameter > (double)composite->number_of_subcurves)
        implicit_parameter = (double)composite->number_of_subcurves;

    subcurve_index = (uint32_t)implicit_parameter;
    if (subcurve_index >= composite->number_of_subcurves)
        subcurve_index = composite->number_of_subcurves - 1;

    subcurve = &composite->subcurves[subcurve_index];
    memset(&subcurve_info, 0, sizeof(subcurve_info));
    code = prc_get_curve_sample_info(ctx, &subcurve->ptr_curve, &subcurve_info);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed to sample composite subcurve in prc_evaluate_composite\n");
        return output;
    }

    if (subcurve->base_func != NULL && subcurve->base_params != NULL)
    {
        subcurve_eval = (curve_func)subcurve->base_func;
        subcurve_params = subcurve->base_params;
    }
    else
    {
        subcurve_eval = subcurve_info.curve_eval_func;
        subcurve_params = subcurve_info.curve_params;
    }

    delta = implicit_parameter - (double)subcurve_index;
    if (subcurve->sense)
    {
        local_param = subcurve_info.start + delta * (subcurve_info.end - subcurve_info.start);
    }
    else
    {
        local_param = subcurve_info.end - delta * (subcurve_info.end - subcurve_info.start);
    }

    output = subcurve_eval(ctx, subcurve_params, local_param);
    return output;
}

static int
prc_get_compressed_curve_sample_info(prc_context *ctx, prc_compressed_curve *curve,
    prc_curve_sampling_info *sample_info)
{
    int code;

    switch (curve->curve_type)
    {
        case PRC_HCG_Line:
        {
            /* We will sample from 0 to 1 and run along the start and end data */
            sample_info->curve_params = &curve->hcg_line.start_end_data;
            sample_info->curve_eval_func = prc_evaluate_line_compressed;
            sample_info->start = 0;
            sample_info->end = 1;
            sample_info->num_samples = 2;
            break;
        }
        case PRC_HCG_Circle:
        {
            /* We will sample from 0 to 1 and run along the circle arc length
               specified.  First though distill the circle information from
               the particular/general circle forms that we have */
            if (!curve->hcg_circle.information_valid)
            {
                code = prc_get_hcg_circle_data(ctx, &curve->hcg_circle,
                    &curve->hcg_circle.circle_data, NULL);
                if (code < 0)
                {
                    prc_error(ctx, PRC_ERROR_INTERNAL, "Failed in prc_get_hcg_circle_data\n");
                    return PRC_ERROR_INTERNAL;
                }
                curve->hcg_circle.information_valid = 1;
            }
            sample_info->curve_params = &curve->hcg_circle;
            sample_info->curve_eval_func = prc_evaluate_circle_compressed;
            sample_info->start = 0;
            sample_info->end = 1;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_HCG_BsplineHermiteCurve:
        {
            /* We will sample from 0 to 1 and run along the start and end data */
            sample_info->curve_params = &curve->hcg_bspline_hermite_curve;
            sample_info->curve_eval_func = prc_evaluate_hermite_compressed;
            sample_info->start = 0;
            sample_info->end = 1;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_HCG_CompositeCurve:
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "TODO implement this\n");
            return PRC_ERROR_INTERNAL;

            break;
        }

        default:
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_compressed_curve_sample_info\n");
            return PRC_ERROR_INTERNAL;

        }
    return 0;
}

static int 
prc_get_curve_sample_info(prc_context *ctx, prc_ptr_curve *ptr_curve,
    prc_curve_sampling_info *sample_info)
{
    int code;

    switch (ptr_curve->curve_type)
    {
        case PRC_TYPE_CRV_NURBS:
        {
            prc_crv_nurbs *nurbs = ptr_curve->crv_nurbs;

            sample_info->curve_params = (void *)nurbs;
            sample_info->curve_eval_func = prc_evaluate_crv_nurbs;
            sample_info->start = nurbs->u[nurbs->d];
            sample_info->end = nurbs->u[nurbs->highest_index_of_knots - nurbs->d];
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Parabola:
        {
            prc_crv_parabola *parabola = ptr_curve->crv_parabola;
            prc_parameterization params = parabola->parameterization;

            sample_info->curve_params = (void *)parabola;
            sample_info->curve_eval_func = prc_evaluate_parabola;
            sample_info->start = params.interval.min_value;
            sample_info->end = params.interval.max_value;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Line:
        {
            prc_crv_line *line = ptr_curve->crv_line;
            prc_parameterization params = line->parameterization;

            sample_info->curve_params = (void *)line;
            sample_info->curve_eval_func = prc_evaluate_line;
            sample_info->start = params.interval.min_value;
            sample_info->end = params.interval.max_value;
            sample_info->num_samples = 2;
            break;
        }

        case PRC_TYPE_CRV_Hyperbola:
        {
            prc_crv_hyperbola *hyperbola = ptr_curve->crv_hyperbola;
            prc_parameterization params = hyperbola->parameterization;

            sample_info->curve_params = (void *)hyperbola;
            sample_info->curve_eval_func = prc_evaluate_hyperbola;
            sample_info->start = params.interval.min_value;
            sample_info->end = params.interval.max_value;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Circle:
        {
            prc_crv_circle *circle = ptr_curve->crv_circle;
            prc_parameterization params = circle->parameterization;

            sample_info->curve_params = (void *)circle;
            sample_info->curve_eval_func = prc_evaluate_circle;
            sample_info->start = params.interval.min_value;
            sample_info->end = params.interval.max_value;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Ellipse:
        {
            prc_crv_ellipse *ellipse = ptr_curve->crv_ellipse;
            prc_parameterization params = ellipse->parameterization;

            sample_info->curve_params = (void *)ellipse;
            sample_info->curve_eval_func = prc_evaluate_ellipse;
            sample_info->start = params.interval.min_value;
            sample_info->end = params.interval.max_value;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_TYPE_CRV_Helix01:
        {
            prc_crv_helix01 *helix = ptr_curve->crv_helix01;
            prc_parameterization params = helix->parameterization;

            sample_info->curve_params = (void *)helix;
            sample_info->curve_eval_func = prc_evaluate_helix;
            sample_info->start = params.interval.min_value;
            sample_info->end = params.interval.max_value;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        /* A series of straight line segments */
        case PRC_TYPE_CRV_PolyLine:
        {
            prc_crv_polyline *polyline = ptr_curve->crv_polyline;

            sample_info->curve_params = (void *)polyline;
            sample_info->curve_eval_func = prc_evaluate_polyline;
            sample_info->start = 0.0;
            sample_info->end = (double)(polyline->number_of_points - 1);
            sample_info->num_samples = polyline->number_of_points;
            break;
        }

        case PRC_TYPE_CRV_Offset:
        {
            prc_crv_offset *offset = ptr_curve->crv_offset;
            prc_ptr_curve base_curve = offset->base_curve;

            /* Make sure the base_curve is NOT PRC_TYPE_CRV_Offset to avoid
               deep recursions */
            if (base_curve.curve_type == PRC_TYPE_CRV_Offset)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_curve_sample_info\n");
                return PRC_ERROR_INTERNAL;
            }

            /* Sample info returns with details for the base_curve */
            code = prc_get_curve_sample_info(ctx, &base_curve, sample_info);
            if (code < 0)
            {
                return code;
            }
            break;
        }

        case PRC_TYPE_CRV_Composite:
        {
            prc_crv_composite *composite = ptr_curve->crv_composite;

            sample_info->curve_params = (void *)composite;
            sample_info->curve_eval_func = prc_evaluate_composite;
            sample_info->start = 0.0;
            sample_info->end = (double)(composite->number_of_subcurves);
            sample_info->num_samples = (composite->number_of_subcurves > 0) ?
                (composite->number_of_subcurves * 2U + 1U) : 1U;
            break;
        }

        case PRC_TYPE_CRV_OnSurf:
        {
            prc_crv_onsurf *onsurf = ptr_curve->crv_onsurf;
            prc_curve_sampling_info base_curve_sample_info;
            prc_surface_sampling_info surface_sample_info;
            surface_func surf_eval_func = NULL;
            void *surf_eval_params = NULL;
            prc_surface_sampling_info surface_samp_info;

            sample_info->curve_params = (void *)onsurf;
            sample_info->curve_eval_func = prc_evaluate_onsurf;

            /* Get details of base curve sample type */
            code = prc_get_curve_sample_info(ctx, &onsurf->uv_curve, &base_curve_sample_info);
            if (code < 0)
            {
                return code;
            }
            onsurf->base_curve_func = base_curve_sample_info.curve_eval_func;
            onsurf->base_curve_params = base_curve_sample_info.curve_params;
            sample_info->start = base_curve_sample_info.start;
            sample_info->end = base_curve_sample_info.end;
            sample_info->num_samples = base_curve_sample_info.num_samples;

            /* Get details on surface sample type. This sets up the matrix */
            code = prc_get_surface_data(ctx, &onsurf->surface.surface, &surface_samp_info);
            if (code < 0)
            {
                return code;
            }

            code = prc_get_surface_eval_func(ctx, &onsurf->surface.surface,
                &surf_eval_func, &surf_eval_params);
            if (code < 0)
            {
                return code;
            }
            onsurf->base_surface_func = surf_eval_func;
            onsurf->base_surface_params = surf_eval_params;
            break;
        }

        default:
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_curve_sample_info\n");
            return PRC_ERROR_INTERNAL;
        }
    }
    return 0;
}
/* Sample curve but dealing with the compressed curve case. It would be
   nice to reduce replicated code with the non-compressed case */
static int
prc_sample_compressed_curve(prc_context *ctx, prc_data *data, uint32_t shell_index,
    uint32_t face_index, prc_compressed_curve *curve, double curve_tolerance)
{
    uint32_t geom_count = data->exact_geom_tess_part_count;
    uint32_t file_index = data->exact_geom_tess_part[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess_part[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess_part[geom_count].body_index;
    uint8_t curve_approx_good = 0;
    void *curve_params = NULL;
    curve_func curve_eval_func = NULL;
    double start;
    double end;
    uint32_t i;
    double t, t0, t1, dist;
    prc_vec3 p0, p1, mid, seg_mid;
    uint32_t num_samples;
    prc_curve_sampling_info sample_info;
    prc_exact_geom_transform exact_geom_trans;
    prc_trans_3d transform;
    int code;
    double tolerance = fmax(CURVE_PRECISION, curve_tolerance);

    code = prc_get_compressed_curve_sample_info(ctx, curve, &sample_info);
    if (code < 0)
    {
        return code;
    }
    start = sample_info.start;
    end = sample_info.end;
    num_samples = sample_info.num_samples;
    curve_params = sample_info.curve_params;
    curve_eval_func = sample_info.curve_eval_func;

    exact_geom_trans.is_identity = 1;
    transform.behavior = 0;

    if (curve_eval_func == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid curve evaluation function in prc_sample_curve\n");
        return PRC_ERROR_INTERNAL;
    }

    while (!curve_approx_good)
    {
        curve_approx_good = 1;

        for (i = 0; i < num_samples - 1; i++)
        {
            t0 = start + (end - start) * ((double)i / (double)(num_samples - 1));
            t1 = start + (end - start) * ((double)(i + 1) / (double)(num_samples - 1));
            p0 = curve_eval_func(ctx, curve_params, t0);
            p1 = curve_eval_func(ctx, curve_params, t1);
            mid = curve_eval_func(ctx, curve_params, (t0 + t1) / 2.0);

            /* Evaluate the midpoint of the segment */
            seg_mid;
            seg_mid.x = (p0.x + p1.x) / 2.0;
            seg_mid.y = (p0.y + p1.y) / 2.0;
            seg_mid.z = (p0.z + p1.z) / 2.0;

            /* Calculate the distance from the midpoint to the curve. Use the encoded
               curve tolerance as the actual acceptance threshold, with a small flooring
               value to avoid zero-tolerance degenerate cases. */
            dist = sqrt((mid.x - seg_mid.x) * (mid.x - seg_mid.x) +
                (mid.y - seg_mid.y) * (mid.y - seg_mid.y) +
                (mid.z - seg_mid.z) * (mid.z - seg_mid.z));
            {
                if (dist > tolerance)
                {
                    curve_approx_good = 0;
                    break;
                }
            }
        }
        if (!curve_approx_good)
        {
            if (num_samples >= (1u << 20))
            {
                prc_error(ctx, PRC_ERROR_INTERNAL,
                    "Compressed curve approximation failed to converge in prc_sample_compressed_curve\n");
                return PRC_ERROR_INTERNAL;
            }
            num_samples *= 2;
        }
    }

    /* We now have a sufficient precision on the curve. Lets generate the
       XYZ sample points and store them */
    data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].wire_data =
        (prc_exact_geom_wire_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_wire_data));
    if (data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].wire_data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of wire_data in prc_sample_curve\n");
        return PRC_ERROR_MEMORY;
    }

    prc_exact_geom_wire_data *wire_data = data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].wire_data;
    wire_data->number_of_points = num_samples;
    wire_data->points = (prc_vec3 *)prc_calloc(ctx, num_samples, sizeof(prc_vec3));
    if (wire_data->points == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of wire_data points in prc_sample_curve\n");
        prc_free(ctx, wire_data);
        return PRC_ERROR_MEMORY;
    }

    for (i = 0; i < num_samples; i++)
    {
        t = start + (end - start) * ((double)i / (double)(num_samples - 1));
        wire_data->points[i] = curve_eval_func(ctx, curve_params, t);
    }
    return 0;
}

static int
prc_sample_curve(prc_context *ctx, prc_content_wire_edge *curve,
    prc_exact_geom_wire_data *wire_data)
{
    uint8_t curve_approx_good = 0;
    void *curve_params = NULL;
    curve_func curve_eval_func = NULL;
    double start;
    double end;
    uint32_t i;
    double t, t0, t1, dist;
    prc_vec3 p0, p1, mid, seg_mid;
    uint32_t num_samples;
    prc_exact_geom_transform *exact_geom_trans = NULL;
    prc_trans_3d *transform = NULL;
    prc_curve_sampling_info sample_info;
    int code;

    code = prc_get_curve_sample_info(ctx, &curve->ptr_curve, &sample_info);
    if (code < 0)
    {
        return code;
    }
    start = sample_info.start;
    end = sample_info.end;
    num_samples = sample_info.num_samples;
    curve_params = sample_info.curve_params;
    curve_eval_func = sample_info.curve_eval_func;

    if (curve->is_trimmed)
    {
        if (curve->trim_interval.min_value > start)
        {
            start = curve->trim_interval.min_value;
        }
        if (curve->trim_interval.max_value < end)
        {
            end = curve->trim_interval.max_value;
        }
    }

    switch (curve->ptr_curve.curve_type)
    {
        case PRC_TYPE_CRV_NURBS:
        {
            prc_crv_nurbs *nurbs = curve->ptr_curve.crv_nurbs;

            break;
        }

        case PRC_TYPE_CRV_Parabola:
        {
            prc_crv_parabola *parabola = curve->ptr_curve.crv_parabola;
            prc_parameterization params = parabola->parameterization;

            exact_geom_trans = &parabola->exact_geom_transform;
            transform = &parabola->transform;
            break;
        }

        case PRC_TYPE_CRV_Line:
        {
            prc_crv_line *line = curve->ptr_curve.crv_line;
            prc_parameterization params = line->parameterization;

            exact_geom_trans = &line->exact_geom_transform;
            transform = &line->transform;
            break;
        }

        case PRC_TYPE_CRV_Hyperbola:
        {
            prc_crv_hyperbola *hyperbola = curve->ptr_curve.crv_hyperbola;
            prc_parameterization params = hyperbola->parameterization;

            exact_geom_trans = &hyperbola->exact_geom_transform;
            transform = &hyperbola->transform;
            break;
        }

        case PRC_TYPE_CRV_Circle:
        {
            prc_crv_circle *circle = curve->ptr_curve.crv_circle;
            prc_parameterization params = circle->parameterization;

            exact_geom_trans = &circle->exact_geom_transform;
            transform = &circle->transform;
            break;
        }

        case PRC_TYPE_CRV_Ellipse:
        {
            prc_crv_ellipse *ellipse = curve->ptr_curve.crv_ellipse;
            prc_parameterization params = ellipse->parameterization;

            exact_geom_trans = &ellipse->exact_geom_transform;
            transform = &ellipse->transform;
            break;
        }

        case PRC_TYPE_CRV_Helix01:
        {
            prc_crv_helix01 *helix = curve->ptr_curve.crv_helix01;
            prc_parameterization params = helix->parameterization;

            exact_geom_trans = &helix->exact_geom_transform;
            transform = &helix->transform;
            break;
        }

        /* A series of straight line segments */
        case PRC_TYPE_CRV_PolyLine:
        {
            prc_crv_polyline *polyline = curve->ptr_curve.crv_polyline;

            exact_geom_trans = &polyline->exact_geom_transform;
            transform = &polyline->transform;
            break;
        }

        case PRC_TYPE_CRV_Offset:
        {
            prc_crv_offset *offset = curve->ptr_curve.crv_offset;

            /* Set the base one in the params so it can be used in
               prc_evaluate_offset_curve */
            offset->base_func = sample_info.curve_eval_func;
            offset->base_params = sample_info.curve_params;
            curve_eval_func = prc_evaluate_offset_curve;
            curve_params = (void*) offset;
            break;
        }

        case PRC_TYPE_CRV_OnSurf:
        {
            prc_crv_onsurf *onsurf = curve->ptr_curve.crv_onsurf;

            exact_geom_trans = &onsurf->exact_geom_transform;
            transform = &onsurf->transform;
            break;
        }

        case PRC_TYPE_CRV_Composite:
        {
            prc_crv_composite *composite = curve->ptr_curve.crv_composite;
            uint32_t num_sub_curves = composite->number_of_subcurves;
            uint32_t k;

            /* Get the needed data to evaluate each of the subcurves and determine
               how many samples each needs to achieve the same curvature tolerance. */
            for (k = 0; k < num_sub_curves; k++)
            {
                prc_composite_subcurve *sub_curve = &composite->subcurves[k];
                prc_curve_sampling_info sub_curve_info;
                curve_func subcurve_eval_func;
                void *subcurve_params;
                uint32_t subcurve_num_samples;
                uint8_t subcurve_approx_good = 0;
                uint32_t j;

                memset(&sub_curve_info, 0, sizeof(sub_curve_info));
                code = prc_get_curve_sample_info(ctx, &sub_curve->ptr_curve, &sub_curve_info);
                if (code < 0)
                {
                    return code;
                }

                subcurve_eval_func = sub_curve_info.curve_eval_func;
                subcurve_params = sub_curve_info.curve_params;
                subcurve_num_samples = sub_curve_info.num_samples;

                switch (sub_curve->ptr_curve.curve_type)
                {
                    case PRC_TYPE_CRV_Line:
                        subcurve_num_samples = 2;
                        break;

                    case PRC_TYPE_CRV_PolyLine:
                        subcurve_num_samples = (uint32_t)((prc_crv_polyline *)sub_curve->ptr_curve.crv_polyline)->number_of_points;
                        if (subcurve_num_samples < 2)
                            subcurve_num_samples = 2;
                        break;

                    default:
                        if (subcurve_num_samples == 0)
                            subcurve_num_samples = 2;

                        while (!subcurve_approx_good)
                        {
                            subcurve_approx_good = 1;
                            for (j = 0; j < subcurve_num_samples - 1; j++)
                            {
                                double t0 = sub_curve_info.start +
                                    (sub_curve_info.end - sub_curve_info.start) *
                                    ((double)j / (double)(subcurve_num_samples - 1));
                                double t1 = sub_curve_info.start +
                                    (sub_curve_info.end - sub_curve_info.start) *
                                    ((double)(j + 1) / (double)(subcurve_num_samples - 1));
                                prc_vec3 p0 = subcurve_eval_func(ctx, subcurve_params, t0);
                                prc_vec3 p1 = subcurve_eval_func(ctx, subcurve_params, t1);
                                prc_vec3 mid = subcurve_eval_func(ctx, subcurve_params, (t0 + t1) / 2.0);
                                prc_vec3 seg_mid = { 0.0, 0.0, 0.0 };
                                double dist;

                                seg_mid.x = (p0.x + p1.x) / 2.0;
                                seg_mid.y = (p0.y + p1.y) / 2.0;
                                seg_mid.z = (p0.z + p1.z) / 2.0;

                                dist = sqrt((mid.x - seg_mid.x) * (mid.x - seg_mid.x) +
                                    (mid.y - seg_mid.y) * (mid.y - seg_mid.y) +
                                    (mid.z - seg_mid.z) * (mid.z - seg_mid.z));
                                if (dist > CURVE_PRECISION)
                                {
                                    subcurve_approx_good = 0;
                                    break;
                                }
                            }
                            if (!subcurve_approx_good)
                            {
                                subcurve_num_samples *= 2;
                            }
                        }
                        break;
                }

                sub_curve->base_func = sub_curve_info.curve_eval_func;
                sub_curve->base_params = sub_curve_info.curve_params;
                sub_curve->num_samples = subcurve_num_samples;
            }

            /* Use the aggregate per-subcurve sample budget for the composite's global sample count. */
            num_samples = 0;
            for (k = 0; k < num_sub_curves; k++)
            {
                num_samples += composite->subcurves[k].num_samples;
            }
            if (num_samples == 0)
                num_samples = 1;
            curve_approx_good = 1;
            break;
        }

        default:
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid curve type in prc_sample_curve\n");
            return PRC_ERROR_INTERNAL;
    }

    if (exact_geom_trans != NULL && transform != NULL)
    {
        code = prc_exact_geom_set_transform(ctx, exact_geom_trans, transform);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_exact_geom_set_transform\n");
            return code;
        }
    }

    /* Create a set of samples across the range, evaluate the curve
     * then evaluate at the midpoint of each segment and see if the distance is within tolerance.
     * If not, subdivide the segment and repeat until we have a good set of samples.
     * Composite curves are piecewise-defined; the midpoint test is not valid across subcurve joins,
     * so their sample count is based on the sum of the subcurve sample counts instead of global
     * recursive subdivision. */

    if (curve_eval_func == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid curve evaluation function in prc_sample_curve\n");
        return PRC_ERROR_INTERNAL;
    }

    if (curve->ptr_curve.curve_type == PRC_TYPE_CRV_Composite)
    {
        curve_approx_good = 1;
    }
    else
    {
        while (!curve_approx_good)
        {
            curve_approx_good = 1;

            for (i = 0; i < num_samples - 1; i++)
            {
                t0 = start + (end - start) * ((double)i / (double)(num_samples - 1));
                t1 = start + (end - start) * ((double)(i + 1) / (double)(num_samples - 1));
                p0 = curve_eval_func(ctx, curve_params, t0);
                p1 = curve_eval_func(ctx, curve_params, t1);
                mid = curve_eval_func(ctx, curve_params, (t0 + t1) / 2.0);

                /* Evaluate the midpoint of the segment */
                seg_mid;
                seg_mid.x = (p0.x + p1.x) / 2.0;
                seg_mid.y = (p0.y + p1.y) / 2.0;
                seg_mid.z = (p0.z + p1.z) / 2.0;

                /* Calculate the distance from the midpoint to the curve */
                dist = sqrt((mid.x - seg_mid.x) * (mid.x - seg_mid.x) +
                    (mid.y - seg_mid.y) * (mid.y - seg_mid.y) +
                    (mid.z - seg_mid.z) * (mid.z - seg_mid.z));
                if (dist > CURVE_PRECISION)
                {
                    curve_approx_good = 0;
                    break;
                }
            }
            if (!curve_approx_good)
            {
                num_samples *= 2;
            }
        }
    }

    /* We now have a sufficient precision on the curve. Lets generate the
       XYZ sample points and store them */
    wire_data->number_of_points = num_samples;
    wire_data->points = (prc_vec3 *)prc_calloc(ctx, num_samples, sizeof(prc_vec3));
    if (wire_data->points == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of wire_data points in prc_sample_curve\n");
        return PRC_ERROR_MEMORY;
    }

    if (curve->ptr_curve.curve_type == PRC_TYPE_CRV_Composite)
    {
        prc_crv_composite *composite = curve->ptr_curve.crv_composite;
        uint32_t point_index = 0;

        for (i = 0; i < composite->number_of_subcurves; i++)
        {
            prc_composite_subcurve *sub_curve = &composite->subcurves[i];
            uint32_t j;
            uint32_t sub_count = sub_curve->num_samples;

            if (sub_count == 0)
                sub_count = 2;

            for (j = 0; j < sub_count; j++)
            {
                double sub_t = (sub_count > 1) ?
                    ((double)j / (double)(sub_count - 1)) : 0.0;
                double composite_t = (double)i + sub_t;

                wire_data->points[point_index++] = curve_eval_func(ctx, curve_params, composite_t);
            }
        }
    }
    else
    {
        for (i = 0; i < num_samples; i++)
        {
            t = start + (end - start) * ((double)i / (double)(num_samples - 1));
            wire_data->points[i] = curve_eval_func(ctx, curve_params, t);
        }
    }

#if 0
    /* Lets print the points for a sanity check */
    for (uint32_t i = 0; i < num_samples; i++)
    {
        prc_vec3 p = data->exact_geom_tess[geom_count].wire_data->points[i];
        printf("prc_sample_curve: point[%u] = (%f, %f, %f)\n", i, p.x, p.y, p.z);
    }
#endif

    return 0;
}

/* Get a base curve eval function and params and limits */
static int
prc_get_curve_eval_func(prc_context *ctx, prc_ptr_curve *curve,
                        curve_func *eval_func, void **params, double *min_u,
                        double *max_u)
{
    prc_parameterization params_interval;
    prc_exact_geom_transform *exact_geom_trans = NULL;
    prc_trans_3d *transform = NULL;
    int code;

    memset(&params_interval, 0, sizeof(prc_parameterization));

    switch (curve->curve_type)
    {
        case PRC_TYPE_CRV_Parabola:
            *eval_func = prc_evaluate_parabola;
            *params = (void *)curve->crv_parabola;
            params_interval = curve->crv_parabola->parameterization;
            if (curve->crv_parabola->has_transform)
            {
                exact_geom_trans = &curve->crv_parabola->exact_geom_transform;
                transform = &curve->crv_parabola->transform;
            }
            break;
        case PRC_TYPE_CRV_Line:
            *eval_func = prc_evaluate_line;
            *params = (void *)curve->crv_line;
            params_interval = curve->crv_line->parameterization;
            if (curve->crv_line->has_transform)
            {
                exact_geom_trans = &curve->crv_line->exact_geom_transform;
                transform = &curve->crv_line->transform;
            }
            break;
        case PRC_TYPE_CRV_Hyperbola:
            *eval_func = prc_evaluate_hyperbola;
            *params = (void *)curve->crv_hyperbola;
            params_interval = curve->crv_hyperbola->parameterization;
            if (curve->crv_hyperbola->has_transform)
            {
                exact_geom_trans = &curve->crv_hyperbola->exact_geom_transform;
                transform = &curve->crv_hyperbola->transform;
            }
            break;
        case PRC_TYPE_CRV_Circle:
            *eval_func = prc_evaluate_circle;
            *params = (void *)curve->crv_circle;
            params_interval = curve->crv_circle->parameterization;
            if (curve->crv_circle->has_transform)
            {
                exact_geom_trans = &curve->crv_circle->exact_geom_transform;
                transform = &curve->crv_circle->transform;
            }
            break;
        case PRC_TYPE_CRV_Ellipse:
            *eval_func = prc_evaluate_ellipse;
            *params = (void *)curve->crv_ellipse;
            params_interval = curve->crv_ellipse->parameterization;
            if (curve->crv_ellipse->has_transform)
            {
                exact_geom_trans = &curve->crv_ellipse->exact_geom_transform;
                transform = &curve->crv_ellipse->transform;
            }
            break;
        case PRC_TYPE_CRV_Helix01:
            *eval_func = prc_evaluate_helix;
            *params = (void *)curve->crv_helix01;
            params_interval = curve->crv_helix01->parameterization;
            if (curve->crv_helix01->has_transform)
            {
                exact_geom_trans = &curve->crv_helix01->exact_geom_transform;
                transform = &curve->crv_helix01->transform;
            }
            break;
        case PRC_TYPE_CRV_PolyLine:
            *eval_func = prc_evaluate_polyline;
            *params = (void *)curve->crv_polyline;
            params_interval = curve->crv_polyline->parameterization;
            if (curve->crv_polyline->has_transform)
            {
                exact_geom_trans = &curve->crv_polyline->exact_geom_transform;
                transform = &curve->crv_polyline->transform;
            }
            break;
        case PRC_TYPE_CRV_NURBS:
            /* prc_crv_nurbs_s has no transform fields and no parameterization interval;
               the valid domain comes directly from the clamped knot vector */
            *eval_func = prc_evaluate_crv_nurbs;
            *params = (void *)curve->crv_nurbs;
            *min_u = curve->crv_nurbs->u[curve->crv_nurbs->d];
            *max_u = curve->crv_nurbs->u[curve->crv_nurbs->highest_index_of_knots - curve->crv_nurbs->d];
            return 0;
        case PRC_TYPE_CRV_Offset:
        {
            prc_crv_offset *offset = curve->crv_offset;
            prc_ptr_curve base_curve = offset->base_curve;
            curve_func base_eval_func = NULL;

            if (base_curve.curve_type == PRC_TYPE_CRV_Offset)
                return PRC_ERROR_INTERNAL;

            code = prc_get_curve_eval_func(ctx, &base_curve, &base_eval_func,
                &offset->base_params, min_u, max_u);
            if (code < 0)
                return code;

            offset->base_func = (void *)base_eval_func;
            *eval_func = prc_evaluate_offset_curve;
            *params = (void *)offset;
            *min_u = offset->parameterization.interval.min_value;
            *max_u = offset->parameterization.interval.max_value;
            return 0;
        }
        default:
            return PRC_ERROR_INTERNAL;
    }

    /* We should probably pull this out of the evaluation TODO */
    if (exact_geom_trans != NULL && transform != NULL)
    {
        code = prc_exact_geom_set_transform(ctx, exact_geom_trans, transform);
        if (code < 0)
        {
            prc_error(ctx, code, "Error in prc_exact_geom_set_transform\n");
            return code;
        }
    }
    *min_u = params_interval.interval.min_value;
    *max_u = params_interval.interval.max_value;
    return 0;
}

static prc_vec3
prc_evaluate_surf_torus(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_torus *torus = (prc_surf_torus *)surf_params->surface_params;
    double major_radius = torus->major_radius;
    double minor_radius = torus->minor_radius;
    double radius;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    radius = major_radius + minor_radius * cos(v);

    output.x = radius * cos(u);
    output.y = radius * sin(u);
    output.z = minor_radius * sin(v);

    if (torus->has_transform && !torus->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &torus->exact_geom_transform, output);
    }

    return output;
}

static prc_vec3
prc_evaluate_surf_fromcurves(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_fromcurves *surf = (prc_surf_fromcurves *)surf_params->surface_params;
    curve_func curve1 = NULL;
    void *curve1_params = NULL;
    double curve1_max_u = 0.0;
    double curve1_min_u = 0.0;
    curve_func curve2 = NULL;
    void *curve2_params = NULL;
    double curve2_max_u = 0.0;
    double curve2_min_u = 0.0;
    int code;
    prc_vec3 origin = surf->origin;
    prc_vec3 curve1_point, curve2_point, temp;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    /* We need to take into account the curve parameterization */
    /* Lets get the base evaluation surface function.  We probably should
       do a 1-D curve sample here to get a good approximation of the curve. ToDo. */
    code = prc_get_curve_eval_func(ctx, &surf->first_curve,
        &curve1, &curve1_params, &curve1_min_u, &curve1_max_u);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_curve_eval_func\n");
        output.x = 0;
        output.y = 0;
        output.z = 0;
        return output;
    }

    code = prc_get_curve_eval_func(ctx, &surf->second_curve,
        &curve2, &curve2_params, &curve2_min_u, &curve2_max_u);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_curve_eval_func\n");
        output.x = 0;
        output.y = 0;
        output.z = 0;
        return output;
    }

    /* We need to apply any parametrization to the base_curve_params */
    if (u < curve1_min_u)
        u = curve1_min_u;
    if (u > curve1_max_u)
        u = curve1_max_u;
    curve1_point = curve1(ctx, curve1_params, u);

    if (v < curve2_min_u)
        v = curve2_min_u;
    if (v > curve2_max_u)
        v = curve2_max_u;
    curve2_point = curve2(ctx, curve2_params, v);

    prc_vec_add(curve1_point, curve2_point, &temp);
    prc_vec_sub(temp, origin, &output);

    if (surf->has_transform && !surf->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &surf->exact_geom_transform, output);
    }
    return output;
}

static prc_vec3
prc_evaluate_surf_cone(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_cone *cone = (prc_surf_cone *)surf_params->surface_params;
    double bottom_radius = cone->radius;
    double semi_angle = cone->semi_angle;
    double radius;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    radius = bottom_radius + v * tan(semi_angle);
    output.x = radius * cos(u);
    output.y = radius * sin(u);
    output.z = v;

    if (cone->has_transform && !cone->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &cone->exact_geom_transform, output);
    }
    
    return output;
}

static prc_vec3
prc_evaluate_surf_sphere(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_sphere *sphere = (prc_surf_sphere *)surf_params->surface_params;
    double radius = sphere->radius;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    output.x = radius * cos(v) * cos(u);
    output.y = radius * cos(v) * sin(u);
    output.z = radius * sin(v);

    if (sphere->has_transform && !sphere->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &sphere->exact_geom_transform, output);
    }

    return output;
}

static prc_vec3
prc_evaluate_surf_cylinder(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_cylinder *cylinder = (prc_surf_cylinder *)surf_params->surface_params;
    double radius = cylinder->radius;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    output.x = radius * cos(u);
    output.y = radius * sin(u);
    output.z = v;

    if (cylinder->has_transform && !cylinder->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &cylinder->exact_geom_transform, output);
    }

    return output;
}

static prc_vec3
prc_evaluate_surf_plane(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_plane *plane = (prc_surf_plane *)surf_params->surface_params;
    double u_parameter_coeff_a = plane->u_parameter_coeff_a;
    double v_parameter_coeff_a = plane->v_parameter_coeff_a;
    double u_parameter_coeff_b = plane->u_parameter_coeff_b;
    double v_parameter_coeff_b = plane->v_parameter_coeff_b;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    output.x = u * u_parameter_coeff_a + u_parameter_coeff_b;
    output.y = v * v_parameter_coeff_a + v_parameter_coeff_b;
    output.z = 0;

    if (!plane->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &plane->exact_geom_transform, output);
    }
    return output;
}

static prc_vec3
prc_evaluate_surf_nurbs(prc_context *ctx, void *params, double u, double v)
{
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_nurbs *nurbs = (prc_surf_nurbs *)surf_params->surface_params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    double Nu[PRC_BSPLINE_MAX_DEGREE + 1];
    double Nv[PRC_BSPLINE_MAX_DEGREE + 1];
    uint32_t num_ctrl_u = nurbs->highest_index_of_control_points_u + 1;
    uint32_t num_ctrl_v = nurbs->highest_index_of_control_points_v + 1;
    uint32_t span_u, span_v, i, j;
    double x = 0.0, y = 0.0, z = 0.0, weight_sum = 0.0;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    if (nurbs->du > PRC_BSPLINE_MAX_DEGREE || nurbs->dv > PRC_BSPLINE_MAX_DEGREE)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "NURBS surface degree exceeds supported maximum\n");
        return output;
    }

    span_u = prc_bspline_find_span(nurbs->du, num_ctrl_u, u, nurbs->knot_vector_u);
    span_v = prc_bspline_find_span(nurbs->dv, num_ctrl_v, v, nurbs->knot_vector_v);
    prc_bspline_basis_funs(span_u, u, nurbs->du, nurbs->knot_vector_u, Nu);
    prc_bspline_basis_funs(span_v, v, nurbs->dv, nurbs->knot_vector_v, Nv);

    /* Control points are stored u-major: index = ctrl_u * num_ctrl_v + ctrl_v */
    for (i = 0; i <= nurbs->du; i++)
    {
        uint32_t ctrl_u = span_u - nurbs->du + i;

        for (j = 0; j <= nurbs->dv; j++)
        {
            uint32_t ctrl_v = span_v - nurbs->dv + j;
            prc_control_points_nurbs_surf *cp = &nurbs->p[ctrl_u * num_ctrl_v + ctrl_v];
            double weight = nurbs->is_rational ? cp->w : 1.0;
            double basis = Nu[i] * Nv[j];  /* Weight is already applied to these */

            x += basis * cp->x;
            y += basis * cp->y;
            z += basis * cp->z;
            weight_sum += basis * weight; /* Denominator does need the weight */
        }
    }

    if (weight_sum != 0.0)
    {
        output.x = x / weight_sum;
        output.y = y / weight_sum;
        output.z = z / weight_sum;
    }

    return output;
}

/* This one uses a base curve */
static prc_vec3
prc_evaluate_surf_extrusion(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output, base_point, temp;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_extrusion *extrusion = (prc_surf_extrusion *)surf_params->surface_params;
    curve_func base_curve_func = NULL;
    void *base_curve_params = NULL;
    prc_vec3 sweep_vector = extrusion->sweep_vector;
    double curve_max_u = 0.0;
    double curve_min_u = 0.0;
    int code;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    /* We need to take into account the curve parameterization */
    /* Lets get the base evaluation surface function.  We probably should
       do a 1-D curve sample here to get a good approximation of the curve. ToDo. */
    code = prc_get_curve_eval_func(ctx, &extrusion->base_curve,
        &base_curve_func, &base_curve_params, &curve_min_u, &curve_max_u);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_curve_eval_func\n");
        output.x = 0;
        output.y = 0;
        output.z = 0;
        return output;
    }

    /* We need to apply any parametrization to the base_curve_params */
    if (u < curve_min_u)
        u = curve_min_u;
    if (u > curve_max_u)
        u = curve_max_u;
    base_point = base_curve_func(ctx, base_curve_params, u);
    prc_vec_scale(v, &sweep_vector);
    prc_vec_add(base_point, sweep_vector, &output);

    if (extrusion->has_transform && !extrusion->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &extrusion->exact_geom_transform, output);
    }

    return output;
}

/* This one also uses a base curve */
static prc_vec3
prc_evaluate_surf_revolution(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output, base_point;
    prc_surface_params *surf_params = (prc_surface_params *)params;
    prc_surf_revolution *revolution = (prc_surf_revolution *)surf_params->surface_params;
    curve_func base_curve_func = NULL;
    void *base_curve_params = NULL;
    double curve_max_u = 0.0;
    double curve_min_u = 0.0;
    int code;
    prc_vec3 origin = revolution->origin;
    prc_vec3 x_axis = revolution->x_axis;
    prc_vec3 y_axis = revolution->y_axis;
    prc_vec3 axis_of_revolution;
    prc_vec3 point_on_axis;
    prc_vec3 temp_axis_x, temp_axis_y;
    prc_vec3 temp1;
    uint32_t num_loops = surf_params->num_loops;
    prc_loop_samples *loops = surf_params->loop_samples;

    /* We need to take into account the curve parameterization */
    /* Lets get the base evaluation surface function.  We probably should
       do a 1-D curve sample here to get a good approximation of the curve. ToDo. */
    code = prc_get_curve_eval_func(ctx, &revolution->base_curve,
        &base_curve_func, &base_curve_params, &curve_min_u, &curve_max_u);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_curve_eval_func\n");
        output.x = 0;
        output.y = 0;
        output.z = 0;
        return output;
    }

    /* We need to apply any parametrization to the base_curve_params */
    if (v < curve_min_u)
        v = curve_min_u;
    if (v > curve_max_u)
        v = curve_max_u;
    base_point = base_curve_func(ctx, base_curve_params, v);

    /* axis_of_revolution is found from the origin and cross product of x_axis and y_axis */
    prc_vec_cross(x_axis, y_axis, &axis_of_revolution);

    point_on_axis = prc_project_point_onto_line(ctx, base_point, origin, axis_of_revolution);
    prc_vec_sub(base_point, point_on_axis, &temp_axis_x);
    prc_vec_cross(axis_of_revolution, temp_axis_x, &temp_axis_y);
    prc_vec_scale(cos(u), &temp_axis_x);
    prc_vec_scale(sin(u), &temp_axis_y);
    prc_vec_add(temp_axis_x, temp_axis_y, &temp1);
    prc_vec_add(point_on_axis, temp1, &output);

    if (revolution->has_transform && !revolution->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &revolution->exact_geom_transform, output);
    }

    return output;
}

/* Keep this one at the bottom as it may call the other evaluate methods above */
/* Maps a base/bound prc_type_surf to its evaluation function and params. Only
   the surface kinds that already have an evaluate function implemented are supported. */
static int
prc_get_surface_eval_func(prc_context *ctx, prc_type_surf *surface,
    surface_func *eval_func, void **params)
{
    switch (surface->surface_type)
    {
        case PRC_TYPE_SURF_Cylinder:
            *eval_func = prc_evaluate_surf_cylinder;
            *params = (void *)surface->surf_cylinder;
            break;
        case PRC_TYPE_SURF_Cone:
            *eval_func = prc_evaluate_surf_cone;
            *params = (void *)surface->surf_cone;
            break;
        case PRC_TYPE_SURF_Sphere:
            *eval_func = prc_evaluate_surf_sphere;
            *params = (void *)surface->surf_sphere;
            break;
        case PRC_TYPE_SURF_Torus:
            *eval_func = prc_evaluate_surf_torus;
            *params = (void *)surface->surf_torus;
            break;
        case PRC_TYPE_SURF_Extrusion:
            *eval_func = prc_evaluate_surf_extrusion;
            *params = (void *)surface->surf_extrusion;
            break;
        case PRC_TYPE_SURF_Revolution:
            *eval_func = prc_evaluate_surf_revolution;
            *params = (void *)surface->surf_revolution;
            break;
        case PRC_TYPE_SURF_FromCurves:
            *eval_func = prc_evaluate_surf_fromcurves;
            *params = (void *)surface->surf_fromcurves;
            break;
        case PRC_TYPE_SURF_Plane:
            *eval_func = prc_evaluate_surf_plane;
            *params = (void *)surface->surf_plane;
            break;
        case PRC_TYPE_SURF_NURBS:
            *eval_func = prc_evaluate_surf_nurbs;
            *params = (void *)surface->surf_nurbs;
            break;
        default:
            return PRC_ERROR_INTERNAL;
    }
    return 0;
}

/* Minimizes the squared distance from point to curve_eval_func(t) via coarse sampling
   followed by Newton refinement on finite-difference derivatives of the squared distance */
static prc_vec3
prc_project_point_onto_curve(prc_context *ctx, curve_func eval_func, void *curve_params,
    double min_u, double max_u, prc_vec3 point)
{
    const uint32_t coarse_samples = 64;
    const uint32_t max_iterations = 30;
    double h = (max_u - min_u) * 1e-5;
    double best_t = min_u;
    double best_dist_sq = -1.0;
    uint32_t i;
    double t;

    if (h < 1e-9)
        h = 1e-9;

    for (i = 0; i < coarse_samples; i++)
    {
        prc_vec3 p;
        double dx, dy, dz, dist_sq;

        t = min_u + (max_u - min_u) * ((double)i / (double)(coarse_samples - 1));
        p = eval_func(ctx, curve_params, t);
        dx = p.x - point.x;
        dy = p.y - point.y;
        dz = p.z - point.z;
        dist_sq = dx * dx + dy * dy + dz * dz;
        if (best_dist_sq < 0.0 || dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_t = t;
        }
    }

    t = best_t;
    for (i = 0; i < max_iterations; i++)
    {
        prc_vec3 p, p_plus, p_minus;
        double t_plus = t + h;
        double t_minus = t - h;
        double dx, dy, dz, dxp, dyp, dzp, dxm, dym, dzm;
        double f, f_plus, f_minus, f_prime, f_double_prime, delta;

        if (t_plus > max_u)
            t_plus = max_u;
        if (t_minus < min_u)
            t_minus = min_u;

        p = eval_func(ctx, curve_params, t);
        p_plus = eval_func(ctx, curve_params, t_plus);
        p_minus = eval_func(ctx, curve_params, t_minus);

        dx = p.x - point.x; dy = p.y - point.y; dz = p.z - point.z;
        f = dx * dx + dy * dy + dz * dz;
        dxp = p_plus.x - point.x; dyp = p_plus.y - point.y; dzp = p_plus.z - point.z;
        f_plus = dxp * dxp + dyp * dyp + dzp * dzp;
        dxm = p_minus.x - point.x; dym = p_minus.y - point.y; dzm = p_minus.z - point.z;
        f_minus = dxm * dxm + dym * dym + dzm * dzm;

        f_prime = (f_plus - f_minus) / (t_plus - t_minus);
        f_double_prime = (f_plus - 2.0 * f + f_minus) / (h * h);

        if (fabs(f_double_prime) < 1e-12)
            break;

        delta = f_prime / f_double_prime;
        t -= delta;
        if (t < min_u)
            t = min_u;
        if (t > max_u)
            t = max_u;

        if (fabs(delta) < 1e-12)
            break;
    }

    return eval_func(ctx, curve_params, t);
}

/* Gauss-Newton refinement of (u, v) (tangents from finite differences) starting
   from a caller-supplied guess, minimizing the squared distance from point to
   surface_eval_func(u,v). No coarse global search: callers that already have a
   good starting guess -- e.g. the previously projected sample along the same
   loop -- should prefer this over prc_project_point_onto_surface, since a fresh
   global coarse search per sample can converge to a different, equally valid
   but discontinuous (u,v) branch whenever the surface nearly meets itself (e.g.
   a NURBS cylinder's own seam, where u near min_u and u near max_u map to
   nearly the same 3D point); Newton's local convergence instead stays on
   whichever branch the guess is already on, keeping a sampled loop continuous
   in uv space */
static prc_vec3
prc_refine_point_on_surface(prc_context *ctx, surface_func eval_func, void *surface_params,
    double min_u, double max_u, double min_v, double max_v, prc_vec3 point,
    double u0, double v0, double *out_u, double *out_v)
{
    const uint32_t max_iterations = 30;
    double hu = (max_u - min_u) * 1e-5;
    double hv = (max_v - min_v) * 1e-5;
    double u = u0, v = v0;
    uint32_t iter;

    if (hu < 1e-9)
        hu = 1e-9;
    if (hv < 1e-9)
        hv = 1e-9;

    for (iter = 0; iter < max_iterations; iter++)
    {
        prc_vec3 p, pu_plus, pu_minus, pv_plus, pv_minus;
        prc_vec3 su, sv, d;
        double u_plus = u + hu, u_minus = u - hu;
        double v_plus = v + hv, v_minus = v - hv;
        double g_u, g_v, h_uu, h_vv, h_uv, det, du, dv;

        if (u_plus > max_u)
            u_plus = max_u;
        if (u_minus < min_u)
            u_minus = min_u;
        if (v_plus > max_v)
            v_plus = max_v;
        if (v_minus < min_v)
            v_minus = min_v;

        p = eval_func(ctx, surface_params, u, v);
        pu_plus = eval_func(ctx, surface_params, u_plus, v);
        pu_minus = eval_func(ctx, surface_params, u_minus, v);
        pv_plus = eval_func(ctx, surface_params, u, v_plus);
        pv_minus = eval_func(ctx, surface_params, u, v_minus);

        su.x = (pu_plus.x - pu_minus.x) / (u_plus - u_minus);
        su.y = (pu_plus.y - pu_minus.y) / (u_plus - u_minus);
        su.z = (pu_plus.z - pu_minus.z) / (u_plus - u_minus);

        sv.x = (pv_plus.x - pv_minus.x) / (v_plus - v_minus);
        sv.y = (pv_plus.y - pv_minus.y) / (v_plus - v_minus);
        sv.z = (pv_plus.z - pv_minus.z) / (v_plus - v_minus);

        d.x = p.x - point.x;
        d.y = p.y - point.y;
        d.z = p.z - point.z;

        /* Gauss-Newton step minimizing |S(u,v) - point|^2 */
        g_u = d.x * su.x + d.y * su.y + d.z * su.z;
        g_v = d.x * sv.x + d.y * sv.y + d.z * sv.z;
        h_uu = su.x * su.x + su.y * su.y + su.z * su.z;
        h_vv = sv.x * sv.x + sv.y * sv.y + sv.z * sv.z;
        h_uv = su.x * sv.x + su.y * sv.y + su.z * sv.z;

        det = h_uu * h_vv - h_uv * h_uv;
        if (fabs(det) < 1e-14)
            break;

        du = (g_u * h_vv - g_v * h_uv) / det;
        dv = (g_v * h_uu - g_u * h_uv) / det;

        u -= du;
        v -= dv;
        if (u < min_u) u = min_u;
        if (u > max_u) u = max_u;
        if (v < min_v) v = min_v;
        if (v > max_v) v = max_v;

        if (fabs(du) < 1e-12 && fabs(dv) < 1e-12)
            break;
    }

    if (out_u != NULL)
        *out_u = u;
    if (out_v != NULL)
        *out_v = v;

    return eval_func(ctx, surface_params, u, v);
}

/* Minimizes the squared distance from point to surface_eval_func(u,v) via coarse grid
   sampling followed by Gauss-Newton refinement (prc_refine_point_on_surface).
   out_u/out_v are optional (pass NULL to ignore) and report the (u,v) found, for
   callers that need the surface parameter itself rather than just the projected point */
static prc_vec3
prc_project_point_onto_surface(prc_context *ctx, surface_func eval_func, void *surface_params,
    double min_u, double max_u, double min_v, double max_v, prc_vec3 point,
    double *out_u, double *out_v)
{
    const uint32_t coarse_samples = 12;
    double best_u = min_u, best_v = min_v, best_dist_sq = -1.0;
    uint32_t i, j;
    double u, v;

    for (i = 0; i < coarse_samples; i++)
    {
        u = min_u + (max_u - min_u) * ((double)i / (double)(coarse_samples - 1));
        for (j = 0; j < coarse_samples; j++)
        {
            prc_vec3 p;
            double dx, dy, dz, dist_sq;

            v = min_v + (max_v - min_v) * ((double)j / (double)(coarse_samples - 1));
            p = eval_func(ctx, surface_params, u, v);
            dx = p.x - point.x; dy = p.y - point.y; dz = p.z - point.z;
            dist_sq = dx * dx + dy * dy + dz * dz;
            if (best_dist_sq < 0.0 || dist_sq < best_dist_sq)
            {
                best_dist_sq = dist_sq;
                best_u = u;
                best_v = v;
            }
        }
    }

    return prc_refine_point_on_surface(ctx, eval_func, surface_params,
        min_u, max_u, min_v, max_v, point, best_u, best_v, out_u, out_v);
}

/* Projects center onto whichever of bound_surface/bound_curve is present (exactly one
   shall be non-NULL per the spec). Referenced (shared) entities are not yet supported. */
static int
prc_project_onto_blend_bound(prc_context *ctx, prc_ptr_surface *bound_surface,
    prc_ptr_curve *bound_curve, prc_vec3 center, prc_vec3 *out)
{
    int code;

    if (!bound_surface->is_referenced && bound_surface->surface.surface_type != PRC_TYPE_ROOT)
    {
        surface_func eval_func = NULL;
        void *eval_params = NULL;
        prc_surface_sampling_info sampling_info = { 0 };

        code = prc_get_surface_eval_func(ctx, &bound_surface->surface, &eval_func, &eval_params);
        if (code < 0)
            return code;

        code = prc_get_surface_data(ctx, &bound_surface->surface, &sampling_info);
        if (code < 0)
            return code;

        *out = prc_project_point_onto_surface(ctx, eval_func, eval_params,
            sampling_info.start_u, sampling_info.end_u,
            sampling_info.start_v, sampling_info.end_v, center, NULL, NULL);
        return 0;
    }

    if (!bound_curve->is_referenced && bound_curve->curve_type != PRC_TYPE_ROOT)
    {
        curve_func eval_func = NULL;
        void *eval_params = NULL;
        double min_u, max_u;

        code = prc_get_curve_eval_func(ctx, bound_curve, &eval_func, &eval_params, &min_u, &max_u);
        if (code < 0)
            return code;

        *out = prc_project_point_onto_curve(ctx, eval_func, eval_params, min_u, max_u, center);
        return 0;
    }

    return PRC_ERROR_INTERNAL;
}

/* Note: the spec's additive term for this formula is the center curve, consistent with
   Blend02 and the surface being "centred on the center curve" (origin_curve only supplies R(v)) */
static prc_vec3
prc_evaluate_surf_blend01(prc_context *ctx, void *params, double u, double v)
{
    prc_surf_blend01 *blend = (prc_surf_blend01 *)params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    curve_func center_eval_func = NULL, origin_eval_func = NULL, tangent_eval_func = NULL;
    void *center_params = NULL, *origin_params = NULL, *tangent_params = NULL;
    double center_min_u, center_max_u, origin_min_u, origin_max_u, tangent_min_u, tangent_max_u;
    prc_vec3 center_pt, origin_pt, r_vec, tangent_vec, cross_vec;
    double tangent_len;
    int code;

    code = prc_get_curve_eval_func(ctx, &blend->center_curve, &center_eval_func,
        &center_params, &center_min_u, &center_max_u);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid center curve type in prc_evaluate_surf_blend01\n");
        return output;
    }
    code = prc_get_curve_eval_func(ctx, &blend->origin_curve, &origin_eval_func,
        &origin_params, &origin_min_u, &origin_max_u);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid origin curve type in prc_evaluate_surf_blend01\n");
        return output;
    }

    center_pt = center_eval_func(ctx, center_params, v);
    origin_pt = origin_eval_func(ctx, origin_params, v);

    r_vec.x = origin_pt.x - center_pt.x;
    r_vec.y = origin_pt.y - center_pt.y;
    r_vec.z = origin_pt.z - center_pt.z;

    if (!blend->tangent_curve.is_referenced && blend->tangent_curve.curve_type != PRC_TYPE_ROOT)
    {
        code = prc_get_curve_eval_func(ctx, &blend->tangent_curve, &tangent_eval_func,
            &tangent_params, &tangent_min_u, &tangent_max_u);
        if (code < 0)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid tangent curve type in prc_evaluate_surf_blend01\n");
            return output;
        }
        tangent_vec = tangent_eval_func(ctx, tangent_params, v);
    }
    else
    {
        /* No tangent curve: fall back to the unitized first derivative of the origin curve at v */
        double h = (origin_max_u - origin_min_u) * 1e-5;
        double v_plus, v_minus;
        prc_vec3 p_plus, p_minus;

        if (h < 1e-9)
            h = 1e-9;
        v_plus = v + h;
        v_minus = v - h;
        if (v_plus > origin_max_u)
            v_plus = origin_max_u;
        if (v_minus < origin_min_u)
            v_minus = origin_min_u;

        p_plus = origin_eval_func(ctx, origin_params, v_plus);
        p_minus = origin_eval_func(ctx, origin_params, v_minus);

        tangent_vec.x = (p_plus.x - p_minus.x) / (v_plus - v_minus);
        tangent_vec.y = (p_plus.y - p_minus.y) / (v_plus - v_minus);
        tangent_vec.z = (p_plus.z - p_minus.z) / (v_plus - v_minus);
    }

    tangent_len = sqrt(tangent_vec.x * tangent_vec.x + tangent_vec.y * tangent_vec.y + tangent_vec.z * tangent_vec.z);
    if (tangent_len > 1e-12)
    {
        tangent_vec.x /= tangent_len;
        tangent_vec.y /= tangent_len;
        tangent_vec.z /= tangent_len;
    }

    prc_vec_cross(tangent_vec, r_vec, &cross_vec);

    output.x = center_pt.x + cos(u) * r_vec.x + sin(u) * cross_vec.x;
    output.y = center_pt.y + cos(u) * r_vec.y + sin(u) * cross_vec.y;
    output.z = center_pt.z + cos(u) * r_vec.z + sin(u) * cross_vec.z;

    if (blend->has_transform && !blend->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &blend->exact_geom_transform, output);
    }

    return output;
}

static prc_vec3
prc_evaluate_surf_blend02(prc_context *ctx, void *params, double u, double v)
{
    prc_surf_blend02 *blend = (prc_surf_blend02 *)params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    curve_func center_eval_func = NULL;
    void *center_params = NULL;
    double center_min_u = 0.0, center_max_u = 0.0;
    double radius, implicit_v, cos_a, angle_a, x_len, y_len, y2_len;
    prc_vec3 center, p1, p2, x_dir, y_dir, cross_xy, y2_dir;
    int code;

    code = prc_get_curve_eval_func(ctx, &blend->center_curve, &center_eval_func,
        &center_params, &center_min_u, &center_max_u);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid center curve type in prc_evaluate_surf_blend02\n");
        return output;
    }
    center = center_eval_func(ctx, center_params, u);

    code = prc_project_onto_blend_bound(ctx, &blend->bound_surface0, &blend->bound_curve0, center, &p1);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid bound geometry 0 in prc_evaluate_surf_blend02\n");
        return output;
    }
    code = prc_project_onto_blend_bound(ctx, &blend->bound_surface1, &blend->bound_curve1, center, &p2);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid bound geometry 1 in prc_evaluate_surf_blend02\n");
        return output;
    }

    x_dir.x = p1.x - center.x; x_dir.y = p1.y - center.y; x_dir.z = p1.z - center.z;
    x_len = sqrt(x_dir.x * x_dir.x + x_dir.y * x_dir.y + x_dir.z * x_dir.z);
    if (x_len > 1e-12)
    {
        x_dir.x /= x_len; x_dir.y /= x_len; x_dir.z /= x_len;
    }

    y_dir.x = p2.x - center.x; y_dir.y = p2.y - center.y; y_dir.z = p2.z - center.z;
    y_len = sqrt(y_dir.x * y_dir.x + y_dir.y * y_dir.y + y_dir.z * y_dir.z);
    if (y_len > 1e-12)
    {
        y_dir.x /= y_len; y_dir.y /= y_len; y_dir.z /= y_len;
    }

    cos_a = x_dir.x * y_dir.x + x_dir.y * y_dir.y + x_dir.z * y_dir.z;
    if (cos_a > 1.0) cos_a = 1.0;
    if (cos_a < -1.0) cos_a = -1.0;
    angle_a = acos(cos_a);

    prc_vec_cross(x_dir, y_dir, &cross_xy);
    prc_vec_cross(cross_xy, x_dir, &y2_dir);
    y2_len = sqrt(y2_dir.x * y2_dir.x + y2_dir.y * y2_dir.y + y2_dir.z * y2_dir.z);
    if (y2_len > 1e-12)
    {
        y2_dir.x /= y2_len; y2_dir.y /= y2_len; y2_dir.z /= y2_len;
    }

    radius = fabs(blend->radius0);
    implicit_v = (blend->parameterization_type == 0) ? angle_a * v : v;

    output.x = center.x + radius * (cos(implicit_v) * x_dir.x + sin(implicit_v) * y2_dir.x);
    output.y = center.y + radius * (cos(implicit_v) * x_dir.y + sin(implicit_v) * y2_dir.y);
    output.z = center.z + radius * (cos(implicit_v) * x_dir.z + sin(implicit_v) * y2_dir.z);

    if (blend->has_transform && !blend->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &blend->exact_geom_transform, output);
    }

    return output;
}

static prc_vec3
prc_evaluate_surf_cylindrical(prc_context *ctx, void *params, double u, double v)
{
    prc_vec3 output, base_point;
    prc_surf_cylindrical *cylindrical = (prc_surf_cylindrical *)params;
    surface_func base_eval_func = NULL;
    void *base_params = NULL;
    prc_surface_params surf_params = { 0 };
    int code;
    prc_ptr_surface base_surf = cylindrical->base_surface;
    double base_start_u;
    double base_end_u;
    double base_start_v;
    double base_end_v;
    prc_surface_sampling_info sampling_info = { 0 };

    if (base_surf.is_referenced)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base surface case in prc_evaluate_surf_cylindrical\n");
        output.x = 0;
        output.y = 0;
        output.z = 0;
        return output;
    }

    code = prc_get_surface_data(ctx, &base_surf.surface, &sampling_info);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Error in prc_get_surface_data\n");
        output.x = 0;
        output.y = 0;
        output.z = 0;
        return output;
    }

    /* Lets get the base evaluation surface function */
    code = prc_get_surface_eval_func(ctx, &cylindrical->base_surface.surface, &base_eval_func, &base_params);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base surface type in prc_evaluate_surf_cylindrical\n");
        output.x = 0;
        output.y = 0;
        output.z = 0;
        return output;
    }

    surf_params.surface_params = base_params;
    base_point = base_eval_func(ctx, &surf_params, u, v);
    output.x = base_point.x * cos(base_point.y);
    output.y = base_point.x * sin(base_point.y);
    output.z = base_point.z;

    if (cylindrical->has_transform && !cylindrical->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &cylindrical->exact_geom_transform, output);
    }

    return output;
}

static double
prc_clamp_surface_param(double value, double min_value, double max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static double
prc_wrap_surface_param(double value, double min_value, double max_value, double period)
{
    double span;

    if (period <= 0.0)
        return prc_clamp_surface_param(value, min_value, max_value);

    span = max_value - min_value;
    if (span == 0.0)
        return min_value;

    while (value < min_value)
        value += period;
    while (value > max_value)
        value -= period;

    return value;
}

static uint8_t
prc_surface_axis_wraps(double start_value, double end_value, uint8_t periodic, double period)
{
    double span;

    if (!periodic || period <= 0.0)
        return 0;

    span = fabs(end_value - start_value);
    return fabs(span - period) <= SURFACE_PRECISION ? 1 : 0;
}

static double
prc_get_surface_param(double start_value, double end_value, uint32_t index,
    uint32_t sample_count, uint8_t wraps)
{
    double denominator;

    if (sample_count <= 1)
        return start_value;

    denominator = wraps ? (double)sample_count : (double)(sample_count - 1);
    return start_value + (end_value - start_value) * ((double)index / denominator);
}

static int
prc_compute_surface_normal(prc_context *ctx, surface_func surface_eval_func,
    void *surface_params, double u, double v, double du, double dv,
    double start_u, double end_u, double start_v, double end_v,
    const prc_surface_sampling_info *sampling_info,
    uint8_t orientation, prc_vec3 *normal)
{
    prc_vec3 pu0, pu1, pv0, pv1;
    prc_vec3 tangent_u, tangent_v;
    double u0;
    double u1;
    double v0;
    double v1;
    int code;

    if (sampling_info->u_periodic)
    {
        u0 = prc_wrap_surface_param(u - du, start_u, end_u, sampling_info->u_period);
        u1 = prc_wrap_surface_param(u + du, start_u, end_u, sampling_info->u_period);
    }
    else
    {
        u0 = prc_clamp_surface_param(u - du, start_u, end_u);
        u1 = prc_clamp_surface_param(u + du, start_u, end_u);
    }

    if (sampling_info->v_periodic)
    {
        v0 = prc_wrap_surface_param(v - dv, start_v, end_v, sampling_info->v_period);
        v1 = prc_wrap_surface_param(v + dv, start_v, end_v, sampling_info->v_period);
    }
    else
    {
        v0 = prc_clamp_surface_param(v - dv, start_v, end_v);
        v1 = prc_clamp_surface_param(v + dv, start_v, end_v);
    }

    pu0 = surface_eval_func(ctx, surface_params, u0, v);
    pu1 = surface_eval_func(ctx, surface_params, u1, v);
    pv0 = surface_eval_func(ctx, surface_params, u, v0);
    pv1 = surface_eval_func(ctx, surface_params, u, v1);

    prc_vec_sub(pu1, pu0, &tangent_u);
    prc_vec_sub(pv1, pv0, &tangent_v);
    prc_vec_cross(tangent_u, tangent_v, normal);
    code = prc_vec_normalize(normal);
    if (code < 0)
    {
        normal->x = 0.0;
        normal->y = 0.0;
        normal->z = 1.0;
    }

    if (orientation == 0)
    {
        prc_vec_negate(normal);
    }

    return 0;
}

static prc_vec3
prc_evaluate_surf_offset(prc_context *ctx, void *params, double u, double v)
{
    prc_surf_offset *offset = (prc_surf_offset *)params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    surface_func base_eval_func = NULL;
    void *base_params = NULL;
    prc_surface_sampling_info sampling_info = { 0 };
    prc_vec3 base_point, base_normal;
    double du, dv;
    int code;

    if (offset->base_surface.is_referenced)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base surface case in prc_evaluate_surf_offset\n");
        return output;
    }

    code = prc_get_surface_eval_func(ctx, &offset->base_surface.surface, &base_eval_func, &base_params);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base surface type in prc_evaluate_surf_offset\n");
        return output;
    }

    code = prc_get_surface_data(ctx, &offset->base_surface.surface, &sampling_info);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Error in prc_get_surface_data in prc_evaluate_surf_offset\n");
        return output;
    }

    du = fabs(sampling_info.end_u - sampling_info.start_u) * 1e-5;
    dv = fabs(sampling_info.end_v - sampling_info.start_v) * 1e-5;
    if (du < 1e-9)
        du = 1e-9;
    if (dv < 1e-9)
        dv = 1e-9;

    base_point = base_eval_func(ctx, base_params, u, v);

    /* Orientation 1 (no negation): the offset direction follows the base surface's natural normal */
    code = prc_compute_surface_normal(ctx, base_eval_func, base_params, u, v, du, dv,
        sampling_info.start_u, sampling_info.end_u, sampling_info.start_v, sampling_info.end_v,
        &sampling_info, 1, &base_normal);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Error computing base normal in prc_evaluate_surf_offset\n");
        return output;
    }

    output.x = base_point.x + offset->offset_distance * base_normal.x;
    output.y = base_point.y + offset->offset_distance * base_normal.y;
    output.z = base_point.z + offset->offset_distance * base_normal.z;

    if (offset->has_transform && !offset->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &offset->exact_geom_transform, output);
    }

    return output;
}

static int
prc_get_surface_data(prc_context *ctx, prc_type_surf *surface,
                     prc_surface_sampling_info *sampling_info)
{
    prc_uv_parameterization params;
    prc_domain domain;
    uint8_t has_transform = 0;
    prc_trans_3d *prc_trans = NULL;
    prc_exact_geom_transform *exact_geom_trans = NULL;
    int code;

    /* Initialize parameters to quiet compiler */
    memset(&params, 0, sizeof(prc_uv_parameterization));

    switch (surface->surface_type)
    {
        case PRC_TYPE_SURF_Cone:
        {
            prc_surf_cone *cone = surface->surf_cone;
            params = cone->parameterization;
            has_transform = cone->has_transform;
            prc_trans = &cone->transform;
            exact_geom_trans = &cone->exact_geom_transform;

            sampling_info->num_samples_u = SURFACE_SAMPLES;
            sampling_info->num_samples_v = 2;

            sampling_info->u_periodic = params.swap_uv ? 0 : 1;
            sampling_info->v_periodic = params.swap_uv ? 1 : 0;
            sampling_info->u_linear = 0;
            sampling_info->v_linear = params.swap_uv ? 0 : 1;
            sampling_info->u_period = params.swap_uv ? 0.0 : 2.0 * PRC_PI;
            sampling_info->v_period = params.swap_uv ? 2.0 * PRC_PI : 0.0;
            sampling_info->precision_u = CONE_SURFACE_PRECISION;
            sampling_info->precision_v = SURFACE_PRECISION;
            sampling_info->max_samples_u = CONE_MAX_SAMPLES;
            sampling_info->max_samples_v = 2;
            if (params.swap_uv)
            {
                sampling_info->u_linear = 1;
                sampling_info->v_linear = 0;
                sampling_info->num_samples_u = 2;
                sampling_info->num_samples_v = SURFACE_SAMPLES;
                sampling_info->precision_u = SURFACE_PRECISION;
                sampling_info->precision_v = CONE_SURFACE_PRECISION;
                sampling_info->max_samples_u = 2;
                sampling_info->max_samples_v = CONE_MAX_SAMPLES;
            }
            break;
        }
        case PRC_TYPE_SURF_Cylinder:
        {
            prc_surf_cylinder *cylinder = surface->surf_cylinder;
            params = cylinder->parameterization;
            has_transform = cylinder->has_transform;
            prc_trans = &cylinder->transform;
            exact_geom_trans = &cylinder->exact_geom_transform;

            sampling_info->num_samples_u = SURFACE_SAMPLES;
            sampling_info->num_samples_v = 2;
            sampling_info->u_periodic = params.swap_uv ? 0 : 1;
            sampling_info->v_periodic = params.swap_uv ? 1 : 0;
            sampling_info->u_linear = 0;
            sampling_info->v_linear = params.swap_uv ? 0 : 1;
            sampling_info->u_period = params.swap_uv ? 0.0 : 2.0 * PRC_PI;
            sampling_info->v_period = params.swap_uv ? 2.0 * PRC_PI : 0.0;
            sampling_info->precision_u = CYLINDER_SURFACE_PRECISION;
            sampling_info->precision_v = SURFACE_PRECISION;
            sampling_info->max_samples_u = CYLINDER_MAX_SAMPLES;
            sampling_info->max_samples_v = 2;
            if (params.swap_uv)
            {
                sampling_info->u_linear = 1;
                sampling_info->v_linear = 0;
                sampling_info->num_samples_u = 2;
                sampling_info->num_samples_v = SURFACE_SAMPLES;
                sampling_info->precision_u = SURFACE_PRECISION;
                sampling_info->precision_v = CYLINDER_SURFACE_PRECISION;
                sampling_info->max_samples_u = 2;
                sampling_info->max_samples_v = CYLINDER_MAX_SAMPLES;
            }
            break;
        }
        case PRC_TYPE_SURF_Blend01:
        {
            prc_surf_blend01 *blend = surface->surf_blend01;
            curve_func center_eval_func = NULL;
            void *center_params = NULL;
            double center_min_u = 0.0, center_max_u = 0.0;

            has_transform = blend->has_transform;
            prc_trans = &blend->transform;
            exact_geom_trans = &blend->exact_geom_transform;

            /* u is the angle around the pipe; v is the center curve parameter */
            code = prc_get_curve_eval_func(ctx, &blend->center_curve, &center_eval_func,
                &center_params, &center_min_u, &center_max_u);
            if (code < 0)
            {
                prc_error(ctx, code, "Invalid center curve type in prc_get_surface_data (Blend01)\n");
                return code;
            }

            domain.min_uv.x = 0.0;
            domain.max_uv.x = 2.0 * PRC_PI;
            domain.min_uv.y = center_min_u;
            domain.max_uv.y = center_max_u;

            sampling_info->num_samples_u = SURFACE_SAMPLES;
            sampling_info->num_samples_v = SURFACE_SAMPLES;
            sampling_info->u_periodic = 1;
            sampling_info->v_periodic = 0;
            sampling_info->u_linear = 0;
            sampling_info->v_linear = 0;
            sampling_info->u_period = 2.0 * PRC_PI;
            sampling_info->v_period = 0;
            sampling_info->precision_u = SURFACE_PRECISION;
            sampling_info->precision_v = SURFACE_PRECISION;
            sampling_info->max_samples_u = SURFACE_MAX_SAMPLES;
            sampling_info->max_samples_v = SURFACE_MAX_SAMPLES;
            break;
        }
        case PRC_TYPE_SURF_Blend02:
        {
            prc_surf_blend02 *blend = surface->surf_blend02;
            curve_func center_eval_func = NULL;
            void *center_params = NULL;
            double center_min_u = 0.0, center_max_u = 0.0;

            has_transform = blend->has_transform;
            prc_trans = &blend->transform;
            exact_geom_trans = &blend->exact_geom_transform;

            /* u is the center curve parameter; v's range depends on parameterization_type */
            code = prc_get_curve_eval_func(ctx, &blend->center_curve, &center_eval_func,
                &center_params, &center_min_u, &center_max_u);
            if (code < 0)
            {
                prc_error(ctx, code, "Invalid center curve type in prc_get_surface_data (Blend02)\n");
                return code;
            }

            domain.min_uv.x = center_min_u;
            domain.max_uv.x = center_max_u;
            domain.min_uv.y = 0.0;
            domain.max_uv.y = (blend->parameterization_type == 0) ? 1.0 : 2.0 * PRC_PI;

            sampling_info->num_samples_u = SURFACE_SAMPLES;
            sampling_info->num_samples_v = SURFACE_SAMPLES;
            sampling_info->u_periodic = 0;
            sampling_info->v_periodic = 0;
            sampling_info->u_linear = 0;
            sampling_info->v_linear = 0;
            sampling_info->u_period = 0;
            sampling_info->v_period = 0;
            sampling_info->precision_u = SURFACE_PRECISION;
            sampling_info->precision_v = SURFACE_PRECISION;
            sampling_info->max_samples_u = SURFACE_MAX_SAMPLES;
            sampling_info->max_samples_v = SURFACE_MAX_SAMPLES;
            break;
        }
        case PRC_TYPE_SURF_Blend03:
        {   
            prc_surf_blend03 *blend = surface->surf_blend03;
            params = blend->parameterization;
            has_transform = blend->has_transform;
            prc_trans = &blend->transform;
            exact_geom_trans = &blend->exact_geom_transform;
            break;
        }
        case PRC_TYPE_SURF_NURBS:
        {
            prc_surf_nurbs *nurbs = surface->surf_nurbs;

            /* Valid parameter range excludes the clamped end knot multiplicities */
            domain.min_uv.x = nurbs->knot_vector_u[nurbs->du];
            domain.max_uv.x = nurbs->knot_vector_u[nurbs->highest_index_of_knots_u - nurbs->du];
            domain.min_uv.y = nurbs->knot_vector_v[nurbs->dv];
            domain.max_uv.y = nurbs->knot_vector_v[nurbs->highest_index_of_knots_v - nurbs->dv];

            sampling_info->num_samples_u = SURFACE_SAMPLES;
            sampling_info->num_samples_v = SURFACE_SAMPLES;
            sampling_info->u_periodic = 0;
            sampling_info->v_periodic = 0;
            sampling_info->u_linear = (nurbs->du <= 1) ? 1 : 0;
            sampling_info->v_linear = (nurbs->dv <= 1) ? 1 : 0;
            sampling_info->u_period = 0;
            sampling_info->v_period = 0;
            sampling_info->precision_u = NURBS_SURFACE_PRECISION;
            sampling_info->precision_v = NURBS_SURFACE_PRECISION;
            sampling_info->max_samples_u = NURBS_MAX_SAMPLES;
            sampling_info->max_samples_v = NURBS_MAX_SAMPLES;
            break;
        }
        case PRC_TYPE_SURF_Cylindrical:
        {
            prc_surf_cylindrical *cylindrical = surface->surf_cylindrical;
            params = cylindrical->parameterization;
            has_transform = cylindrical->has_transform;
            prc_trans = &cylindrical->transform;
            exact_geom_trans = &cylindrical->exact_geom_transform;

            sampling_info->num_samples_u = CYLINDRICAL_MAX_SAMPLES;
            sampling_info->num_samples_v = CYLINDRICAL_MAX_SAMPLES;
            sampling_info->u_periodic = 0;
            sampling_info->v_periodic = 0;
            sampling_info->u_linear = 1;
            sampling_info->v_linear = 1;
            sampling_info->u_period = 0;
            sampling_info->v_period = 0;
            sampling_info->precision_u = CYLINDRICAL_SURFACE_PRECISION;
            sampling_info->precision_v = CYLINDRICAL_SURFACE_PRECISION;
            break;
        }
        case PRC_TYPE_SURF_Offset:
        {
            prc_surf_offset *offset = surface->surf_offset;
            prc_surface_sampling_info base_sampling_info = { 0 };
            params = offset->parameterization;

            has_transform = offset->has_transform;
            prc_trans = &offset->transform;
            exact_geom_trans = &offset->exact_geom_transform;

            /* The implicit parameterization matches the base surface's UV domain */
            if (offset->base_surface.is_referenced)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base surface case in prc_get_surface_data (Offset)\n");
                return PRC_ERROR_INTERNAL;
            }

            code = prc_get_surface_data(ctx, &offset->base_surface.surface, &base_sampling_info);
            if (code < 0)
            {
                prc_error(ctx, code, "Error in prc_get_surface_data for Offset base surface\n");
                return code;
            }

            *sampling_info = base_sampling_info;
            sampling_info->start_u = params.surface_domain.min_uv.x;
            sampling_info->start_v = params.surface_domain.min_uv.y;
            sampling_info->end_u = params.surface_domain.max_uv.x;
            sampling_info->end_v = params.surface_domain.max_uv.y;
            break;
        }
        case PRC_TYPE_SURF_Pipe:
        {
            prc_surf_pipe *pipe = surface->surf_pipe;
            params = pipe->parameterization;
            has_transform = pipe->has_transform;
            prc_trans = &pipe->transform;
            exact_geom_trans = &pipe->exact_geom_transform;
            break;
        }
        case PRC_TYPE_SURF_Plane:
        {
            /* This one does not have the has_transform bit*/
            prc_surf_plane *plane = surface->surf_plane;
            domain = plane->domain;
            has_transform = 1; /* Always has a transform */
            prc_trans = &plane->transform;
            exact_geom_trans = &plane->exact_geom_transform;
            sampling_info->num_samples_u = PLANE_MAX_SAMPLES;
            sampling_info->num_samples_v = PLANE_MAX_SAMPLES;
            sampling_info->u_periodic = 0;
            sampling_info->v_periodic = 0;
            sampling_info->u_linear = 1;
            sampling_info->v_linear = 1;
            sampling_info->u_period = 0;
            sampling_info->v_period = 0;
            sampling_info->precision_u = PLANE_SURFACE_PRECISION;
            sampling_info->precision_v = PLANE_SURFACE_PRECISION;
            break;
        }
        case PRC_TYPE_SURF_Ruled:
        {
            prc_surf_ruled *ruled = surface->surf_ruled;
            params = ruled->parameterization;
            has_transform = ruled->has_transform;
            prc_trans = &ruled->transform;
            exact_geom_trans = &ruled->exact_geom_transform;
            break;
        }
        case PRC_TYPE_SURF_Sphere:
        {
            prc_surf_sphere *sphere = surface->surf_sphere;
            params = sphere->parameterization;
            has_transform = sphere->has_transform;
            prc_trans = &sphere->transform;
            exact_geom_trans = &sphere->exact_geom_transform;
            sampling_info->num_samples_u = SPHERE_MAX_SAMPLES;
            sampling_info->num_samples_v = SPHERE_MAX_SAMPLES;
            sampling_info->u_periodic = 1;
            sampling_info->v_periodic = 1;
            sampling_info->u_linear = 0;
            sampling_info->v_linear = 0;
            sampling_info->u_period = 2.0 * PRC_PI;
            sampling_info->v_period = 2.0 * PRC_PI;
            sampling_info->precision_u = SPHERE_SURFACE_PRECISION;
            sampling_info->precision_v = SPHERE_SURFACE_PRECISION;
            break;
        }
        case PRC_TYPE_SURF_Revolution:
        {
            prc_surf_revolution *revolution = surface->surf_revolution;
            uint8_t curve_periodic = 0;
            double curve_period = 0.0;

            params = revolution->parameterization;
            has_transform = revolution->has_transform;
            prc_trans = &revolution->transform;
            exact_geom_trans = &revolution->exact_geom_transform;

            /* Goes with v unless swapped */
            prc_get_curve_periodicity(ctx, &revolution->base_curve,
                &curve_periodic, &curve_period);

            /* TODO Finish the period here */
            sampling_info->num_samples_u = REVOLUTION_MAX_SAMPLES;
            sampling_info->num_samples_v = REVOLUTION_MAX_SAMPLES;

            /* This is a tricky one in terms of the period */
            sampling_info->u_periodic = params.swap_uv ? curve_periodic : 1;
            sampling_info->v_periodic = params.swap_uv ? 1 : curve_periodic;
            sampling_info->u_linear = params.swap_uv ? 0 : 1;
            sampling_info->v_linear = params.swap_uv ? 1 : 0;
            sampling_info->u_period = params.swap_uv ? 2.0 * PRC_PI : curve_period;
            sampling_info->v_period = params.swap_uv ? curve_period : 2.0 * PRC_PI;
            sampling_info->precision_u = REVOLUTION_SURFACE_PRECISION;
            sampling_info->precision_v = REVOLUTION_SURFACE_PRECISION;
            break;
        }
        case PRC_TYPE_SURF_Extrusion:
        {
            prc_surf_extrusion *extrusion = surface->surf_extrusion;
            params = extrusion->parameterization;
            has_transform = extrusion->has_transform;
            uint8_t curve_periodic = 0;
            double curve_period = 0.0;

            prc_trans = &extrusion->transform;
            exact_geom_trans = &extrusion->exact_geom_transform;

            prc_get_curve_periodicity(ctx, &extrusion->base_curve,
                &curve_periodic, &curve_period);

            sampling_info->num_samples_u = EXTRUSION_MAX_SAMPLES;
            sampling_info->num_samples_v = EXTRUSION_MAX_SAMPLES;

            sampling_info->u_periodic = params.swap_uv ? 0 : curve_periodic;
            sampling_info->v_periodic = params.swap_uv ? curve_periodic : 0;
            sampling_info->u_linear = 0;
            sampling_info->v_linear = params.swap_uv ? 0 : 1;
            sampling_info->u_period = params.swap_uv ? 0.0 : curve_period;
            sampling_info->v_period = params.swap_uv ? curve_period : 0.0;

            sampling_info->precision_u = EXTRUSION_SURFACE_PRECISION;
            sampling_info->precision_v = EXTRUSION_SURFACE_PRECISION;
            break;
        }
        case PRC_TYPE_SURF_FromCurves:
        {
            prc_surf_fromcurves *from_curves = surface->surf_fromcurves;
            uint8_t curve1_periodic, curve2_periodic;
            double curve1_period, curve2_period;

            params = from_curves->parameterization;
            has_transform = from_curves->has_transform;
            prc_trans = &from_curves->transform;
            exact_geom_trans = &from_curves->exact_geom_transform;

            prc_get_curve_periodicity(ctx, &from_curves->first_curve,
                &curve1_periodic, &curve1_period);
            prc_get_curve_periodicity(ctx, &from_curves->second_curve,
                &curve2_periodic, &curve2_period);

            sampling_info->num_samples_u = SURFACE_SAMPLES;
            sampling_info->num_samples_v = SURFACE_SAMPLES;
            sampling_info->u_periodic = params.swap_uv ? curve2_periodic : curve1_periodic;
            sampling_info->v_periodic = params.swap_uv ? curve1_periodic : curve2_periodic;
            sampling_info->u_linear = 0;
            sampling_info->v_linear = 0;
            sampling_info->u_period = params.swap_uv ? curve2_period : curve1_period;
            sampling_info->v_period = params.swap_uv ? curve1_period : curve2_period;
            sampling_info->precision_u = SURFACE_PRECISION;
            sampling_info->precision_v = SURFACE_PRECISION;
            sampling_info->max_samples_u = SURFACE_MAX_SAMPLES;
            sampling_info->max_samples_v = SURFACE_MAX_SAMPLES;
            break;
        }
        case PRC_TYPE_SURF_Torus:
        {
            prc_surf_torus *torus = surface->surf_torus;
            params = torus->parameterization;
            has_transform = torus->has_transform;
            prc_trans = &torus->transform;
            exact_geom_trans = &torus->exact_geom_transform;

            sampling_info->num_samples_u = TORUS_MAX_SAMPLES;
            sampling_info->num_samples_v = TORUS_MAX_SAMPLES;
            sampling_info->u_periodic = 0;
            sampling_info->v_periodic = 0;
            sampling_info->u_linear = 1;
            sampling_info->v_linear = 1;
            sampling_info->u_period = 2.0 * PRC_PI;
            sampling_info->v_period = 2.0 * PRC_PI;
            sampling_info->precision_u = TORUS_SURFACE_PRECISION;
            sampling_info->precision_v = TORUS_SURFACE_PRECISION;
            sampling_info->max_samples_u = TORUS_MAX_SAMPLES;
            sampling_info->max_samples_v = TORUS_MAX_SAMPLES;
            break;
        }
        case PRC_TYPE_SURF_Transform:
        {
            prc_surf_transform *transform = surface->surf_transform;
            params = transform->parameterization;
            has_transform = transform->has_transform;
            prc_trans = &transform->transform;
            exact_geom_trans = &transform->exact_geom_transform;
            break;
        }
        case PRC_TYPE_SURF_Blend04:
        {
            /* TODO */
            break;
        }
        default:
            return PRC_ERROR_INTERNAL;
    }

    if (has_transform && prc_trans != NULL && exact_geom_trans != NULL)
    {
        code = prc_exact_geom_set_transform(ctx, exact_geom_trans, prc_trans);
        if (code < 0)
        {
            return code;
        }
    }
    else if (exact_geom_trans != NULL)
    {
        exact_geom_trans->is_identity = 1;
    }

    if (surface->surface_type == PRC_TYPE_SURF_Plane || surface->surface_type == PRC_TYPE_SURF_NURBS ||
        surface->surface_type == PRC_TYPE_SURF_Blend01 || surface->surface_type == PRC_TYPE_SURF_Blend02)
    {
        sampling_info->start_u = domain.min_uv.x;
        sampling_info->start_v = domain.min_uv.y;
        sampling_info->end_u = domain.max_uv.x;
        sampling_info->end_v = domain.max_uv.y;
    }
    else
    {
        sampling_info->start_u = params.surface_domain.min_uv.x;
        sampling_info->start_v = params.surface_domain.min_uv.y;
        sampling_info->end_u = params.surface_domain.max_uv.x;
        sampling_info-> end_v = params.surface_domain.max_uv.y;

        if (params.swap_uv)
        {
            double temp = sampling_info->start_u;
            sampling_info->start_u = sampling_info->start_v;
            sampling_info->start_v = temp;
            temp = sampling_info->end_u;
            sampling_info->end_u = sampling_info->end_v;
            sampling_info->end_v = temp;
        }
    }
    return 0;
}

static int
prc_get_curve_by_id(prc_context *ctx, prc_nano_brep_compressed_data *compressed_data,
    uint32_t index_compressed_curve, prc_compressed_curve **curve)
{
    if (index_compressed_curve >= compressed_data->current_curve_index)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Invalid compressed curve index: %u\n", index_compressed_curve);
        return PRC_ERROR_PARSE;
    }
    /* Slots are allocated on demand, so an in-range index can still be null if
       the curve it names was never reached -- a truncated or malformed file.
       Returning null here would push the check onto every caller. */
    if (compressed_data->curves[index_compressed_curve] == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Compressed curve %u was never parsed\n",
            index_compressed_curve);
        return PRC_ERROR_PARSE;
    }
    *curve = compressed_data->curves[index_compressed_curve];
    return 0;
}

static int
prc_get_vertex_by_id(prc_context *ctx, prc_nano_brep_compressed_data *compressed_data,
    uint32_t index_compressed_vertex, prc_compressed_vertex **vertex)
{
    if (compressed_data == NULL ||
        index_compressed_vertex >= compressed_data->current_vertex_index)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Vertex ref struct NULL or invalid compressed vertex index: %u\n", index_compressed_vertex);
        return PRC_ERROR_PARSE;
    }
    *vertex = &compressed_data->vertices[index_compressed_vertex];
    return 0;
}

static int
prc_get_compressed_curve(prc_context *ctx, prc_ref_or_compressed_curve *ref_or_comp_curve,
    prc_compressed_curve **comp_curve, prc_nano_brep_compressed_data *compressed_data)
{
    int code = 0;

    if (ref_or_comp_curve->curve_is_not_already_stored ||
        ref_or_comp_curve->is_deduced_curve)
    {
        *comp_curve = ref_or_comp_curve->compressed_curve;
    }
    else
    {
        code = prc_get_curve_by_id(ctx, compressed_data,
            ref_or_comp_curve->index_compressed_curve, comp_curve);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed to get curve by id in prc_get_compressed_curve\n");
            return code;
        }
    }
    return 0;
}

static int
prc_get_compressed_vertex(prc_context *ctx, prc_compressed_vertex *comp_vertex,
                         prc_vec3 *vertex, prc_nano_brep_compressed_data *compressed_data)
{
    int code = 0;

    if (comp_vertex->not_already_stored)
    {
        *vertex = comp_vertex->point_data.point;
        return 0;
    }
    else
    {
        prc_compressed_vertex *vertex_store = NULL;
        code = prc_get_vertex_by_id(ctx, compressed_data, comp_vertex->point_index,
                                    &vertex_store);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed to get common vertex by id in prc_get_compressed_point\n");
            return code;
        }
        *vertex = vertex_store->point_data.point;
    }
    return 0;
}

static void
prc_get_compressed_point(prc_context *ctx, prc_compressed_point *comp_point,
    prc_vec3 *vertex)
{
    *vertex = comp_point->point;
}

static int
prc_get_start_end_data(prc_context *ctx, prc_start_end_data *data,
    prc_vec3 *start, prc_vec3 *end, prc_nano_brep_compressed_data *compressed_data)
{
    int code = 0;

    if (data->is_vertex)
    {
        code = prc_get_compressed_vertex(ctx, &data->start_vertex, start, compressed_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed to get start vertex by id in prc_get_start_end_data\n");
            return code;
        }
        code = prc_get_compressed_vertex(ctx, &data->end_vertex, end, compressed_data);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed to get end vertex by id in prc_get_start_end_data\n");
            return code;
        }
    }
    else
    {
        prc_get_compressed_point(ctx, &data->start_point, start);
        prc_get_compressed_point(ctx, &data->end_point, end);
    }
    return 0;
}

static int
prc_get_hcg_line_data(prc_context *ctx, prc_hcg_line *data,
    prc_vec3 *start, prc_vec3 *end, prc_nano_brep_compressed_data *compressed_data)
{
    int code;

    code = prc_get_start_end_data(ctx, &data->start_end_data, start, end, compressed_data);

    return code;
}

static void
prc_compute_hcg_circle_theta(prc_hcg_circle_information *info)
{
    prc_vec3 start_rel;
    prc_vec3 end_rel;
    prc_vec3 normal;
    prc_vec3 cross_vec;
    double dot_se;
    double signed_angle;

    info->theta = 0.0;

    if (!info->has_center || !info->has_normal || !info->has_start_end_points)
    {
        return;
    }

    if (info->is_full_circle)
    {
        info->theta = 2.0 * PRC_PI;
        return;
    }

    if (info->is_arc_of_zero_pi_or_twopi)
    {
        return;
    }

    prc_vec_sub(info->start_point, info->center, &start_rel);
    prc_vec_sub(info->end_point, info->center, &end_rel);

    if (prc_vec_length(start_rel) <= CURVE_PRECISION ||
        prc_vec_length(end_rel) <= CURVE_PRECISION)
    {
        return;
    }

    normal = info->normal;
    prc_vec_normalize(&normal);
    prc_vec_normalize(&start_rel);
    prc_vec_normalize(&end_rel);

    dot_se = prc_vec_dot_product(start_rel, end_rel);
    prc_vec_cross(start_rel, end_rel, &cross_vec);
    signed_angle = atan2(prc_vec_dot_product(normal, cross_vec), dot_se);
    if (signed_angle < 0.0)
    {
        signed_angle += 2.0 * PRC_PI;
    }

    if (info->has_middle_of_arc_point)
    {
        prc_vec3 mid_rel;
        prc_vec3 mid_cross;
        double mid_dot;

        prc_vec_sub(info->middle_of_arc_point, info->center, &mid_rel);
        if (prc_vec_length(mid_rel) > CURVE_PRECISION)
        {
            prc_vec_normalize(&mid_rel);
            prc_vec_cross(start_rel, mid_rel, &mid_cross);
            mid_dot = prc_vec_dot_product(normal, mid_cross);
            if (mid_dot < 0.0)
            {
                signed_angle = 2.0 * PRC_PI - signed_angle;
            }
        }
    }

    info->theta = signed_angle;
}

static int
prc_get_hcg_circle_data(prc_context *ctx, prc_hcg_circle *hcg_circle,
    prc_hcg_circle_information *info, prc_nano_brep_compressed_data *compressed_data)
{
    int code;

    memset(info, 0, sizeof(*info));

    if (hcg_circle->is_particular_circle)
    {
        info->is_arc_of_zero_pi_or_twopi = 1;
        info->is_full_circle = hcg_circle->particular_circle.full_circle;
        if (!hcg_circle->particular_circle.compressed_iso_spline)
        {
            info->has_start_end_points = 1;
            code = prc_get_start_end_data(ctx, &hcg_circle->particular_circle.start_end_data,
                &info->start_point, &info->end_point, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get start end in prc_get_hcg_circle_data\n");
                return code;
            }
        }
        if (info->is_full_circle)
        {
            info->has_center = 1;
            info->center = hcg_circle->particular_circle.center.point;
            info->has_normal = 1;
            info->normal = hcg_circle->particular_circle.normal_plane.point;
        }
        else
        {
            info->has_middle_of_arc_point = 1;
            info->middle_of_arc_point = hcg_circle->particular_circle.middle_of_arc.point;
        }
    }
    else
    {
        if (!hcg_circle->general_circle.compressed_iso_spline)
        {
            info->has_start_end_points = 1;
            info->start_point = hcg_circle->general_circle.start_end_data.start_point.point;
            info->end_point = hcg_circle->general_circle.start_end_data.end_point.point;
        }
        info->has_center = 1;
        info->center = hcg_circle->general_circle.center.point;
        info->has_circle_angle_bit = 1;
        info->circle_angle_bit = hcg_circle->general_circle.circle_angle;
    }

    if (!info->has_center && !info->is_full_circle && info->has_start_end_points)
    {
        prc_vec3 midpoint;

        if (prc_vec_dist_between_two_points(info->start_point, info->end_point) <= CURVE_PRECISION)
        {
            if (info->has_middle_of_arc_point &&
                prc_vec_dist_between_two_points(info->start_point, info->middle_of_arc_point) > CURVE_PRECISION)
            {
                /* Full circle: the opposite point is the arc midpoint and the center is
                   at the midpoint between the repeated start/end and that opposite point. */
                prc_vec_avg(info->start_point, info->middle_of_arc_point, &midpoint);
                info->center = midpoint;
                info->has_center = 1;
                info->is_arc_of_zero_pi_or_twopi = 1;
            }
            else
            {
                /* Zero-angle arc: nothing to infer; treat as degenerate and reject. */
                info->is_arc_of_zero_pi_or_twopi = 1;
            }
        }
        else if (info->has_middle_of_arc_point)
        {
            /* Half-circle: the circle center lies at the midpoint of the diameter endpoints. */
            prc_vec_avg(info->start_point, info->end_point, &midpoint);
            info->center = midpoint;
            info->has_center = 1;
            info->is_arc_of_zero_pi_or_twopi = 1;
        }
    }

    /* If we have a valid, non-degenerate circle but no explicit endpoints, use the
       arc midpoint and center to synthesize a diameter pair. This keeps the caller
       supplied with start/end geometry for drawing and trimming while ignoring the
       truly degenerate zero-angle case. */
    if (!info->has_start_end_points && info->has_center && !info->is_full_circle &&
        !info->is_arc_of_zero_pi_or_twopi && info->has_middle_of_arc_point)
    {
        prc_vec3 radius_vec;

        prc_vec_sub(info->middle_of_arc_point, info->center, &radius_vec);
        if (prc_vec_length(radius_vec) > CURVE_PRECISION)
        {
            prc_vec3 start = info->center;
            prc_vec3 end = info->center;

            prc_vec_sub(info->center, radius_vec, &start);
            prc_vec_add(info->center, radius_vec, &end);
            info->start_point = start;
            info->end_point = end;
            info->has_start_end_points = 1;
        }
    }

    if (info->has_center && info->has_start_end_points)
    {
        prc_compute_hcg_circle_theta(info);
    }

    return 0;
}

static double
prc_get_hcg_circle_radius(const prc_hcg_circle_information *info)
{
    prc_vec3 center_to_point;
    double radius = 0.0;

    if (!info->has_center)
    {
        return 0.0;
    }

    if (info->has_start_end_points)
    {
        prc_vec_sub(info->start_point, info->center, &center_to_point);
        radius = prc_vec_length(center_to_point);
        if (radius > 0.0)
        {
            return radius;
        }
        prc_vec_sub(info->end_point, info->center, &center_to_point);
        radius = prc_vec_length(center_to_point);
        if (radius > 0.0)
        {
            return radius;
        }
    }

    if (info->has_middle_of_arc_point)
    {
        prc_vec_sub(info->middle_of_arc_point, info->center, &center_to_point);
        radius = prc_vec_length(center_to_point);
        if (radius > 0.0)
        {
            return radius;
        }
    }

    return 0.0;
}

static int
prc_build_iso_cylinder_from_circle_data(prc_context *ctx,
    const prc_hcg_circle_information *circle_info,
    const prc_vec3 *line_start,
    const prc_vec3 *line_end,
    const prc_vec3 *common_vertex,
    prc_surf_cylinder *cylinder)
{
    prc_vec3 axis;
    prc_vec3 center;
    prc_vec3 radial_vec;
    prc_vec3 x_axis;
    prc_vec3 y_axis;
    prc_vec3 ref_axis;
    prc_vec3 line_dir;
    double radius = 0.0;
    int code;

    if (circle_info == NULL || cylinder == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Invalid cylinder circle data in prc_build_iso_cylinder_from_circle_data\n");
        return PRC_ERROR_PARSE;
    }
    if (!circle_info->has_center)
    {
        prc_error(ctx, PRC_ERROR_PARSE,
            "Missing cylinder circle center in prc_build_iso_cylinder_from_circle_data\n");
        return PRC_ERROR_PARSE;
    }

    memset(cylinder, 0, sizeof(*cylinder));
    center = circle_info->center;

    if (circle_info->has_normal)
    {
        axis = circle_info->normal;
    }
    else if (line_start != NULL && line_end != NULL)
    {
        prc_vec_sub(*line_end, *line_start, &line_dir);
        axis = line_dir;
    }
    else
    {
        axis.x = 0.0;
        axis.y = 0.0;
        axis.z = 1.0;
    }
    code = prc_vec_normalize(&axis);
    if (code < 0)
    {
        prc_error(ctx, code, "Degenerate cylinder axis in prc_build_iso_cylinder_from_circle_data\n");
        return code;
    }

    if (common_vertex != NULL)
    {
        prc_vec_sub(*common_vertex, center, &radial_vec);
    }
    else if (line_start != NULL && line_end != NULL)
    {
        prc_vec3 line_mid;
        prc_vec_avg(*line_start, *line_end, &line_mid);
        prc_vec_sub(line_mid, center, &radial_vec);
    }
    else if (circle_info->has_start_end_points)
    {
        prc_vec_sub(circle_info->start_point, center, &radial_vec);
    }
    else
    {
        radial_vec.x = 1.0;
        radial_vec.y = 0.0;
        radial_vec.z = 0.0;
    }

    if (prc_vec_length(radial_vec) > 0.0)
    {
        double axial_component = prc_vec_dot_product(radial_vec, axis);
        prc_vec3 axial_vec = axis;
        prc_vec_scale(axial_component, &axial_vec);
        prc_vec_sub(radial_vec, axial_vec, &radial_vec);
        radius = prc_vec_length(radial_vec);
    }
    if (radius <= 0.0 && circle_info->has_start_end_points)
    {
        prc_vec_sub(circle_info->start_point, center, &radial_vec);
        if (prc_vec_length(radial_vec) > 0.0)
        {
            double axial_component = prc_vec_dot_product(radial_vec, axis);
            prc_vec3 axial_vec = axis;
            prc_vec_scale(axial_component, &axial_vec);
            prc_vec_sub(radial_vec, axial_vec, &radial_vec);
            radius = prc_vec_length(radial_vec);
        }
    }
    if (radius <= 0.0 && circle_info->has_middle_of_arc_point)
    {
        prc_vec_sub(circle_info->middle_of_arc_point, center, &radial_vec);
        if (prc_vec_length(radial_vec) > 0.0)
        {
            double axial_component = prc_vec_dot_product(radial_vec, axis);
            prc_vec3 axial_vec = axis;
            prc_vec_scale(axial_component, &axial_vec);
            prc_vec_sub(radial_vec, axial_vec, &radial_vec);
            radius = prc_vec_length(radial_vec);
        }
    }
    if (radius <= 0.0)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Invalid cylinder radius in prc_build_iso_cylinder_from_circle_data\n");
        return PRC_ERROR_PARSE;
    }

    if (prc_vec_length(radial_vec) > 0.0)
    {
        prc_vec_scale(1.0 / prc_vec_length(radial_vec), &radial_vec);
        x_axis = radial_vec;
    }
    else
    {
        if (fabs(axis.z) > 0.9)
        {
            ref_axis.x = 1.0;
            ref_axis.y = 0.0;
            ref_axis.z = 0.0;
        }
        else
        {
            ref_axis.x = 0.0;
            ref_axis.y = 0.0;
            ref_axis.z = 1.0;
        }
        prc_vec_cross(ref_axis, axis, &x_axis);
        code = prc_vec_normalize(&x_axis);
        if (code < 0)
        {
            x_axis.x = 1.0;
            x_axis.y = 0.0;
            x_axis.z = 0.0;
        }
    }

    prc_vec_cross(axis, x_axis, &y_axis);
    code = prc_vec_normalize(&y_axis);
    if (code < 0)
    {
        y_axis.x = 0.0;
        y_axis.y = 1.0;
        y_axis.z = 0.0;
    }

    cylinder->tag = PRC_TYPE_SURF_Cylinder;
    cylinder->has_transform = 1;
    cylinder->transform.behavior = PRC_TRANSFORMATION_Translate | PRC_TRANSFORMATION_Rotate;
    cylinder->transform.translation = center;
    cylinder->transform.rotation[0] = x_axis;
    cylinder->transform.rotation[1] = y_axis;
    cylinder->transform.scale = 1.0;
    cylinder->radius = radius;
    cylinder->parameterization.swap_uv = 0;
    cylinder->parameterization.surface_domain.min_uv.x = 0.0;
    cylinder->parameterization.surface_domain.min_uv.y = 0.0;
    cylinder->parameterization.surface_domain.max_uv.x = 2.0 * PRC_PI;
    cylinder->parameterization.surface_domain.max_uv.y = 1.0;

    if (prc_exact_geom_set_transform(ctx, &cylinder->exact_geom_transform, &cylinder->transform) < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Failed to initialize cylinder transform in prc_build_iso_cylinder_from_circle_data\n");
        return PRC_ERROR_INTERNAL;
    }

    return 0;
}

static void
prc_compute_circle_angle_basis(const prc_vec3 *plane_normal, prc_vec3 *u_axis, prc_vec3 *v_axis)
{
    prc_vec3 ref_axis;
    prc_vec3 normal = *plane_normal;

    if (fabs(normal.z) > 0.9)
    {
        ref_axis.x = 1.0;
        ref_axis.y = 0.0;
        ref_axis.z = 0.0;
    }
    else
    {
        ref_axis.x = 0.0;
        ref_axis.y = 0.0;
        ref_axis.z = 1.0;
    }

    prc_vec_cross(ref_axis, normal, u_axis);
    prc_vec_normalize(u_axis);
    prc_vec_cross(normal, *u_axis, v_axis);
    prc_vec_normalize(v_axis);
}

static double
prc_compute_circle_angle(const prc_vec3 *center, const prc_vec3 *plane_normal,
    const prc_vec3 *point)
{
    prc_vec3 u_axis;
    prc_vec3 v_axis;
    prc_vec3 rel;
    double x;
    double y;

    prc_compute_circle_angle_basis(plane_normal, &u_axis, &v_axis);
    prc_vec_sub(*point, *center, &rel);
    x = prc_vec_dot_product(rel, u_axis);
    y = prc_vec_dot_product(rel, v_axis);
    return atan2(y, x);
}

static void
prc_normalize_angle_range(double *min_angle, double *max_angle)
{
    double span = *max_angle - *min_angle;

    if (span < 0.0)
    {
        span += 2.0 * PRC_PI;
        *max_angle = *min_angle + span;
    }
    if (span <= 0.0)
    {
        *max_angle = *min_angle + 2.0 * PRC_PI;
    }
}

static int
prc_build_iso_torus_from_circle_data(prc_context *ctx,
    const prc_hcg_circle_information *major_radius_circle_info,
    const prc_hcg_circle_information *minor_radius_circle_info,
    const prc_vec3 *common_vertex, prc_surf_torus *torus)
{
    prc_vec3 torus_center;
    prc_vec3 torus_axis;
    prc_vec3 minor_center_to_major_center;
    prc_vec3 major_center_to_common;
    prc_vec3 torus_reference;
    prc_vec3 x_axis;
    prc_vec3 y_axis;
    double major_radius;
    double minor_radius;
    double u_min = 0.0;
    double u_max = 2.0 * PRC_PI;
    double v_min = 0.0;
    double v_max = 2.0 * PRC_PI;
    int code;

    if (major_radius_circle_info == NULL || minor_radius_circle_info == NULL || torus == NULL)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "Invalid torus circle data in prc_build_iso_torus_from_circle_data\n");
        return PRC_ERROR_PARSE;
    }
    if (!major_radius_circle_info->has_center || !minor_radius_circle_info->has_center)
    {
        prc_error(ctx, PRC_ERROR_PARSE,
            "Missing torus circle center: a zero/π/2π compressed circle cannot define a torus without a resolvable center\n");
        return PRC_ERROR_PARSE;
    }

    memset(torus, 0, sizeof(*torus));
    torus_center = major_radius_circle_info->center;
    torus->tag = PRC_TYPE_SURF_Torus;
    torus->has_transform = 1;
    torus->transform.behavior = PRC_TRANSFORMATION_Translate | PRC_TRANSFORMATION_Rotate;
    torus->transform.translation = torus_center;
    torus->transform.scale = 1.0;

    if (major_radius_circle_info->has_normal)
    {
        torus_axis = major_radius_circle_info->normal;
    }
    else if (common_vertex != NULL)
    {
        prc_vec_sub(minor_radius_circle_info->center, major_radius_circle_info->center, &minor_center_to_major_center);
        prc_vec_sub(*common_vertex, major_radius_circle_info->center, &major_center_to_common);
        prc_vec_cross(minor_center_to_major_center, major_center_to_common, &torus_axis);
    }
    else
    {
        torus_axis.x = 0.0;
        torus_axis.y = 0.0;
        torus_axis.z = 1.0;
    }
    code = prc_vec_normalize(&torus_axis);
    if (code < 0)
    {
        prc_error(ctx, code, "Degenerate torus axis in prc_build_iso_torus_from_circle_data\n");
        return code;
    }

    major_radius = prc_get_hcg_circle_radius(major_radius_circle_info);
    minor_radius = prc_get_hcg_circle_radius(minor_radius_circle_info);
    if (major_radius <= 0.0 || minor_radius <= 0.0)
    {
        if (common_vertex != NULL)
        {
            prc_vec_sub(*common_vertex, torus_center, &major_center_to_common);
            if (prc_vec_length(major_center_to_common) > 0.0)
            {
                major_radius = prc_vec_length(major_center_to_common);
            }
        }
        if (major_radius <= 0.0 || minor_radius <= 0.0)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "Invalid torus radii in prc_build_iso_torus_from_circle_data\n");
            return PRC_ERROR_PARSE;
        }
    }
    torus->major_radius = major_radius;
    torus->minor_radius = minor_radius;

    if (common_vertex != NULL)
    {
        prc_vec_sub(minor_radius_circle_info->center, torus_center, &minor_center_to_major_center);
        if (prc_vec_length(minor_center_to_major_center) > 0.0)
        {
            prc_vec_scale(1.0 / prc_vec_length(minor_center_to_major_center), &minor_center_to_major_center);
            x_axis = minor_center_to_major_center;
        }
        else
        {
            prc_vec_sub(*common_vertex, torus_center, &major_center_to_common);
            if (prc_vec_length(major_center_to_common) > 0.0)
            {
                prc_vec_scale(1.0 / prc_vec_length(major_center_to_common), &major_center_to_common);
                x_axis = major_center_to_common;
            }
            else
            {
                x_axis.x = 1.0;
                x_axis.y = 0.0;
                x_axis.z = 0.0;
            }
        }
    }
    else
    {
        x_axis.x = 1.0;
        x_axis.y = 0.0;
        x_axis.z = 0.0;
    }

    if (fabs(torus_axis.z) > 0.9)
    {
        torus_reference.x = 1.0;
        torus_reference.y = 0.0;
        torus_reference.z = 0.0;
    }
    else
    {
        torus_reference.x = 0.0;
        torus_reference.y = 0.0;
        torus_reference.z = 1.0;
    }

    prc_vec_cross(torus_reference, torus_axis, &x_axis);
    code = prc_vec_normalize(&x_axis);
    if (code < 0)
    {
        x_axis.x = 1.0;
        x_axis.y = 0.0;
        x_axis.z = 0.0;
        code = prc_vec_normalize(&x_axis);
        if (code < 0)
        {
            return code;
        }
    }
    prc_vec_cross(torus_axis, x_axis, &y_axis);
    code = prc_vec_normalize(&y_axis);
    if (code < 0)
    {
        y_axis.x = 0.0;
        y_axis.y = 1.0;
        y_axis.z = 0.0;
    }

    torus->transform.rotation[0] = x_axis;
    torus->transform.rotation[1] = y_axis;
    torus->parameterization.swap_uv = 0;

    if (major_radius_circle_info->has_start_end_points && major_radius_circle_info->has_center)
    {
        double theta_start = prc_compute_circle_angle(&major_radius_circle_info->center, &torus_axis,
            &major_radius_circle_info->start_point);
        double theta_end = prc_compute_circle_angle(&major_radius_circle_info->center, &torus_axis,
            &major_radius_circle_info->end_point);
        u_min = theta_start;
        u_max = theta_end;
        prc_normalize_angle_range(&u_min, &u_max);
    }
    if (minor_radius_circle_info->has_start_end_points && minor_radius_circle_info->has_center)
    {
        double theta_start = prc_compute_circle_angle(&minor_radius_circle_info->center, &torus_axis,
            &minor_radius_circle_info->start_point);
        double theta_end = prc_compute_circle_angle(&minor_radius_circle_info->center, &torus_axis,
            &minor_radius_circle_info->end_point);
        v_min = theta_start;
        v_max = theta_end;
        prc_normalize_angle_range(&v_min, &v_max);
    }
    if (major_radius_circle_info->is_full_circle)
    {
        u_min = 0.0;
        u_max = 2.0 * PRC_PI;
    }
    if (minor_radius_circle_info->is_full_circle)
    {
        v_min = 0.0;
        v_max = 2.0 * PRC_PI;
    }

    torus->parameterization.surface_domain.min_uv.x = u_min;
    torus->parameterization.surface_domain.min_uv.y = v_min;
    torus->parameterization.surface_domain.max_uv.x = u_max;
    torus->parameterization.surface_domain.max_uv.y = v_max;

    if (prc_exact_geom_set_transform(ctx, &torus->exact_geom_transform, &torus->transform) < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Failed to initialize torus transform in prc_build_iso_torus_from_circle_data\n");
        return PRC_ERROR_INTERNAL;
    }

    return 0;
}

static int
prc_tessellate_compressed_face(prc_context *ctx, prc_data *data, uint32_t shell_index, uint32_t face_index,
    prc_compressed_face *topo_face, prc_nano_brep_compressed_data *compressed_data)
{
    int code;

    switch (topo_face->tag)
    {
        case PRC_HCG_IsoPlane:
        {
            prc_hcg_iso_plane hcg_iso_plane = topo_face->hcg_iso_plane;
            break;
        }  

        case PRC_HCG_IsoCylinder:
        {
            /* For this we have one circle and one line plus a vertex, where
             * the circle defines the radius and axis and the line supplies an
             * axis direction; we then synthesize a normal prc_surf_cylinder and
             * tessellate it through the standard surface path. */
            prc_hcg_iso_cylinder *hcg_iso_cylinder = &topo_face->hcg_iso_cylinder;
            prc_content_compressed_face *face = &hcg_iso_cylinder->face;
            prc_compressed_curve *first_trim_curve = NULL;
            prc_compressed_curve *second_trim_curve = NULL;
            prc_compressed_curve *third_trim_curve = NULL;
            prc_compressed_curve *fourth_trim_curve = NULL;
            prc_compressed_curve *line_curve = first_trim_curve;
            prc_compressed_curve *circle_curve = second_trim_curve;
            prc_vec3 line_start, line_end;
            prc_hcg_circle_information circle_info;
            prc_vec3 common_vertex;
            prc_surf_cylinder cylinder = { 0 };
            prc_topo_face synthetic_face = { 0 };
            prc_type_surf synthetic_surface = { 0 };

            code = prc_get_compressed_curve(ctx, &face->iso_face.first_trim_curve,
                &first_trim_curve, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get first trim curve by id in prc_tessellate_compressed_face\n");
                return code;
            }

            code = prc_get_compressed_curve(ctx, &face->iso_face.second_trim_curve,
                &second_trim_curve, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get second trim curve by id in prc_tessellate_compressed_face\n");
                return code;
            }

            if (first_trim_curve->curve_type == PRC_HCG_Circle && second_trim_curve->curve_type == PRC_HCG_Line)
            {
                circle_curve = first_trim_curve;
                line_curve = second_trim_curve;
            }
            else if (first_trim_curve->curve_type == PRC_HCG_Line && second_trim_curve->curve_type == PRC_HCG_Circle)
            {
                line_curve = first_trim_curve;
                circle_curve = second_trim_curve;
            }
            else
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Invalid curve types for cylinder in prc_tessellate_compressed_face\n");
                return PRC_ERROR_PARSE;
            }

            code = prc_get_hcg_circle_data(ctx, &circle_curve->hcg_circle, &circle_info, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get circle data in prc_tessellate_compressed_face\n");
                return code;
            }
            code = prc_get_hcg_line_data(ctx, &line_curve->hcg_line, &line_start, &line_end, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get line data in prc_tessellate_compressed_face\n");
                return code;
            }

            code = prc_get_compressed_vertex(ctx, &face->iso_face.common_third_fourth_vertex,
                &common_vertex, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get common vertex in prc_tessellate_compressed_face\n");
                return code;
            }

            /* The missing bounds close the ISO cylinder parameter rectangle.
               Curve 3 is the line parallel to the supplied axis line, while
               curve 4 is the same circle translated by the line length along the axis. */
            {
                prc_vec3 axis_dir;
                prc_vec3 axis_offset;
                prc_vec3 right_boundary_start;
                double height;

                prc_vec_sub(line_end, line_start, &axis_dir);
                height = prc_vec_length(axis_dir);
                if (height <= CURVE_PRECISION)
                {
                    axis_dir.x = 0.0;
                    axis_dir.y = 0.0;
                    axis_dir.z = 1.0;
                    height = 1.0;
                }
                else
                {
                    prc_vec_scale(1.0 / height, &axis_dir);
                }
                axis_offset = axis_dir;
                prc_vec_scale(height, &axis_offset);

                if (circle_info.has_start_end_points)
                {
                    double start_dist = prc_vec_dist_between_two_points(circle_info.start_point, common_vertex);
                    double end_dist = prc_vec_dist_between_two_points(circle_info.end_point, common_vertex);
                    right_boundary_start = (start_dist <= end_dist) ? circle_info.end_point : circle_info.start_point;
                }
                else
                {
                    right_boundary_start = common_vertex;
                }

                if (face->iso_face.third_trim_curve_is_not_yet_saved)
                {
                    code = prc_get_compressed_curve(ctx, &face->iso_face.third_trim_curve,
                        &third_trim_curve, compressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed to get third trim curve by id in prc_tessellate_compressed_face\n");
                        return code;
                    }

                    memset(&third_trim_curve->hcg_line, 0, sizeof(third_trim_curve->hcg_line));
                    third_trim_curve->curve_type = PRC_HCG_Line;
                    third_trim_curve->hcg_line.type = PRC_HCG_Line;
                    third_trim_curve->hcg_line.start_end_data.is_vertex = 0;
                    third_trim_curve->hcg_line.start_end_data.start_point.point = right_boundary_start;
                    prc_vec_add(right_boundary_start, axis_offset,
                        &third_trim_curve->hcg_line.start_end_data.end_point.point);
                }

                if (face->iso_face.fourth_trim_curve_is_not_yet_saved)
                {
                    code = prc_get_compressed_curve(ctx, &face->iso_face.fourth_trim_curve,
                        &fourth_trim_curve, compressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed to get fourth trim curve by id in prc_tessellate_compressed_face\n");
                        return code;
                    }

                    memset(&fourth_trim_curve->hcg_circle, 0, sizeof(fourth_trim_curve->hcg_circle));
                    fourth_trim_curve->curve_type = PRC_HCG_Circle;
                    fourth_trim_curve->hcg_circle.type = PRC_HCG_Circle;
                    fourth_trim_curve->hcg_circle.is_particular_circle = 1;
                    fourth_trim_curve->hcg_circle.particular_circle.full_circle = circle_info.is_full_circle;
                    fourth_trim_curve->hcg_circle.particular_circle.compressed_iso_spline = 0;
                    fourth_trim_curve->hcg_circle.particular_circle.start_end_data.is_vertex = 0;
                    fourth_trim_curve->hcg_circle.particular_circle.center.point = circle_info.center;
                    prc_vec_add(fourth_trim_curve->hcg_circle.particular_circle.center.point, axis_offset,
                        &fourth_trim_curve->hcg_circle.particular_circle.center.point);
                    if (circle_info.has_normal)
                    {
                        fourth_trim_curve->hcg_circle.particular_circle.normal_plane.point = circle_info.normal;
                    }
                    else
                    {
                        fourth_trim_curve->hcg_circle.particular_circle.normal_plane.point = axis_dir;
                    }
                    if (circle_info.has_start_end_points)
                    {
                        prc_vec3 translated_start = circle_info.start_point;
                        prc_vec3 translated_end = circle_info.end_point;
                        prc_vec_add(translated_start, axis_offset, &translated_start);
                        prc_vec_add(translated_end, axis_offset, &translated_end);
                        fourth_trim_curve->hcg_circle.particular_circle.start_end_data.start_point.point = translated_start;
                        fourth_trim_curve->hcg_circle.particular_circle.start_end_data.end_point.point = translated_end;
                    }
                    if (circle_info.has_middle_of_arc_point)
                    {
                        prc_vec3 translated_mid = circle_info.middle_of_arc_point;
                        prc_vec_add(translated_mid, axis_offset, &translated_mid);
                        fourth_trim_curve->hcg_circle.particular_circle.middle_of_arc.point = translated_mid;
                    }
                }
            }

            code = prc_build_iso_cylinder_from_circle_data(ctx, &circle_info, &line_start, &line_end,
                &common_vertex, &cylinder);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to reconstruct cylinder from compressed circle data\n");
                return code;
            }

            synthetic_surface.surface_type = PRC_TYPE_SURF_Cylinder;
            synthetic_surface.surf_cylinder = &cylinder;
            synthetic_face.surface_geometry.is_referenced = 0;
            synthetic_face.surface_geometry.surface = synthetic_surface;

            code = prc_tessellate_surface(ctx, data, shell_index, face_index, &synthetic_face,
                face->orientation_surface_with_shell, NULL, NULL);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_tessellate_surface for compressed ISO cylinder\n");
                return code;
            }

            break;
        }

        case PRC_HCG_IsoTorus:
        {
            /* is_major_radius TRUE indicates if the first serialized circle defines
               the major radius. */
            prc_hcg_iso_torus *hcg_iso_torus = &topo_face->hcg_iso_torus;
            uint8_t is_major_radius = hcg_iso_torus->is_major_radius;
            prc_content_compressed_face *face = &hcg_iso_torus->face;
            prc_compressed_curve *first_trim_curve = NULL;
            prc_compressed_curve *second_trim_curve = NULL;
            prc_compressed_curve *third_trim_curve = NULL;
            prc_compressed_curve *fourth_trim_curve = NULL;
            prc_compressed_curve *major_radius_curve = first_trim_curve;
            prc_compressed_curve *minor_radius_curve = second_trim_curve;
            prc_vec3 common_vertex;
            prc_hcg_circle_information minor_radius_circle_info;
            prc_hcg_circle_information major_radius_circle_info;
            prc_surf_torus torus = {0};
            prc_topo_face synthetic_face = {0};
            prc_type_surf synthetic_surface = {0};

            code = prc_get_compressed_curve(ctx, &face->iso_face.first_trim_curve,
                                            &first_trim_curve, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get second trim curve by id in prc_tessellate_compressed_face\n");
                return code;
            }

            code = prc_get_compressed_curve(ctx, &face->iso_face.second_trim_curve,
                                            &second_trim_curve, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to get second trim curve by id in prc_tessellate_compressed_face\n");
                return code;
            }

            if (is_major_radius)
            {
                major_radius_curve = first_trim_curve;
                minor_radius_curve = second_trim_curve;
            }
            else
            {
                major_radius_curve = second_trim_curve;
                minor_radius_curve = first_trim_curve;
            }

            code = prc_get_compressed_vertex(ctx, &face->iso_face.common_third_fourth_vertex,
                &common_vertex, compressed_data);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to reconstruct torus from compressed circle data\n");
                return code;
            }

            if (minor_radius_curve->curve_type != PRC_HCG_Circle || major_radius_curve->curve_type != PRC_HCG_Circle)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "Invalid curve type for torus in prc_tessellate_compressed_face\n");
                return PRC_ERROR_PARSE;
            }

            prc_get_hcg_circle_data(ctx, &minor_radius_curve->hcg_circle, &minor_radius_circle_info, compressed_data);
            prc_get_hcg_circle_data(ctx, &major_radius_curve->hcg_circle, &major_radius_circle_info, compressed_data);

            code = prc_build_iso_torus_from_circle_data(ctx,
                &major_radius_circle_info, &minor_radius_circle_info,
                &common_vertex, &torus);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed to reconstruct torus from compressed circle data\n");
                return code;
            }

            /* We have to construct the deduced curves in case they are later
               referenced. These are the implicit right/top boundaries of the
               torus patch and must remain in the same torus parameter space as the
               supplied trim curves. */
            {
                prc_vec3 torus_axis;
                prc_vec3 major_dir = torus.transform.rotation[0];
                prc_vec3 minor_dir = torus.transform.rotation[1];
                const double u_min = torus.parameterization.surface_domain.min_uv.x;
                const double u_max = torus.parameterization.surface_domain.max_uv.x;
                const double v_min = torus.parameterization.surface_domain.min_uv.y;
                const double v_max = torus.parameterization.surface_domain.max_uv.y;

                prc_vec_cross(major_dir, minor_dir, &torus_axis);
                code = prc_vec_normalize(&torus_axis);
                if (code < 0)
                {
                    prc_error(ctx, code, "Degenerate torus axis while constructing implied trim curves\n");
                    return code;
                }

                /* This curve *could* be referencing another curve, in which case,
                   we don't do the implied creation */
                if (face->iso_face.third_trim_curve_is_not_yet_saved)
                {
                    /* Curve 3: fixed u = u_max, sweep v = v_min..v_max. */
                    code = prc_get_compressed_curve(ctx, &face->iso_face.third_trim_curve,
                        &third_trim_curve, compressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed to get third trim curve by id in prc_tessellate_compressed_face\n");
                        return code;
                    }

                    memset(&third_trim_curve->hcg_circle, 0, sizeof(third_trim_curve->hcg_circle));
                    third_trim_curve->curve_type = PRC_HCG_Circle;
                    third_trim_curve->hcg_circle.type = PRC_HCG_Circle;
                    third_trim_curve->hcg_circle.is_particular_circle = 1;
                    third_trim_curve->hcg_circle.particular_circle.full_circle = 0;
                    third_trim_curve->hcg_circle.particular_circle.compressed_iso_spline = 0;
                    third_trim_curve->hcg_circle.particular_circle.start_end_data.is_vertex = 0;
                    third_trim_curve->hcg_circle.particular_circle.start_end_data.start_point.point =
                        prc_evaluate_surf_torus(ctx, &torus, u_max, v_min);
                    third_trim_curve->hcg_circle.particular_circle.start_end_data.end_point.point =
                        prc_evaluate_surf_torus(ctx, &torus, u_max, v_max);
                    third_trim_curve->hcg_circle.particular_circle.center.point =
                        torus.transform.translation;
                    {
                        prc_vec3 offset = major_dir;
                        prc_vec_scale(torus.major_radius * cos(u_max), &offset);
                        {
                            prc_vec3 offset2 = minor_dir;
                            prc_vec_scale(torus.major_radius * sin(u_max), &offset2);
                            prc_vec_add(offset, offset2, &offset);
                        }
                        prc_vec_add(third_trim_curve->hcg_circle.particular_circle.center.point, offset,
                            &third_trim_curve->hcg_circle.particular_circle.center.point);
                    }
                    third_trim_curve->hcg_circle.particular_circle.normal_plane.point = torus_axis;
                    third_trim_curve->hcg_circle.particular_circle.middle_of_arc.point =
                        prc_evaluate_surf_torus(ctx, &torus, u_max, 0.5 * (v_min + v_max));
                }

                /* Only do this creation if we have not referenced it */
                if (face->iso_face.fourth_trim_curve_is_not_yet_saved)
                {
                    /* Curve 4: fixed v = v_max, sweep u = u_min..u_max. */
                    code = prc_get_compressed_curve(ctx, &face->iso_face.fourth_trim_curve,
                        &fourth_trim_curve, compressed_data);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed to get fourth trim curve by id in prc_tessellate_compressed_face\n");
                        return code;
                    }

                    memset(&fourth_trim_curve->hcg_circle, 0, sizeof(fourth_trim_curve->hcg_circle));
                    fourth_trim_curve->curve_type = PRC_HCG_Circle;
                    fourth_trim_curve->hcg_circle.type = PRC_HCG_Circle;
                    fourth_trim_curve->hcg_circle.is_particular_circle = 1;
                    fourth_trim_curve->hcg_circle.particular_circle.full_circle = 0;
                    fourth_trim_curve->hcg_circle.particular_circle.compressed_iso_spline = 0;
                    fourth_trim_curve->hcg_circle.particular_circle.start_end_data.is_vertex = 0;
                    fourth_trim_curve->hcg_circle.particular_circle.start_end_data.start_point.point =
                        prc_evaluate_surf_torus(ctx, &torus, u_min, v_max);
                    fourth_trim_curve->hcg_circle.particular_circle.start_end_data.end_point.point =
                        prc_evaluate_surf_torus(ctx, &torus, u_max, v_max);
                    fourth_trim_curve->hcg_circle.particular_circle.center.point = torus.transform.translation;
                    {
                        prc_vec3 offset = torus_axis;
                        prc_vec_scale(torus.minor_radius * sin(v_max), &offset);
                        prc_vec_add(fourth_trim_curve->hcg_circle.particular_circle.center.point, offset,
                            &fourth_trim_curve->hcg_circle.particular_circle.center.point);
                    }
                    fourth_trim_curve->hcg_circle.particular_circle.normal_plane.point = torus_axis;
                    fourth_trim_curve->hcg_circle.particular_circle.middle_of_arc.point =
                        prc_evaluate_surf_torus(ctx, &torus, 0.5 * (u_min + u_max), v_max);
                }
            }

            synthetic_surface.surface_type = PRC_TYPE_SURF_Torus;
            synthetic_surface.surf_torus = &torus;
            synthetic_face.surface_geometry.is_referenced = 0;
            synthetic_face.surface_geometry.surface = synthetic_surface;

            code = prc_tessellate_surface(ctx, data, shell_index, face_index, &synthetic_face,
                face->orientation_surface_with_shell, NULL, NULL);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_tessellate_surface for compressed ISO torus\n");
                return code;
            }

            break;
        }

        case PRC_HCG_IsoSphere:
        {
            prc_hcg_iso_sphere hcg_iso_sphere = topo_face->hcg_iso_sphere;
            break;
        }

        case PRC_HCG_IsoCone:
        {
            prc_hcg_iso_cone hcg_iso_cone = topo_face->hcg_iso_cone;
            break;
        }

        case PRC_HCG_IsoNURBS:
        {
            prc_hcg_iso_nurbs hcg_iso_nurbs = topo_face->hcg_iso_nurbs;
            break;
        }

        case PRC_HCG_AnaPlane:
        {
            prc_hcg_ana_plane hcg_ana_plane = topo_face->hcg_ana_plane;
            break;
        }

        case PRC_HCG_AnaCylinder:
        {
            prc_hcg_ana_cylinder hcg_ana_cylinder = topo_face->hcg_ana_cylinder;
            break;
        }

        case PRC_HCG_AnaTorus:
        {
            prc_hcg_ana_torus hcg_ana_torus = topo_face->hcg_ana_torus;
            break;
        }

        case PRC_HCG_AnaSphere:
        {
            prc_hcg_ana_sphere hcg_ana_sphere = topo_face->hcg_ana_sphere;
            break;
        }

        case PRC_HCG_AnaCone:
        {
            prc_hcg_ana_cone hcg_ana_cone = topo_face->hcg_ana_cone;
            break;
        }

        case PRC_HCG_AnaNURBS:
        {
            prc_hcg_ana_nurbs hcg_ana_nurbs = topo_face->hcg_ana_nurbs;
            break;
        }

        case PRC_HCG_AnaGenericFace:
        {
            prc_hcg_ana_generic_face hcg_ana_generic_face = topo_face->hcg_ana_generic_face;
            break;
        }
        default:
            /* topo_face->tag is what this switch dispatches on; `entity_type`
               named nothing in this scope and only compiled because the old
               prc_error macro discarded its arguments without evaluating them. */
            prc_error(ctx, PRC_ERROR_PARSE,
                "Unknown entity type in prc_tessellate_compressed_face: %u\n",
                (unsigned)topo_face->tag);
            return PRC_ERROR_PARSE;
    }
    return 0;
}

static int
prc_get_ptr_vertex(prc_context *ctx, prc_nano_brep_ref_data *brep_ref_data,
                   prc_ptr_topology *ptr_topo_in, prc_vec3 *vertex)
{
    prc_topo *topo;

    if (!ptr_topo_in->is_stored)
    {
        topo = ptr_topo_in->topo;
    }
    else
    {
        if (ptr_topo_in->topo_identifier >= brep_ref_data->number_of_topo_refs)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid topo identifier in prc_get_ptr_vertex\n");
            return PRC_ERROR_INTERNAL;
        }
        /* These are biased I believe */
        topo = brep_ref_data->topo_refs[ptr_topo_in->topo_identifier - 1];
    }

    /* I think this could be PRC_TYPE_TOPO_MultipleVertex also */
    if (topo->tag != PRC_TYPE_TOPO_UniqueVertex)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid topo identifier in prc_get_ptr_vertex\n");
        return PRC_ERROR_INTERNAL;
    }

    *vertex = topo->topo_unique_vertex->vertex;
    return 0;
}

static int
prc_sample_coedge(prc_context *ctx, prc_nano_brep_ref_data *brep_ref_data,
    prc_coedge_in_loop *topo_coedge, prc_coedge_samples *coedge_samples)
{
    int code;

    /* This function samples a coedge and stores the results in coedge_samples */
    if (topo_coedge->next_coedge.topo->tag != PRC_TYPE_TOPO_CoEdge)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Error in prc_sample_coedge\n");
        return PRC_ERROR_INTERNAL;
    }
    else
    {
        prc_topo *topo;

        /* First get the loop that we need */
        if (!topo_coedge->next_coedge.topo->topo_coedge->ptr_topology.is_stored)
        {
            topo = topo_coedge->next_coedge.topo->topo_coedge->ptr_topology.topo;
        }
        else
        {
            if (topo_coedge->next_coedge.topo->topo_coedge->ptr_topology.topo_identifier >= brep_ref_data->number_of_topo_refs)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid topo identifier in prc_sample_coedge\n");
                return PRC_ERROR_INTERNAL;
            }
            /* These are biased I believe */
            topo = brep_ref_data->topo_refs[topo_coedge->next_coedge.topo->topo_coedge->ptr_topology.topo_identifier - 1];
        }

        if (topo->tag != PRC_TYPE_TOPO_Edge)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Error in prc_sample_coedge\n");
            return PRC_ERROR_INTERNAL;
        }
        else
        {
            /* Now we finally get to the curve. However, the curve may be
               defined by two vertices */
            prc_topo_coedge *actual_coedge = topo_coedge->next_coedge.topo->topo_coedge;
            prc_exact_geom_wire_data wire_samples = { 0 };

            /* We could have a PRC_TYPE_TOPO_WireEdge or a PRC_TYPE_TOPO_Edge */
            if (topo->tag == PRC_TYPE_TOPO_WireEdge)
            {
                prc_topo_wire_edge *wire_edge = topo->topo_wire_edge;

                code = prc_sample_curve(ctx, &wire_edge->curve, &wire_samples);
                if (code < 0)
                {
                    if (wire_samples.points != NULL)
                    {
                        prc_free(ctx, wire_samples.points);
                    }
                    prc_error(ctx, code, "Error in prc_sample_curve\n");
                    return code;
                }
            }
            else if (topo->tag == PRC_TYPE_TOPO_Edge)
            {
                /* This is a confusing object. Not sure how the start
                   and end vertex are used and there is a curve. For now
                   check the curve. If it is no data use the vertices */
                prc_topo_edge *topo_edge = topo->topo_edge;

                if (topo_edge->wire_edge.ptr_curve.curve_type != PRC_TYPE_CRV_NONE)
                {
                    code = prc_sample_curve(ctx, &topo_edge->wire_edge, &wire_samples);
                    if (code < 0)
                    {
                        if (wire_samples.points != NULL)
                        {
                            prc_free(ctx, wire_samples.points);
                        }
                        prc_error(ctx, code, "Error in prc_sample_curve\n");
                        return code;
                    }
                }
                else
                {
                    prc_vec3 vertex1, vertex2;
                    code = prc_get_ptr_vertex(ctx, brep_ref_data,
                                              &topo_edge->start_vertex, &vertex1);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Error in prc_get_ptr_vertex\n");
                        return code;
                    }

                    code = prc_get_ptr_vertex(ctx, brep_ref_data,
                                              &topo_edge->end_vertex, &vertex2);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Error in prc_get_ptr_vertex\n");
                        return code;
                    }

                    /* If vertex1 == vertex2, we only need one point.  This
                       happens if we are clipping at a cone apex for example */
                    if (vertex1.x == vertex2.x && vertex1.y == vertex2.y && 
                        vertex1.z == vertex2.z)
                    {
                        wire_samples.number_of_points = 1;
                    }
                    else
                    {
                        wire_samples.number_of_points = 2;
                    }

                    wire_samples.points = (prc_vec3 *)prc_calloc(ctx,
                                wire_samples.number_of_points, sizeof(prc_vec3));
                    if (wire_samples.points == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Error in prc_sample_coedge\n");
                        return PRC_ERROR_MEMORY;
                    }
                    wire_samples.points[0] = vertex1;
                    if (wire_samples.number_of_points > 1)
                    {
                        wire_samples.points[1] = vertex2;
                    }
                }
            }
            else
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "Error in prc_sample_coedge\n");
                return PRC_ERROR_INTERNAL;
            }

            /* coedge_orientation == 0 means this coedge is traversed opposite
               to its underlying edge curve's own direction; prc_sample_curve
               always samples in the curve's native direction, so reverse here
               to match the loop's actual boundary traversal (confirmed by a
               real file: without this, consecutive coedges' samples don't
               connect end-to-start, producing bogus discontinuities) */
            if (!actual_coedge->coedge_orientation && wire_samples.number_of_points > 1)
            {
                uint32_t lo = 0, hi = wire_samples.number_of_points - 1;

                while (lo < hi)
                {
                    prc_vec3 tmp = wire_samples.points[lo];

                    wire_samples.points[lo] = wire_samples.points[hi];
                    wire_samples.points[hi] = tmp;
                    lo++;
                    hi--;
                }
            }

            /* Transfer the samples to the coedge structure */
            coedge_samples->num_samples = wire_samples.number_of_points;
            coedge_samples->samples = wire_samples.points;
        }
    }
    return 0;
}

static int
prc_sample_loop(prc_context *ctx, prc_nano_brep_ref_data *brep_ref_data,
    prc_topo_face *topo_face, prc_ptr_topology *loop, prc_loop_samples *loop_samples)
{
    /* Loops are made up of several co-edges which should be curves.
       Here we make our way through a sampling approximation of the loop */
    prc_topo_loop *topo_loop;
    uint32_t num_coedges;
    uint32_t k, j;
    int code;
    prc_coedge_samples *coedge_samples;
    uint32_t total_samples = 0;

    /* First get the loop that we need */
    if (!loop->is_stored)
    {
        topo_loop = loop->topo->topo_loop;
    }
    else
    {
        if (loop->topo_identifier >= brep_ref_data->number_of_topo_refs)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid loop identifier in prc_sample_loop\n");
            return PRC_ERROR_INTERNAL;
        }
        /* These are biased I believe */
        topo_loop = brep_ref_data->topo_refs[loop->topo_identifier - 1]->topo_loop;
    }

    num_coedges = topo_loop->number_of_coedges;
    coedge_samples = (prc_coedge_samples *)prc_calloc(ctx, num_coedges, sizeof(prc_coedge_samples));
    if (coedge_samples == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed in allocation prc_sample_loop\n");
        return PRC_ERROR_MEMORY;
    }

    /* We have to sample each of the coedges and concatenate them together. */
    for (k = 0; k < num_coedges; k++)
    {
        code = prc_sample_coedge(ctx, brep_ref_data, &topo_loop->coedge[k], &coedge_samples[k]);
        if (code < 0)
        {
            /* Clean up any coedges that did work */
            for (j = 0; j <= k; j++)
            {
                if (coedge_samples[j].samples != NULL)
                {
                    prc_free(ctx, coedge_samples[j].samples);
                }
            }
            prc_free(ctx, coedge_samples);
            prc_error(ctx, code, "Failed in prc_sample_coedge\n");
            return code;
        }
        total_samples += coedge_samples[k].num_samples;
    }

#ifdef PRC_DEBUG_EARCLIP
    fprintf(stderr, "prc_sample_loop: num_coedges=%u\n", num_coedges);
    for (k = 0; k < num_coedges; k++)
    {
        prc_vec3 first = coedge_samples[k].samples[0];
        prc_vec3 last = coedge_samples[k].samples[coedge_samples[k].num_samples - 1];

        fprintf(stderr, "  coedge[%u]: num_samples=%u first=(%.9f,%.9f,%.9f) last=(%.9f,%.9f,%.9f)\n",
            k, coedge_samples[k].num_samples,
            first.x, first.y, first.z, last.x, last.y, last.z);
    }
#endif

    /* Now we have to combine the coedges into one loop */
    /* Each coedge begins with the end point of the previous coedge so
       we want to drop any of those points as we make our way around the loop.
       This includes the last point of the last coedge being the first point
       of the first coedge. We will maintain that point. */
    /* Each coedge has one redundant point. */
    loop_samples->num_samples = total_samples - num_coedges + 1;
    loop_samples->samples = (prc_vec3 *)prc_calloc(ctx, loop_samples->num_samples, sizeof(prc_vec3));
    if (loop_samples->samples != NULL)
    {
        uint32_t pos = 0;
        for (j = 0; j < num_coedges; j++)
        {
            /* Parens matter: this drops exactly the coedge's last (redundant)
               point, not one byte of it */
            memcpy(&loop_samples->samples[pos], coedge_samples[j].samples, sizeof(prc_vec3) * (coedge_samples[j].num_samples - 1));
            pos += (coedge_samples[j].num_samples - 1);
        }
        /* Duplicate the first point to complete the loop */
        loop_samples->samples[pos] = coedge_samples[0].samples[0];

        /* A loop trimmed down to a single point (e.g. a cone's apex) samples
           as every point coinciding with the first -- flag it so later
           stages can skip the angular uv mapping and hole/ear-clip bridging
           that only make sense for a loop enclosing real area */
        {
            uint32_t s;
            prc_vec3 first = loop_samples->samples[0];
            uint8_t all_coincide = 1;

            for (s = 1; s < loop_samples->num_samples; s++)
            {
                if (prc_vec_dist_between_two_points(first, loop_samples->samples[s]) > CURVE_PRECISION)
                {
                    all_coincide = 0;
                    break;
                }
            }
            loop_samples->is_point_loop = all_coincide;
        }
    }

    /* Free up the coedge samples */
    for (j = 0; j < num_coedges; j++)
    {
        if (coedge_samples[j].samples != NULL)
        {
            prc_free(ctx, coedge_samples[j].samples);
        }
    }
    prc_free(ctx, coedge_samples);

    return 0;
}

static int
prc_get_surface_transform_inverse(prc_context *ctx, prc_type_surf *surface,
    prc_exact_geom_transform *inverse_transform)
{
    prc_exact_geom_transform *exact_transform;
    int code;

    inverse_transform->is_identity = 1;

    switch (surface->surface_type)
    {
        case PRC_TYPE_SURF_FromCurves:
        {
            exact_transform = &surface->surf_fromcurves->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Cone:
        {
            exact_transform = &surface->surf_cone->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Cylinder:
        {
            exact_transform = &surface->surf_cylinder->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Sphere:
        {
            exact_transform = &surface->surf_sphere->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Torus:
        {
            exact_transform = &surface->surf_torus->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Cylindrical:
        {
            exact_transform = &surface->surf_cylindrical->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Extrusion:
        {
            exact_transform = &surface->surf_extrusion->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Revolution:
        {
            exact_transform = &surface->surf_revolution->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Plane:
        {
            exact_transform = &surface->surf_plane->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Offset:
        {
            exact_transform = &surface->surf_offset->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_NURBS:
        {
            exact_transform = NULL;
            break;
        }

        case PRC_TYPE_SURF_Blend02:
        {
            exact_transform = &surface->surf_blend02->exact_geom_transform;
            break;
        }

        case PRC_TYPE_SURF_Blend01:
        {
            exact_transform = &surface->surf_blend01->exact_geom_transform;
            break;
        }

        default:
            return 0;
    }

    if (exact_transform != NULL && !exact_transform->is_identity)
    {
        /* First compute the inverse matrix */
        code = prc_invert_exact_transform(ctx, exact_transform, inverse_transform);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_invert_exact_transform\n");
            return code;
        }
    }
    return 0;
}

/* Point-in-polygon test (even-odd / ray casting rule), used to determine
   which loop of a planar face encloses the others in uv space */
static int
prc_uv_point_in_polygon(prc_vec2 point, uint32_t num_verts, const prc_vec2 *verts)
{
    uint32_t i, j;
    int inside = 0;

    for (i = 0, j = num_verts - 1; i < num_verts; j = i++)
    {
        double xi = verts[i].x, yi = verts[i].y;
        double xj = verts[j].x, yj = verts[j].y;

        if (((yi > point.y) != (yj > point.y)) &&
            (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi))
        {
            inside = !inside;
        }
    }

    return inside;
}

/* Finds the loop whose uv polygon contains a sample point from every other
   loop and marks it as the outer one (the rest are holes). Only meaningful
   when every loop is a genuine simple closed polygon in uv space (i.e. none
   of them wind around a periodic axis -- see wind_u/wind_v) */
static int
prc_assign_outer_loop_by_nesting(prc_context *ctx, uint32_t num_loops, prc_loop_samples *loop_samples)
{
    uint32_t candidate, other;
    int outer_found = 0;

    for (candidate = 0; candidate < num_loops && !outer_found; candidate++)
    {
        int contains_all = 1;

        for (other = 0; other < num_loops; other++)
        {
            if (other == candidate)
                continue;

            if (loop_samples[other].num_samples == 0 ||
                !prc_uv_point_in_polygon(loop_samples[other].uv_samples[0],
                    loop_samples[candidate].num_samples, loop_samples[candidate].uv_samples))
            {
                contains_all = 0;
                break;
            }
        }

        if (contains_all)
        {
            loop_samples[candidate].is_outer_loop = 1;
            outer_found = 1;
        }
    }

    if (!outer_found)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL,
            "Could not determine outer loop in prc_assign_outer_loop_by_nesting\n");
        return PRC_ERROR_INTERNAL;
    }

    return 0;
}

/* Here if we have multiple loops we may need to assign them as inner or outer
   loops. This seems very messy in the specification so this may need some work
   here */
static int
prc_assign_loop_inner_outer(prc_context *ctx, prc_topo_face *topo_face, uint8_t orientation,
    uint8_t topo_context_behavior, uint32_t num_loops, prc_loop_samples *loop_samples,
    uint32_t surface_type)
{
    uint8_t first_loop_is_outer = topo_context_behavior & PRC_CONTEXT_OuterLoopsFirst;

    if (num_loops == 0)
        return 0;

    /* If number of loops is one then that is the outer loop and there
       are no inner loops or if first_loop_is_outer is true */
    if (num_loops == 1 || first_loop_is_outer == 1)
    {
        loop_samples->is_outer_loop = 1;
        return 0;
    }

    /* The spec is a little funny with respect to index_of_output_loop description
       for the face. In one spot it says . If PRC_CONTEXT_OuterLoopsFirst
       is set to TRUE in the topological context the face is contained in,
       the index of the outer loop shall be defined. But if PRC_CONTEXT_OuterLoopsFirst
       then I would think 0 would be the index as that one is first.... */
    if (topo_face->index_of_outer_loop != -1 && topo_face->index_of_outer_loop < num_loops)
    {
        loop_samples[topo_face->index_of_outer_loop].is_outer_loop = 1;
    }
    else
    {
        uint32_t k;
        uint8_t any_wraps = 0;

        /* Now we have to figure out the outer loop by brute force. This is 
           going to depend upon the surface type.  For some surfaces, the loop
           may actually be a straight line on the surface. Think of a loop going
           around the circumference of a cylinder. In this case, this loop is
           really just a boundary edge. If you have two of these you have two edges */
        for (k = 0; k < num_loops; k++)
        {
            if (loop_samples[k].wind_u != 0 || loop_samples[k].wind_v != 0)
            {
                any_wraps = 1;
                break;
            }
        }

        switch (surface_type)
        {
            case PRC_TYPE_SURF_FromCurves:
            {
                break;
            }

            /* A wrapping loop divides the surface along a periodic axis
               rather than nesting inside another loop's polygon, so the
               nesting test below does not apply to it -- that case is
               instead handled directly by prc_tessellate_surface via each
               loop's wind_u/wind_v (see prc_tessellate_periodic_face_from_loops) */
            case PRC_TYPE_SURF_Cone:
            case PRC_TYPE_SURF_Cylinder:
            case PRC_TYPE_SURF_Sphere:
            case PRC_TYPE_SURF_Torus:
            {
                if (any_wraps)
                    break;

                return prc_assign_outer_loop_by_nesting(ctx, num_loops, loop_samples);
            }

            case PRC_TYPE_SURF_Cylindrical:
            {
                break;
            }

            case PRC_TYPE_SURF_Extrusion:
            {
                break;
            }

            case PRC_TYPE_SURF_Revolution:
            {
                break;
            }

            case PRC_TYPE_SURF_Plane:
            {
                /* In the case of a plane, there is definitely one loop that
                   is the outer boundary and all the others are holes. */
                return prc_assign_outer_loop_by_nesting(ctx, num_loops, loop_samples);
            }

            case PRC_TYPE_SURF_Offset:
            {
                break;
            }

            case PRC_TYPE_SURF_NURBS:
            {
                /* NURBS loops are never periodic/wrapping (see prc_get_
                   surface_data), so like Plane this is always plain
                   point-in-polygon nesting. */
                return prc_assign_outer_loop_by_nesting(ctx, num_loops, loop_samples);
            }

            case PRC_TYPE_SURF_Blend02:
            {
                break;
            }

            case PRC_TYPE_SURF_Blend01:
            {
                break;
            }

            default:
                return 0;
        }
    }

    return 0;
}

static double
prc_map_to_two_pi(double angle)
{
    if (angle < 0.0)
    {
        angle += 2.0 * PRC_PI;
    }
    return angle;
}

/* Rewrites one coordinate (x if use_y_component is 0, else y) of a closed
   loop's uv samples into a continuous unwrapped polyline, undoing the 2*pi
   branch cuts atan2 introduces between consecutive samples, and reports how
   many full turns the loop winds around that axis before closing back up (0
   if it does not wrap, i.e. it is a genuine simple polygon on this axis) */
static int32_t
prc_unwrap_angular_component(prc_vec2 *pts, uint32_t count, uint8_t use_y_component)
{
    uint32_t j;
    double total;

    if (count < 2)
        return 0;

    for (j = 1; j < count; j++)
    {
        double prev = use_y_component ? pts[j - 1].y : pts[j - 1].x;
        double curr = use_y_component ? pts[j].y : pts[j].x;
        double delta = curr - prev;

        delta -= (2.0 * PRC_PI) * floor((delta + PRC_PI) / (2.0 * PRC_PI));
        if (use_y_component)
            pts[j].y = prev + delta;
        else
            pts[j].x = prev + delta;
    }

    total = (use_y_component ? pts[count - 1].y : pts[count - 1].x) -
            (use_y_component ? pts[0].y : pts[0].x);

    return (int32_t)lround(total / (2.0 * PRC_PI));
}

/* Based upon surface type, map the 3D points to the UV surface storing
   the values in loop_samples structure. At this point, the loops are samples of
   the path in 3D space on the surface. We will move this to the uv parametric
   space. The loops at this point have the same start and end point and are
   sampled at a sufficient approximation with all the coedge samples placed in
   a single loop */
static int
prc_map_loops_to_surface(prc_context *ctx, prc_topo_face *topo_face, uint8_t orientation,
    prc_type_surf *surface, uint32_t num_loops, prc_loop_samples *loop_samples)
{
    int code;
    uint32_t k, j;
    prc_exact_geom_transform *exact_transform;
    prc_exact_geom_transform inverse_transform;
    prc_loop_samples *curr_loop;
    uint32_t num_samples;

    code = prc_get_surface_transform_inverse(ctx, surface, &inverse_transform);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_get_surface_transform_inverse\n");
        return code;
    }

    /* Lets map the loop values using the inverse transform if needed */
    if (!inverse_transform.is_identity)
    {
        for (k = 0; k < num_loops; k++)
        {
            curr_loop = &loop_samples[k];
            num_samples = curr_loop->num_samples;
            for (j = 0; j < num_samples; j++)
            {
                curr_loop->samples[j] = prc_exact_geom_apply_transform(ctx,
                    &inverse_transform, curr_loop->samples[j]);
            }
        }
    }

    /* At this point all the loops should be in the uv base space for the surface */

    /* Allocate space for the uv values of the loops */
    for (k = 0; k < num_loops; k++)
    {
        loop_samples[k].uv_samples = (prc_vec2*) prc_calloc(ctx, loop_samples[k].num_samples, sizeof(prc_vec2));
        if (loop_samples[k].uv_samples == NULL)
        {
            for (j = 0; j < (k - 1); j++)
            {
                prc_free(ctx, loop_samples[j].uv_samples);
                loop_samples[j].uv_samples = NULL;
            }
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed in prc_map_loops_to_surface\n");
            return PRC_ERROR_MEMORY;
        }
    }

    /* Now actually map from the 3D base space to the uv surface space */
    switch (surface->surface_type)
    {
        case PRC_TYPE_SURF_FromCurves:
        {
            break;
        }

        case PRC_TYPE_SURF_Cone:
        {
            prc_surf_cone *cone = surface->surf_cone;
            double u_coeff_a = (cone->parameterization.u_param_coeff_a != 0.0) ? cone->parameterization.u_param_coeff_a : 1.0;
            double v_coeff_a = (cone->parameterization.v_param_coeff_a != 0.0) ? cone->parameterization.v_param_coeff_a : 1.0;
            double u, v;

            for (k = 0; k < num_loops; k++)
            {
                curr_loop = &loop_samples[k];
                num_samples = curr_loop->num_samples;

                /* A loop collapsed to the cone's apex has x=y=0 (within
                   tolerance) at every sample, so atan2(y, x) there is just
                   amplifying floating-point noise into an arbitrary angle --
                   skip it and park every sample at a single, stable u since
                   the loop bounds no area and no actual u value matters */
                if (curr_loop->is_point_loop)
                {
                    v = curr_loop->samples[0].z;
                    for (j = 0; j < num_samples; j++)
                    {
                        curr_loop->uv_samples[j].x = (0.0 - cone->parameterization.u_param_coeff_b) / u_coeff_a;
                        curr_loop->uv_samples[j].y = (v - cone->parameterization.v_param_coeff_b) / v_coeff_a;
                    }
                    curr_loop->wind_u = 0;
                    curr_loop->wind_v = 0;
                    continue;
                }

                for (j = 0; j < num_samples; j++)
                {
                    double x = curr_loop->samples[j].x;
                    double y = curr_loop->samples[j].y;

                    v = curr_loop->samples[j].z;

                    /* x = radius(v)*cos(u), y = radius(v)*sin(u): wherever
                       this cone's linear radius(v) = radius + v*tan(semi_angle)
                       is negative (common away from the apex -- a cone's
                       bottom_radius/semi_angle pair can put the whole usable
                       v range on the negative side), atan2(y, x) recovers
                       u + pi instead of u, since negating radius is the same
                       as rotating u by pi. Negate x,y first to undo that so
                       the recovered u matches the surface's own u = atan2(y,x)
                       convention used by prc_evaluate_surf_cone. */
                    if (cone->radius + v * tan(cone->semi_angle) < 0.0)
                    {
                        x = -x;
                        y = -y;
                    }
                    u = atan2(y, x);
                    u = prc_map_to_two_pi(u);

                    /* Store the raw angle/height for now; unwrap before the
                       affine parameterization map so the 2*pi period used to
                       detect wraparound isn't distorted by u_coeff_a */
                    curr_loop->uv_samples[j].x = u;
                    curr_loop->uv_samples[j].y = v;
                }

                curr_loop->wind_u = prc_unwrap_angular_component(curr_loop->uv_samples, num_samples, 0);
                curr_loop->wind_v = 0;

                for (j = 0; j < num_samples; j++)
                {
                    curr_loop->uv_samples[j].x = (curr_loop->uv_samples[j].x - cone->parameterization.u_param_coeff_b) / u_coeff_a;
                    curr_loop->uv_samples[j].y = (curr_loop->uv_samples[j].y - cone->parameterization.v_param_coeff_b) / v_coeff_a;
                }
            }
            break;
        }

        case PRC_TYPE_SURF_Cylinder:
        {
            prc_surf_cylinder *cylinder = surface->surf_cylinder;
            double u_coeff_a = (cylinder->parameterization.u_param_coeff_a != 0.0) ? cylinder->parameterization.u_param_coeff_a : 1.0;
            double v_coeff_a = (cylinder->parameterization.v_param_coeff_a != 0.0) ? cylinder->parameterization.v_param_coeff_a : 1.0;
            double u, v;

            for (k = 0; k < num_loops; k++)
            {
                curr_loop = &loop_samples[k];
                num_samples = curr_loop->num_samples;
                for (j = 0; j < num_samples; j++)
                {
                    u = atan2(curr_loop->samples[j].y, curr_loop->samples[j].x);
                    u = prc_map_to_two_pi(u);
                    v = curr_loop->samples[j].z;

                    curr_loop->uv_samples[j].x = u;
                    curr_loop->uv_samples[j].y = v;
                }

                curr_loop->wind_u = prc_unwrap_angular_component(curr_loop->uv_samples, num_samples, 0);
                curr_loop->wind_v = 0;

                for (j = 0; j < num_samples; j++)
                {
                    curr_loop->uv_samples[j].x = (curr_loop->uv_samples[j].x - cylinder->parameterization.u_param_coeff_b) / u_coeff_a;
                    curr_loop->uv_samples[j].y = (curr_loop->uv_samples[j].y - cylinder->parameterization.v_param_coeff_b) / v_coeff_a;
                }
            }
            break;
        }

        case PRC_TYPE_SURF_Sphere:
        {
            prc_surf_sphere *sphere = surface->surf_sphere;
            double u_coeff_a = (sphere->parameterization.u_param_coeff_a != 0.0) ? sphere->parameterization.u_param_coeff_a : 1.0;
            double v_coeff_a = (sphere->parameterization.v_param_coeff_a != 0.0) ? sphere->parameterization.v_param_coeff_a : 1.0;
            double u, v;
            double horizontal_radius;

            for (k = 0; k < num_loops; k++)
            {
                curr_loop = &loop_samples[k];
                num_samples = curr_loop->num_samples;
                for (j = 0; j < num_samples; j++)
                {
                    u = atan2(curr_loop->samples[j].y, curr_loop->samples[j].x);
                    u = prc_map_to_two_pi(u);
                    horizontal_radius = sqrt(curr_loop->samples[j].y * curr_loop->samples[j].y + curr_loop->samples[j].x * curr_loop->samples[j].x);
                    v = atan2(curr_loop->samples[j].z, horizontal_radius);

                    curr_loop->uv_samples[j].x = u;
                    curr_loop->uv_samples[j].y = v;
                }

                /* v is latitude and does not wrap a full 2*pi (it runs pole
                   to pole), so only u (longitude) is ever unwrapped here */
                curr_loop->wind_u = prc_unwrap_angular_component(curr_loop->uv_samples, num_samples, 0);
                curr_loop->wind_v = 0;

                for (j = 0; j < num_samples; j++)
                {
                    curr_loop->uv_samples[j].x = (curr_loop->uv_samples[j].x - sphere->parameterization.u_param_coeff_b) / u_coeff_a;
                    curr_loop->uv_samples[j].y = (curr_loop->uv_samples[j].y - sphere->parameterization.v_param_coeff_b) / v_coeff_a;
                }
            }
            break;
        }

        case PRC_TYPE_SURF_Torus:
        {
            prc_surf_torus *torus = surface->surf_torus;
            double u_coeff_a = (torus->parameterization.u_param_coeff_a != 0.0) ? torus->parameterization.u_param_coeff_a : 1.0;
            double v_coeff_a = (torus->parameterization.v_param_coeff_a != 0.0) ? torus->parameterization.v_param_coeff_a : 1.0;
            double minor_radius = torus->minor_radius;
            double major_radius = torus->major_radius;
            double u, v, current_radius;

            for (k = 0; k < num_loops; k++)
            {
                curr_loop = &loop_samples[k];
                num_samples = curr_loop->num_samples;
                for (j = 0; j < num_samples; j++)
                {
                    u = atan2(curr_loop->samples[j].y, curr_loop->samples[j].x);
                    u = prc_map_to_two_pi(u);
                    current_radius =
                        sqrt(curr_loop->samples[j].y * curr_loop->samples[j].y + curr_loop->samples[j].x * curr_loop->samples[j].x);
                    v = atan2(curr_loop->samples[j].z, current_radius - major_radius);
                    v = prc_map_to_two_pi(v);

                    curr_loop->uv_samples[j].x = u;
                    curr_loop->uv_samples[j].y = v;
                }

                /* Both axes are angular on a torus, so a loop can wind
                   around either (or, in principle, both -- not yet handled
                   by the tessellation dispatch below) */
                curr_loop->wind_u = prc_unwrap_angular_component(curr_loop->uv_samples, num_samples, 0);
                curr_loop->wind_v = prc_unwrap_angular_component(curr_loop->uv_samples, num_samples, 1);

                for (j = 0; j < num_samples; j++)
                {
                    curr_loop->uv_samples[j].x = (curr_loop->uv_samples[j].x - torus->parameterization.u_param_coeff_b) / u_coeff_a;
                    curr_loop->uv_samples[j].y = (curr_loop->uv_samples[j].y - torus->parameterization.v_param_coeff_b) / v_coeff_a;
                }
            }
            break;
        }

        case PRC_TYPE_SURF_Cylindrical:
        {
            break;
        }

        case PRC_TYPE_SURF_Extrusion:
        {
            break;
        }

        case PRC_TYPE_SURF_Revolution:
        {
            break;
        }

        case PRC_TYPE_SURF_Plane:
        {
            /* The Z = 0 plane which these samples should already be mapped to
               with the inverse transform is local (x, y) space, not (u, v):
               prc_evaluate_surf_plane maps x = u * coeff_a + coeff_b (same for
               v/y), so that affine map has to be inverted here to recover the
               true surface parameters the tessellator evaluates against */
            prc_surf_plane *plane = surface->surf_plane;
            double u_coeff_a = (plane->u_parameter_coeff_a != 0.0) ? plane->u_parameter_coeff_a : 1.0;
            double v_coeff_a = (plane->v_parameter_coeff_a != 0.0) ? plane->v_parameter_coeff_a : 1.0;

            for (k = 0; k < num_loops; k++)
            {
                curr_loop = &loop_samples[k];
                num_samples = curr_loop->num_samples;
                for (j = 0; j < num_samples; j++)
                {
                    curr_loop->uv_samples[j].x = (curr_loop->samples[j].x - plane->u_parameter_coeff_b) / u_coeff_a;
                    curr_loop->uv_samples[j].y = (curr_loop->samples[j].y - plane->v_parameter_coeff_b) / v_coeff_a;
                }
            }
            break;
        }

        case PRC_TYPE_SURF_Offset:
        {
            break;
        }

        case PRC_TYPE_SURF_NURBS:
        {
            /* No closed-form inverse like the analytic surfaces above (cone/
               cylinder/sphere/torus can recover u,v directly from x,y,z via
               atan2/etc.) -- each 3D sample is instead projected back onto
               the surface numerically (coarse grid search + Gauss-Newton,
               prc_project_point_onto_surface), the same machinery already
               used to invert points for the Blend01/02 bound surfaces */
            prc_surf_nurbs *nurbs = surface->surf_nurbs;
            prc_surface_params nurbs_eval_params = { 0 };
            double min_u = nurbs->knot_vector_u[nurbs->du];
            double max_u = nurbs->knot_vector_u[nurbs->highest_index_of_knots_u - nurbs->du];
            double min_v = nurbs->knot_vector_v[nurbs->dv];
            double max_v = nurbs->knot_vector_v[nurbs->highest_index_of_knots_v - nurbs->dv];

            nurbs_eval_params.surface_params = (void *)nurbs;

            for (k = 0; k < num_loops; k++)
            {
                curr_loop = &loop_samples[k];
                num_samples = curr_loop->num_samples;
                for (j = 0; j < num_samples; j++)
                {
                    double u, v;

                    /* Warm-start every sample but the first from its
                       predecessor along the same loop instead of redoing an
                       independent global search: a NURBS surface can nearly
                       meet itself (e.g. a cylinder's own seam), where a
                       fresh coarse search per sample may land on either of
                       two equally valid but discontinuous (u,v) branches --
                       seeding Newton from the previous point's (u,v) keeps
                       it on the same branch, producing a continuous uv loop */
                    if (j == 0)
                        prc_project_point_onto_surface(ctx, prc_evaluate_surf_nurbs, &nurbs_eval_params,
                            min_u, max_u, min_v, max_v, curr_loop->samples[j], &u, &v);
                    else
                        prc_refine_point_on_surface(ctx, prc_evaluate_surf_nurbs, &nurbs_eval_params,
                            min_u, max_u, min_v, max_v, curr_loop->samples[j],
                            curr_loop->uv_samples[j - 1].x, curr_loop->uv_samples[j - 1].y, &u, &v);
                    curr_loop->uv_samples[j].x = u;
                    curr_loop->uv_samples[j].y = v;
#if CHECK_SURFACE_PROJECTION

                    prc_vec3 test_xyz;
                    test_xyz = prc_evaluate_surf_nurbs(ctx, &nurbs_eval_params, u, v);
                    fprintf(stderr, " Nurbs projection: curve_sample=%u delta=(%.9f,%.9f,%.9f)\n",
                        j, curr_loop->samples[j].x - test_xyz.x, curr_loop->samples[j].y - test_xyz.y, curr_loop->samples[j].z - test_xyz.z);
#endif
                }
            }
            break;
        }

        case PRC_TYPE_SURF_Blend02:
        {
            /* No closed-form inverse: u is the center curve's own parameter
               and v is the swept angle (scaled to [0,1] when
               parameterization_type == 0), but the angle itself depends on
               the two bound directions projected from the center curve at
               that u, which vary arbitrarily along the curve -- so each 3D
               sample is projected back numerically, same as NURBS above */
            prc_surf_blend02 *blend = surface->surf_blend02;
            prc_surface_params blend_eval_params = { 0 };
            curve_func center_eval_func = NULL;
            void *center_params = NULL;
            double min_u = 0.0, max_u = 0.0;
            double min_v = 0.0;
            double max_v = (blend->parameterization_type == 0) ? 1.0 : (2.0 * PRC_PI);

            code = prc_get_curve_eval_func(ctx, &blend->center_curve, &center_eval_func,
                &center_params, &min_u, &max_u);
            if (code < 0)
            {
                prc_error(ctx, code, "Invalid center curve type in prc_map_loops_to_surface (Blend02)\n");
                return code;
            }

            blend_eval_params.surface_params = (void *)blend;

            for (k = 0; k < num_loops; k++)
            {
                curr_loop = &loop_samples[k];
                num_samples = curr_loop->num_samples;
                for (j = 0; j < num_samples; j++)
                {
                    double u, v;

                    /* Same warm-start rationale as the NURBS case above */
                    if (j == 0)
                        prc_project_point_onto_surface(ctx, prc_evaluate_surf_blend02, &blend_eval_params,
                            min_u, max_u, min_v, max_v, curr_loop->samples[j], &u, &v);
                    else
                        prc_refine_point_on_surface(ctx, prc_evaluate_surf_blend02, &blend_eval_params,
                            min_u, max_u, min_v, max_v, curr_loop->samples[j],
                            curr_loop->uv_samples[j - 1].x, curr_loop->uv_samples[j - 1].y, &u, &v);
                    curr_loop->uv_samples[j].x = u;
                    curr_loop->uv_samples[j].y = v;
                }
            }
            break;
        }

        case PRC_TYPE_SURF_Blend01:
        {
            break;
        }

        default:
            return 0;
    }

    return 0;
}

/* One node of the circular doubly-linked-list polygon used for ear clipping.
   uv is a plain surface parameter (u, v), not a screen coordinate */
typedef struct prc_tri_vertex_s
{
    prc_vec2 uv;
    uint32_t next;
    uint32_t prev;
    uint8_t removed;
} prc_tri_vertex;

/* Twice the signed area (shoelace formula): positive for CCW, negative for CW */
static double
prc_uv_polygon_signed_area(uint32_t count, const prc_vec2 *pts)
{
    double area = 0.0;
    uint32_t i;

    for (i = 0; i < count; i++)
    {
        uint32_t next = (i + 1 == count) ? 0 : (i + 1);

        area += pts[i].x * pts[next].y - pts[next].x * pts[i].y;
    }

    return area;
}

static double
prc_uv_loop_max_x(const prc_loop_samples *loop)
{
    uint32_t count = (loop->num_samples > 0) ? (loop->num_samples - 1) : 0;
    double max_x;
    uint32_t i;

    if (count == 0)
        return 0.0;

    max_x = loop->uv_samples[0].x;
    for (i = 1; i < count; i++)
    {
        if (loop->uv_samples[i].x > max_x)
            max_x = loop->uv_samples[i].x;
    }

    return max_x;
}

/* Stores count points (dropping any closing duplicate is the caller's job)
   into a circular ring starting at slot base, forcing the winding direction
   given by reverse */
static void
prc_tri_build_ring(prc_tri_vertex *verts, uint32_t base, uint32_t count,
    const prc_vec2 *pts, int reverse)
{
    uint32_t i;

    for (i = 0; i < count; i++)
    {
        uint32_t idx = base + i;

        verts[idx].uv = reverse ? pts[count - 1 - i] : pts[i];
        verts[idx].next = base + ((i + 1) % count);
        verts[idx].prev = base + ((i + count - 1) % count);
        verts[idx].removed = 0;
    }
}

static double
prc_tri_cross(prc_vec2 o, prc_vec2 a, prc_vec2 b)
{
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

static int
prc_tri_point_in_triangle(prc_vec2 p, prc_vec2 a, prc_vec2 b, prc_vec2 c)
{
    double d1 = (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y);
    double d2 = (p.x - c.x) * (b.y - c.y) - (b.x - c.x) * (p.y - c.y);
    double d3 = (p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y);
    int has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    int has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

    return !(has_neg && has_pos);
}

/* idx is an ear of a CCW ring if it turns convex and no other remaining
   vertex of the ring falls inside the candidate triangle */
static int
prc_tri_is_ear(const prc_tri_vertex *verts, uint32_t idx)
{
    uint32_t prev = verts[idx].prev;
    uint32_t next = verts[idx].next;
    prc_vec2 a = verts[prev].uv;
    prc_vec2 b = verts[idx].uv;
    prc_vec2 c = verts[next].uv;
    uint32_t k;

    if (prc_tri_cross(a, b, c) <= 0.0)
        return 0;

    for (k = verts[next].next; k != prev; k = verts[k].next)
    {
        prc_vec2 p = verts[k].uv;

        /* A bridged hole duplicates its entry/exit vertices elsewhere in the
           ring (see prc_tri_eliminate_hole); finding that duplicate here just
           means it coincides with one of this triangle's own corners, which
           is not a real obstruction */
        if ((p.x == a.x && p.y == a.y) || (p.x == b.x && p.y == b.y) || (p.x == c.x && p.y == c.y))
            continue;

        if (prc_tri_point_in_triangle(p, a, b, c))
            return 0;
    }

    return 1;
}

/* Define PRC_DEBUG_EARCLIP to dump the ring state (each vertex's uv, whether
   it is reflex, and whichever other vertex is blocking it as an ear) when
   ear clipping cannot make progress */
#ifdef PRC_DEBUG_EARCLIP
static void
prc_tri_debug_dump_ring(prc_context *ctx, const prc_tri_vertex *verts, uint32_t head, uint32_t ring_count)
{
    uint32_t cur = head;
    uint32_t i;

    (void)ctx;
    fprintf(stderr, "prc_tri_earclip: ring_count=%u\n", ring_count);
    for (i = 0; i < ring_count; i++)
    {
        uint32_t prev = verts[cur].prev;
        uint32_t next = verts[cur].next;
        double cross = prc_tri_cross(verts[prev].uv, verts[cur].uv, verts[next].uv);
        uint32_t blocker = (uint32_t)-1;
        uint32_t k;

        if (cross > 0.0)
        {
            for (k = verts[next].next; k != prev; k = verts[k].next)
            {
                if (prc_tri_point_in_triangle(verts[k].uv, verts[prev].uv, verts[cur].uv, verts[next].uv))
                {
                    blocker = k;
                    break;
                }
            }
        }

        fprintf(stderr, "  [%u] uv=(%.9f,%.9f) prev=%u next=%u cross=%.9g %s\n",
            cur, verts[cur].uv.x, verts[cur].uv.y, prev, next, cross,
            (cross <= 0.0) ? "REFLEX" : (blocker != (uint32_t)-1) ? "BLOCKED" : "EAR");

        if (cross > 0.0 && blocker != (uint32_t)-1)
        {
            fprintf(stderr, "      blocked by [%u] uv=(%.9f,%.9f)\n",
                blocker, verts[blocker].uv.x, verts[blocker].uv.y);
        }

        cur = next;
    }
}
#endif /* PRC_DEBUG_EARCLIP */

/* Standard ear-clipping triangulation of the ring starting at head, which
   must be CCW (prc_tri_eliminate_hole guarantees this even with holes
   bridged in). Allocates and returns the triangle index array */
static int
prc_tri_earclip(prc_context *ctx, prc_tri_vertex *verts, uint32_t head,
    uint32_t ring_count, uint32_t **triangles_out, uint32_t *num_triangles_out)
{
    uint32_t remaining = ring_count;
    uint32_t *triangles;
    uint32_t tri_index = 0;
    uint32_t cur = head;

    if (ring_count < 3)
    {
        *triangles_out = NULL;
        *num_triangles_out = 0;
        return 0;
    }

    triangles = (uint32_t *)prc_calloc(ctx, (ring_count - 2) * 3, sizeof(uint32_t));
    if (triangles == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate triangles in prc_tri_earclip\n");
        return PRC_ERROR_MEMORY;
    }

    while (remaining > 3)
    {
        /* Scan every remaining vertex for a valid ear and take whichever
           produces the "roundest" triangle (smallest longest-edge length),
           rather than just the first one found by a linear scan: a long run
           of exactly-collinear boundary samples (e.g. many points along one
           straight edge of a trimmed loop, or a curve's own dense sampling)
           otherwise lets ear clipping degenerate into one long fan from a
           single vertex -- harmless on a flat plane, but a bad
           approximation on a curved surface, and even on a thin fringe
           strip it still shows up as visible "spoke" triangles */
        uint32_t best = (uint32_t)-1;
        double best_score = 0.0;
        uint32_t start = cur;
        uint32_t scan = cur;

        do
        {
            if (prc_tri_is_ear(verts, scan))
            {
                uint32_t prev = verts[scan].prev;
                uint32_t next = verts[scan].next;
                prc_vec2 a = verts[prev].uv, b = verts[scan].uv, c = verts[next].uv;
                double e0 = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
                double e1 = (b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y);
                double e2 = (c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y);
                double score = e0 > e1 ? e0 : e1;

                score = score > e2 ? score : e2;

                if (best == (uint32_t)-1 || score < best_score)
                {
                    best = scan;
                    best_score = score;
                }
            }
            scan = verts[scan].next;
        } while (scan != start);

        if (best == (uint32_t)-1)
        {
#ifdef PRC_DEBUG_EARCLIP
            prc_tri_debug_dump_ring(ctx, verts, cur, remaining);
#endif
            /* No ear anywhere in a full pass means the ring is degenerate
               or self-intersecting */
            prc_free(ctx, triangles);
            prc_error(ctx, PRC_ERROR_INTERNAL, "Failed to triangulate planar loop boundary\n");
            return PRC_ERROR_INTERNAL;
        }

        {
            uint32_t prev = verts[best].prev;
            uint32_t next = verts[best].next;

            triangles[tri_index++] = prev;
            triangles[tri_index++] = best;
            triangles[tri_index++] = next;

            verts[prev].next = next;
            verts[next].prev = prev;
            verts[best].removed = 1;
            remaining--;
            cur = next;
        }
    }

    triangles[tri_index++] = verts[cur].prev;
    triangles[tri_index++] = cur;
    triangles[tri_index++] = verts[cur].next;

    *triangles_out = triangles;
    *num_triangles_out = ring_count - 2;

    return 0;
}

/* Finds the outer-ring vertex mutually visible with hole_start's uv, so a
   hole can be bridged into the outer ring without crossing it. Classic
   hole-elimination technique: cast a rightward ray from hole_start, take
   the rightmost endpoint of the outer edge it crosses first, then fall back
   to whichever reflex vertex inside that candidate triangle is most aligned
   with the ray, since that one is guaranteed visible */
static int
prc_tri_find_hole_bridge(const prc_tri_vertex *verts, uint32_t outer_head,
    uint32_t hole_start, uint32_t *bridge_out)
{
    prc_vec2 h = verts[hole_start].uv;
    double best_x = 0.0;
    uint8_t have_bridge = 0;
    uint32_t bridge = outer_head;
    uint32_t cur = outer_head;
    prc_vec2 intersect;

    do
    {
        uint32_t nxt = verts[cur].next;
        prc_vec2 a = verts[cur].uv;
        prc_vec2 b = verts[nxt].uv;

        if ((a.y > h.y) != (b.y > h.y))
        {
            double x = a.x + (h.y - a.y) * (b.x - a.x) / (b.y - a.y);

            if (x >= h.x && (!have_bridge || x < best_x))
            {
                best_x = x;
                bridge = (a.x > b.x) ? cur : nxt;
                have_bridge = 1;
            }
        }

        cur = nxt;
    } while (cur != outer_head);

    if (!have_bridge)
        return -1;

    intersect.x = best_x;
    intersect.y = h.y;

    {
        prc_vec2 m_point = verts[bridge].uv;
        uint32_t best_vertex = bridge;
        double best_angle = atan2(fabs(m_point.y - h.y), fabs(m_point.x - h.x));

        cur = outer_head;
        do
        {
            prc_vec2 p = verts[cur].uv;

            if (p.x >= h.x && prc_tri_point_in_triangle(p, h, intersect, m_point))
            {
                double angle = atan2(fabs(p.y - h.y), fabs(p.x - h.x));

                if (angle < best_angle)
                {
                    best_angle = angle;
                    best_vertex = cur;
                }
            }

            cur = verts[cur].next;
        } while (cur != outer_head);

        bridge = best_vertex;
    }

    *bridge_out = bridge;

    return 0;
}

/* Splices a hole ring (hole_head, hole_count vertices, CW) into the outer
   ring by duplicating the bridge vertex and the hole's own entry vertex,
   producing a single simple (if zero-width-slit) CCW ring */
static int
prc_tri_eliminate_hole(prc_context *ctx, prc_tri_vertex *verts, uint32_t *next_free_slot,
    uint32_t outer_head, uint32_t hole_head, uint32_t hole_count)
{
    uint32_t hole_start = hole_head;
    uint32_t cur = verts[hole_head].next;
    uint32_t k;
    uint32_t bridge;
    uint32_t h_dup, m_dup;
    uint32_t hole_last;
    uint32_t old_after_bridge;
    int code;

    /* The rightmost hole vertex is the conventional bridge entry point */
    for (k = 1; k < hole_count; k++)
    {
        if (verts[cur].uv.x > verts[hole_start].uv.x)
            hole_start = cur;
        cur = verts[cur].next;
    }

    code = prc_tri_find_hole_bridge(verts, outer_head, hole_start, &bridge);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "No visible bridge to outer loop in prc_tri_eliminate_hole\n");
        return PRC_ERROR_INTERNAL;
    }

    h_dup = (*next_free_slot)++;
    m_dup = (*next_free_slot)++;
    verts[h_dup].uv = verts[hole_start].uv;
    verts[m_dup].uv = verts[bridge].uv;
    verts[h_dup].removed = 0;
    verts[m_dup].removed = 0;

    old_after_bridge = verts[bridge].next;
    hole_last = verts[hole_start].prev;

    verts[bridge].next = hole_start;
    verts[hole_start].prev = bridge;

    verts[hole_last].next = h_dup;
    verts[h_dup].prev = hole_last;
    verts[h_dup].next = m_dup;
    verts[m_dup].prev = h_dup;

    verts[m_dup].next = old_after_bridge;
    verts[old_after_bridge].prev = m_dup;

    return 0;
}

/* Builds one ear-clippable ring from the outer loop with every hole loop
   bridged in, triangulates it, and hands back the (u, v) vertex positions
   the triangle indices refer to. Caller owns and must free *verts_out and
   *triangles_out */
static int
prc_triangulate_planar_loops(prc_context *ctx, uint32_t num_loops, prc_loop_samples *loop_samples,
    prc_vec2 **verts_out, uint32_t *num_verts_out,
    uint32_t **triangles_out, uint32_t *num_triangles_out)
{
    uint32_t outer_index = num_loops;
    uint32_t outer_count = 0;
    uint32_t total_capacity = 0;
    uint32_t num_holes = 0;
    uint32_t k;
    prc_tri_vertex *verts;
    uint32_t next_free;
    uint32_t outer_head;
    uint32_t *hole_order = NULL;
    int code;

    for (k = 0; k < num_loops; k++)
    {
        uint32_t count = (loop_samples[k].num_samples > 0) ? (loop_samples[k].num_samples - 1) : 0;

        total_capacity += count;
        if (loop_samples[k].is_outer_loop)
        {
            outer_index = k;
            outer_count = count;
        }
        else
        {
            num_holes++;
        }
    }

    if (outer_index == num_loops || outer_count < 3)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "No usable outer loop in prc_triangulate_planar_loops\n");
        return PRC_ERROR_INTERNAL;
    }

    total_capacity += num_holes * 2;   /* two duplicate vertices per bridged hole */

    verts = (prc_tri_vertex *)prc_calloc(ctx, total_capacity, sizeof(prc_tri_vertex));
    if (verts == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate loop vertices in prc_triangulate_planar_loops\n");
        return PRC_ERROR_MEMORY;
    }

    prc_tri_build_ring(verts, 0, outer_count, loop_samples[outer_index].uv_samples,
        prc_uv_polygon_signed_area(outer_count, loop_samples[outer_index].uv_samples) < 0.0);
    outer_head = 0;
    next_free = outer_count;

    if (num_holes > 0)
    {
        uint32_t idx = 0;

        hole_order = (uint32_t *)prc_calloc(ctx, num_holes, sizeof(uint32_t));
        if (hole_order == NULL)
        {
            prc_free(ctx, verts);
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate hole order in prc_triangulate_planar_loops\n");
            return PRC_ERROR_MEMORY;
        }

        for (k = 0; k < num_loops; k++)
        {
            if (k != outer_index && !loop_samples[k].is_outer_loop)
                hole_order[idx++] = k;
        }

        /* Merge farthest-right holes first so bridge segments cannot cross */
        for (k = 1; k < num_holes; k++)
        {
            uint32_t cand = hole_order[k];
            double cand_max_x = prc_uv_loop_max_x(&loop_samples[cand]);
            int m = (int)k - 1;

            while (m >= 0 && prc_uv_loop_max_x(&loop_samples[hole_order[m]]) < cand_max_x)
            {
                hole_order[m + 1] = hole_order[m];
                m--;
            }
            hole_order[m + 1] = cand;
        }

        for (k = 0; k < num_holes; k++)
        {
            uint32_t li = hole_order[k];
            uint32_t count = (loop_samples[li].num_samples > 0) ? (loop_samples[li].num_samples - 1) : 0;
            uint32_t hole_head = next_free;

            if (count < 3)
                continue;   /* degenerate hole loop; skip rather than fail the whole face */

            prc_tri_build_ring(verts, hole_head, count, loop_samples[li].uv_samples,
                prc_uv_polygon_signed_area(count, loop_samples[li].uv_samples) > 0.0);
            next_free += count;

            code = prc_tri_eliminate_hole(ctx, verts, &next_free, outer_head, hole_head, count);
            if (code < 0)
            {
                prc_free(ctx, hole_order);
                prc_free(ctx, verts);
                return code;
            }
        }

        prc_free(ctx, hole_order);
    }

    code = prc_tri_earclip(ctx, verts, outer_head, next_free, triangles_out, num_triangles_out);
    if (code < 0)
    {
        prc_free(ctx, verts);
        return code;
    }

    *verts_out = (prc_vec2 *)prc_calloc(ctx, next_free, sizeof(prc_vec2));
    if (*verts_out == NULL)
    {
        prc_free(ctx, verts);
        prc_free(ctx, *triangles_out);
        *triangles_out = NULL;
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate output vertices in prc_triangulate_planar_loops\n");
        return PRC_ERROR_MEMORY;
    }
    for (k = 0; k < next_free; k++)
        (*verts_out)[k] = verts[k].uv;
    *num_verts_out = next_free;

    prc_free(ctx, verts);

    return 0;
}

/* Tessellates a planar face directly from its loop boundaries instead of the
   regular parametric grid: the plane's own domain need not match the face at
   all, only the loops define its real outer edge and holes */
static int
prc_tessellate_planar_face_from_loops(prc_context *ctx, prc_data *data, uint32_t shell_index,
    uint32_t face_index, uint32_t geom_count, uint8_t orientation,
    surface_func surface_eval_func, prc_surface_params *surf_params,
    const prc_surface_sampling_info *sampling_info,
    uint32_t num_loops, prc_loop_samples *loop_samples)
{
    prc_vec2 *poly_verts = NULL;
    uint32_t poly_num_verts = 0;
    uint32_t *poly_triangles = NULL;
    uint32_t poly_num_triangles = 0;
    prc_exact_geom_tess_data *tess_data;
    prc_vec3 normal;
    uint32_t v, t;
    int code;

    code = prc_triangulate_planar_loops(ctx, num_loops, loop_samples,
        &poly_verts, &poly_num_verts, &poly_triangles, &poly_num_triangles);
    if (code < 0)
    {
        prc_error(ctx, code, "Failed in prc_triangulate_planar_loops\n");
        return code;
    }

    data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data =
        (prc_exact_geom_tess_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_tess_data));
    if (data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data == NULL)
    {
        prc_free(ctx, poly_verts);
        prc_free(ctx, poly_triangles);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data in prc_tessellate_surface\n");
        return PRC_ERROR_MEMORY;
    }
    tess_data = data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data;

    tess_data->number_of_vertices = poly_num_verts;
    tess_data->vertices = (prc_exact_geom_vertex *)prc_calloc(ctx, poly_num_verts, sizeof(prc_exact_geom_vertex));
    if (tess_data->vertices == NULL)
    {
        prc_free(ctx, poly_verts);
        prc_free(ctx, poly_triangles);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data vertices in prc_tessellate_surface\n");
        return PRC_ERROR_MEMORY;
    }

    /* The face is flat, so one evaluation gives the normal for every vertex */
    code = prc_compute_surface_normal(ctx, surface_eval_func, surf_params,
        poly_verts[0].x, poly_verts[0].y, PLANE_SURFACE_PRECISION, PLANE_SURFACE_PRECISION,
        poly_verts[0].x - 1.0, poly_verts[0].x + 1.0,
        poly_verts[0].y - 1.0, poly_verts[0].y + 1.0,
        sampling_info, orientation, &normal);
    if (code < 0)
    {
        prc_free(ctx, poly_verts);
        prc_free(ctx, poly_triangles);
        return code;
    }

    for (v = 0; v < poly_num_verts; v++)
    {
        prc_vec3 position = surface_eval_func(ctx, surf_params, poly_verts[v].x, poly_verts[v].y);

        tess_data->vertices[v].position[0] = (float)position.x;
        tess_data->vertices[v].position[1] = (float)position.y;
        tess_data->vertices[v].position[2] = (float)position.z;
        tess_data->vertices[v].normal[0] = (float)normal.x;
        tess_data->vertices[v].normal[1] = (float)normal.y;
        tess_data->vertices[v].normal[2] = (float)normal.z;
    }

    tess_data->number_of_triangles = poly_num_triangles;
    tess_data->triangles = (uint32_t *)prc_calloc(ctx, poly_num_triangles * 3, sizeof(uint32_t));
    if (tess_data->triangles == NULL)
    {
        prc_free(ctx, poly_verts);
        prc_free(ctx, poly_triangles);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data triangles in prc_tessellate_surface\n");
        return PRC_ERROR_MEMORY;
    }

    /* Ear clipping walked the CCW ring, which is the winding orientation == 1 expects */
    for (t = 0; t < poly_num_triangles; t++)
    {
        uint32_t i0 = poly_triangles[t * 3 + 0];
        uint32_t i1 = poly_triangles[t * 3 + 1];
        uint32_t i2 = poly_triangles[t * 3 + 2];

        if (orientation == 0)
        {
            tess_data->triangles[t * 3 + 0] = i0;
            tess_data->triangles[t * 3 + 1] = i2;
            tess_data->triangles[t * 3 + 2] = i1;
        }
        else
        {
            tess_data->triangles[t * 3 + 0] = i0;
            tess_data->triangles[t * 3 + 1] = i1;
            tess_data->triangles[t * 3 + 2] = i2;
        }
    }

    prc_free(ctx, poly_verts);
    prc_free(ctx, poly_triangles);

    return 0;
}

/* Computes a per-vertex surface normal for a point produced by loop-based
   (rather than regular-grid) tessellation, where u/v may legitimately fall
   outside the surface's nominal [start_u,end_u]x[start_v,end_v] domain (loop
   samples are unwrapped, so a periodic axis can run past one period tile).
   Sidesteps prc_compute_surface_normal's domain clamping/wrapping by feeding
   it a synthetic wide range that never clips the finite-difference step,
   the same trick prc_tessellate_planar_face_from_loops uses */
static int
prc_compute_loop_vertex_normal(prc_context *ctx, surface_func surface_eval_func,
    void *surf_params, double u, double v, double precision_u, double precision_v,
    const prc_surface_sampling_info *sampling_info, uint8_t orientation, prc_vec3 *normal)
{
    prc_surface_sampling_info local_info = *sampling_info;

    local_info.u_periodic = 0;
    local_info.v_periodic = 0;

    return prc_compute_surface_normal(ctx, surface_eval_func, surf_params,
        u, v, precision_u, precision_v,
        u - 1.0, u + 1.0, v - 1.0, v + 1.0,
        &local_info, orientation, normal);
}

/* Recursively centroid-splits a curved triangle (given as 3 uv corners)
   until the true surface point at its centroid is within tolerance of the
   flat (linearly-interpolated) centroid, or max_depth is reached. A split
   only ever adds a new interior point private to that one triangle -- it
   never touches a shared edge -- so, unlike edge-midpoint subdivision,
   neighboring triangles independently refined to different depths can never
   open a crack between them. Leaf triangles are appended as a flat,
   non-indexed triangle soup (3 fresh vertices each) to the growable output
   arrays, doubling capacity via prc_realloc as needed. Caller owns and must
   free *out_verts/*out_tris */
static int
prc_subdivide_curved_triangle(prc_context *ctx, surface_func surface_eval_func, void *surf_params,
    prc_vec2 uv0, prc_vec2 uv1, prc_vec2 uv2, double tolerance, uint32_t depth, uint32_t max_depth,
    prc_vec2 **out_verts, uint32_t *out_num_verts, uint32_t *out_verts_cap,
    uint32_t **out_tris, uint32_t *out_num_triangles, uint32_t *out_tris_cap)
{
    prc_vec3 p0, p1, p2, centroid_linear, centroid_true;
    prc_vec2 uvc;
    int code;

    p0 = surface_eval_func(ctx, surf_params, uv0.x, uv0.y);
    p1 = surface_eval_func(ctx, surf_params, uv1.x, uv1.y);
    p2 = surface_eval_func(ctx, surf_params, uv2.x, uv2.y);

    uvc.x = (uv0.x + uv1.x + uv2.x) / 3.0;
    uvc.y = (uv0.y + uv1.y + uv2.y) / 3.0;
    centroid_true = surface_eval_func(ctx, surf_params, uvc.x, uvc.y);
    centroid_linear.x = (p0.x + p1.x + p2.x) / 3.0;
    centroid_linear.y = (p0.y + p1.y + p2.y) / 3.0;
    centroid_linear.z = (p0.z + p1.z + p2.z) / 3.0;

    if (depth >= max_depth || prc_vec_dist_between_two_points(centroid_true, centroid_linear) <= tolerance)
    {
        if (*out_num_verts + 3 > *out_verts_cap)
        {
            uint32_t new_cap = (*out_verts_cap == 0) ? 64 : (*out_verts_cap * 2);
            prc_vec2 *new_verts;

            while (new_cap < *out_num_verts + 3)
                new_cap *= 2;
            new_verts = (prc_vec2 *)prc_realloc(ctx, *out_verts, new_cap * sizeof(prc_vec2));
            if (new_verts == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to grow vertices in prc_subdivide_curved_triangle\n");
                return PRC_ERROR_MEMORY;
            }
            *out_verts = new_verts;
            *out_verts_cap = new_cap;
        }
        if (*out_num_triangles + 1 > *out_tris_cap)
        {
            uint32_t new_cap = (*out_tris_cap == 0) ? 32 : (*out_tris_cap * 2);
            uint32_t *new_tris = (uint32_t *)prc_realloc(ctx, *out_tris, new_cap * 3 * sizeof(uint32_t));

            if (new_tris == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to grow triangles in prc_subdivide_curved_triangle\n");
                return PRC_ERROR_MEMORY;
            }
            *out_tris = new_tris;
            *out_tris_cap = new_cap;
        }

        (*out_verts)[*out_num_verts + 0] = uv0;
        (*out_verts)[*out_num_verts + 1] = uv1;
        (*out_verts)[*out_num_verts + 2] = uv2;
        (*out_tris)[*out_num_triangles * 3 + 0] = *out_num_verts + 0;
        (*out_tris)[*out_num_triangles * 3 + 1] = *out_num_verts + 1;
        (*out_tris)[*out_num_triangles * 3 + 2] = *out_num_verts + 2;
        *out_num_verts += 3;
        *out_num_triangles += 1;
        return 0;
    }

    code = prc_subdivide_curved_triangle(ctx, surface_eval_func, surf_params, uv0, uv1, uvc, tolerance, depth + 1, max_depth,
        out_verts, out_num_verts, out_verts_cap, out_tris, out_num_triangles, out_tris_cap);
    if (code < 0)
        return code;
    code = prc_subdivide_curved_triangle(ctx, surface_eval_func, surf_params, uv1, uv2, uvc, tolerance, depth + 1, max_depth,
        out_verts, out_num_verts, out_verts_cap, out_tris, out_num_triangles, out_tris_cap);
    if (code < 0)
        return code;
    code = prc_subdivide_curved_triangle(ctx, surface_eval_func, surf_params, uv2, uv0, uvc, tolerance, depth + 1, max_depth,
        out_verts, out_num_verts, out_verts_cap, out_tris, out_num_triangles, out_tris_cap);
    if (code < 0)
        return code;

    return 0;
}

/* One tessellation of a surface's full natural uv domain on a regular,
   curvature-adaptive grid (same refinement prc_tessellate_surface's
   untrimmed path already used) -- shared by the untrimmed path and by
   prc_tessellate_trimmed_face, which keeps whichever cells fall inside a
   face's trim loop(s) and only needs to ear-clip-fill the thin leftover
   fringe. Caller owns and must free grid->uv and grid->verts */
typedef struct prc_regular_grid_s
{
    prc_vec2 *uv;
    prc_exact_geom_vertex *verts;
    uint32_t vertex_samples_u;
    uint32_t vertex_samples_v;
    uint8_t wrap_u;
    uint8_t wrap_v;
} prc_regular_grid;

/* An undirected pair of grid vertex indices, used to represent one boundary
   edge while tracing the outline of a classified grid's kept cells */
typedef struct prc_uint32_pair_s
{
    uint32_t a;
    uint32_t b;
} prc_uint32_pair;

static int
prc_build_regular_grid(prc_context *ctx, surface_func surface_eval_func, prc_surface_params *surf_params,
    const prc_surface_sampling_info *sampling_info_in, uint8_t orientation, prc_regular_grid *grid_out)
{
    prc_surface_sampling_info sampling_info = *sampling_info_in;
    uint32_t num_samples_u = sampling_info.num_samples_u;
    uint32_t num_samples_v = sampling_info.num_samples_v;
    double precision_u = sampling_info.precision_u;
    double precision_v = sampling_info.precision_v;
    uint32_t max_samples_u = sampling_info.max_samples_u;
    uint32_t max_samples_v = sampling_info.max_samples_v;
    double start_u = sampling_info.start_u;
    double start_v = sampling_info.start_v;
    double end_u = sampling_info.end_u;
    double end_v = sampling_info.end_v;
    uint8_t wrap_u, wrap_v;
    uint32_t vertex_samples_u, vertex_samples_v, cell_count_u, cell_count_v;
    uint8_t surface_approx_good = 0;
    int code;

    wrap_u = prc_surface_axis_wraps(start_u, end_u, sampling_info.u_periodic, sampling_info.u_period);
    wrap_v = prc_surface_axis_wraps(start_v, end_v, sampling_info.v_periodic, sampling_info.v_period);
    vertex_samples_u = wrap_u ? (num_samples_u - 1) : num_samples_u;
    vertex_samples_v = wrap_v ? (num_samples_v - 1) : num_samples_v;
    cell_count_u = wrap_u ? vertex_samples_u : (vertex_samples_u - 1);
    cell_count_v = wrap_v ? vertex_samples_v : (vertex_samples_v - 1);

    while (!surface_approx_good)
    {
        uint32_t i, j;
        uint8_t refine_u = 0;
        uint8_t refine_v = 0;

        surface_approx_good = 1;

        if (!sampling_info.u_linear)
        {
            for (j = 0; j < vertex_samples_v; j++)
            {
                for (i = 0; i < cell_count_u; i++)
                {
                    uint32_t next_i = (i + 1 == vertex_samples_u && wrap_u) ? 0 : (i + 1);
                    double u0 = prc_get_surface_param(start_u, end_u, i, vertex_samples_u, wrap_u);
                    double u1 = prc_get_surface_param(start_u, end_u, next_i, vertex_samples_u, wrap_u);
                    double v = prc_get_surface_param(start_v, end_v, j, vertex_samples_v, wrap_v);
                    prc_vec3 p0 = surface_eval_func(ctx, surf_params, u0, v);
                    prc_vec3 p1 = surface_eval_func(ctx, surf_params, u1, v);
                    prc_vec3 mid = surface_eval_func(ctx, surf_params, 0.5 * (u0 + u1), v);
                    prc_vec3 seg_mid;

                    seg_mid.x = 0.5 * (p0.x + p1.x);
                    seg_mid.y = 0.5 * (p0.y + p1.y);
                    seg_mid.z = 0.5 * (p0.z + p1.z);

                    if (prc_vec_dist_between_two_points(mid, seg_mid) > precision_u)
                    {
                        refine_u = 1;
                        break;
                    }
                }

                if (refine_u)
                    break;
            }
        }

        if (!sampling_info.v_linear)
        {
            for (i = 0; i < vertex_samples_u; i++)
            {
                for (j = 0; j < cell_count_v; j++)
                {
                    uint32_t next_j = (j + 1 == vertex_samples_v && wrap_v) ? 0 : (j + 1);
                    double u = prc_get_surface_param(start_u, end_u, i, vertex_samples_u, wrap_u);
                    double v0 = prc_get_surface_param(start_v, end_v, j, vertex_samples_v, wrap_v);
                    double v1 = prc_get_surface_param(start_v, end_v, next_j, vertex_samples_v, wrap_v);
                    prc_vec3 p0 = surface_eval_func(ctx, surf_params, u, v0);
                    prc_vec3 p1 = surface_eval_func(ctx, surf_params, u, v1);
                    prc_vec3 mid = surface_eval_func(ctx, surf_params, u, 0.5 * (v0 + v1));
                    prc_vec3 seg_mid;

                    seg_mid.x = 0.5 * (p0.x + p1.x);
                    seg_mid.y = 0.5 * (p0.y + p1.y);
                    seg_mid.z = 0.5 * (p0.z + p1.z);

                    if (prc_vec_dist_between_two_points(mid, seg_mid) > precision_v)
                    {
                        refine_v = 1;
                        break;
                    }
                }

                if (refine_v)
                    break;
            }
        }

        surface_approx_good = !(refine_u || refine_v);

        if (!surface_approx_good)
        {
            if ((refine_u && num_samples_u >= max_samples_u) ||
                (refine_v && num_samples_v >= max_samples_v))
            {
                surface_approx_good = 1;
                break;
            }

            if (refine_u)
                num_samples_u *= 2;
            if (refine_v)
                num_samples_v *= 2;

            vertex_samples_u = wrap_u ? (num_samples_u - 1) : num_samples_u;
            vertex_samples_v = wrap_v ? (num_samples_v - 1) : num_samples_v;
            cell_count_u = wrap_u ? vertex_samples_u : (vertex_samples_u - 1);
            cell_count_v = wrap_v ? vertex_samples_v : (vertex_samples_v - 1);
        }
    }

    grid_out->vertex_samples_u = vertex_samples_u;
    grid_out->vertex_samples_v = vertex_samples_v;
    grid_out->wrap_u = wrap_u;
    grid_out->wrap_v = wrap_v;
    grid_out->uv = (prc_vec2 *)prc_calloc(ctx, vertex_samples_u * vertex_samples_v, sizeof(prc_vec2));
    grid_out->verts = (prc_exact_geom_vertex *)prc_calloc(ctx, vertex_samples_u * vertex_samples_v, sizeof(prc_exact_geom_vertex));
    if (grid_out->uv == NULL || grid_out->verts == NULL)
    {
        prc_free(ctx, grid_out->uv);
        prc_free(ctx, grid_out->verts);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure in prc_build_regular_grid\n");
        return PRC_ERROR_MEMORY;
    }

    {
        uint32_t i, j;
        double du = (vertex_samples_u > 1) ? fabs(end_u - start_u) /
            (double)(wrap_u ? vertex_samples_u : (vertex_samples_u - 1)) : 1.0;
        double dv = (vertex_samples_v > 1) ? fabs(end_v - start_v) /
            (double)(wrap_v ? vertex_samples_v : (vertex_samples_v - 1)) : 1.0;

        for (j = 0; j < vertex_samples_v; j++)
        {
            for (i = 0; i < vertex_samples_u; i++)
            {
                uint32_t vertex_index = j * vertex_samples_u + i;
                prc_vec3 position;
                prc_vec3 normal;
                double u = prc_get_surface_param(start_u, end_u, i, vertex_samples_u, wrap_u);
                double v = prc_get_surface_param(start_v, end_v, j, vertex_samples_v, wrap_v);

                position = surface_eval_func(ctx, surf_params, u, v);

                code = prc_compute_surface_normal(ctx, surface_eval_func, surf_params,
                    u, v, du, dv, start_u, end_u, start_v, end_v, &sampling_info,
                    orientation, &normal);
                if (code < 0)
                {
                    prc_free(ctx, grid_out->uv);
                    prc_free(ctx, grid_out->verts);
                    return code;
                }

                grid_out->uv[vertex_index].x = u;
                grid_out->uv[vertex_index].y = v;
                grid_out->verts[vertex_index].position[0] = (float)position.x;
                grid_out->verts[vertex_index].position[1] = (float)position.y;
                grid_out->verts[vertex_index].position[2] = (float)position.z;
                grid_out->verts[vertex_index].normal[0] = (float)normal.x;
                grid_out->verts[vertex_index].normal[1] = (float)normal.y;
                grid_out->verts[vertex_index].normal[2] = (float)normal.z;
            }
        }
    }

    return 0;
}

/* Emits the standard 2-triangles-per-cell tessellation of every cell in a
   regular grid (the untrimmed case, where every cell is kept) */
static int
prc_emit_grid_all_triangles(prc_context *ctx, const prc_regular_grid *grid, uint8_t orientation,
    uint32_t **triangles_out, uint32_t *num_triangles_out)
{
    uint32_t cell_count_u = grid->wrap_u ? grid->vertex_samples_u : (grid->vertex_samples_u - 1);
    uint32_t cell_count_v = grid->wrap_v ? grid->vertex_samples_v : (grid->vertex_samples_v - 1);
    uint32_t *triangles;
    uint32_t triangle_index = 0;
    uint32_t i, j;

    triangles = (uint32_t *)prc_calloc(ctx, cell_count_u * cell_count_v * 6, sizeof(uint32_t));
    if (triangles == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure in prc_emit_grid_all_triangles\n");
        return PRC_ERROR_MEMORY;
    }

    for (j = 0; j < cell_count_v; j++)
    {
        for (i = 0; i < cell_count_u; i++)
        {
            uint32_t next_i = (i + 1 == grid->vertex_samples_u && grid->wrap_u) ? 0 : (i + 1);
            uint32_t next_j = (j + 1 == grid->vertex_samples_v && grid->wrap_v) ? 0 : (j + 1);
            uint32_t i00 = j * grid->vertex_samples_u + i;
            uint32_t i10 = j * grid->vertex_samples_u + next_i;
            uint32_t i01 = next_j * grid->vertex_samples_u + i;
            uint32_t i11 = next_j * grid->vertex_samples_u + next_i;

            if (orientation == 0)
            {
                triangles[triangle_index++] = i00;
                triangles[triangle_index++] = i01;
                triangles[triangle_index++] = i10;
                triangles[triangle_index++] = i10;
                triangles[triangle_index++] = i01;
                triangles[triangle_index++] = i11;
            }
            else
            {
                triangles[triangle_index++] = i00;
                triangles[triangle_index++] = i10;
                triangles[triangle_index++] = i01;
                triangles[triangle_index++] = i10;
                triangles[triangle_index++] = i11;
                triangles[triangle_index++] = i01;
            }
        }
    }

    *triangles_out = triangles;
    *num_triangles_out = cell_count_u * cell_count_v * 2;
    return 0;
}

/* Finds where a loop that winds once around uv axis wrap_axis crosses the
   query point's own coordinate on that axis, and reports whether the query
   point is on the loop's material side there (interior-on-the-left
   convention -- see prc_build_periodic_outer_loop) */
static int
prc_wrapping_loop_side(const prc_loop_samples *loop, uint8_t wrap_axis, int32_t wind, prc_vec2 q)
{
    uint32_t count = (loop->num_samples > 0) ? (loop->num_samples - 1) : 0;
    double a_first, a_last, period, mid, qa, qa_shifted, qb;
    uint32_t i;

    if (count < 2)
        return 1;

    a_first = wrap_axis ? loop->uv_samples[0].y : loop->uv_samples[0].x;
    a_last = wrap_axis ? loop->uv_samples[count].y : loop->uv_samples[count].x;
    period = fabs(a_last - a_first);
    if (period <= 0.0)
        return 1;

    mid = (a_first + a_last) / 2.0;
    qa = wrap_axis ? q.y : q.x;
    qb = wrap_axis ? q.x : q.y;
    qa_shifted = qa + period * (double)lround((mid - qa) / period);

    for (i = 0; i < count; i++)
    {
        double ai = wrap_axis ? loop->uv_samples[i].y : loop->uv_samples[i].x;
        double bi = wrap_axis ? loop->uv_samples[i].x : loop->uv_samples[i].y;
        double ai1 = wrap_axis ? loop->uv_samples[i + 1].y : loop->uv_samples[i + 1].x;
        double bi1 = wrap_axis ? loop->uv_samples[i + 1].x : loop->uv_samples[i + 1].y;

        if ((ai <= qa_shifted && qa_shifted <= ai1) || (ai1 <= qa_shifted && qa_shifted <= ai))
        {
            double t = (ai1 != ai) ? (qa_shifted - ai) / (ai1 - ai) : 0.0;
            double b_curve = bi + t * (bi1 - bi);

            return (wind > 0) ? (qb > b_curve) : (qb < b_curve);
        }
    }

    /* No bracketing segment found (shouldn't happen for a loop spanning a
       full period) -- fall back to comparing against the nearest endpoint */
    {
        double b_curve = (fabs(qa_shifted - a_first) < fabs(qa_shifted - a_last)) ?
            (wrap_axis ? loop->uv_samples[0].x : loop->uv_samples[0].y) :
            (wrap_axis ? loop->uv_samples[count].x : loop->uv_samples[count].y);

        return (wind > 0) ? (qb > b_curve) : (qb < b_curve);
    }
}

/* Whether uv point q lies inside the region a face's loop(s) bound.
   Non-wrapping loops use point-in-polygon nesting (inside an outer loop,
   outside every hole loop); a loop that winds once around a periodic axis
   instead divides the surface along that axis, classified via
   prc_wrapping_loop_side. Multiple loops simply AND together */
static int
prc_point_inside_trimmed_region(uint32_t num_loops, const prc_loop_samples *loop_samples, prc_vec2 q)
{
    uint32_t k;

    for (k = 0; k < num_loops; k++)
    {
        const prc_loop_samples *loop = &loop_samples[k];
        uint32_t count = (loop->num_samples > 0) ? (loop->num_samples - 1) : 0;

        /* A loop collapsed to a single point (e.g. a cone's apex) bounds
           zero area -- it can never exclude a grid point, so skip it rather
           than running it through point-in-polygon */
        if (loop->is_point_loop)
            continue;

        if (loop->wind_u != 0 || loop->wind_v != 0)
        {
            uint8_t wrap_axis = (loop->wind_u != 0) ? 0 : 1;
            int32_t wind = wrap_axis ? loop->wind_v : loop->wind_u;

            if (!prc_wrapping_loop_side(loop, wrap_axis, wind, q))
                return 0;
        }
        else if (count >= 3)
        {
            int contains = prc_uv_point_in_polygon(q, count, loop->uv_samples);

            if (loop->is_outer_loop)
            {
                if (!contains)
                    return 0;
            }
            else if (contains)
            {
                return 0;
            }
        }
    }

    return 1;
}

static int32_t
prc_wrap_cell_index(int32_t idx, uint32_t cell_count, uint8_t wraps)
{
    if (wraps)
    {
        int32_t m = (int32_t)cell_count;

        return ((idx % m) + m) % m;
    }
    if (idx < 0 || idx >= (int32_t)cell_count)
        return -1;
    return idx;
}

/* Traces the boundary/boundaries of the "kept" (fully inside) cells of a
   classified regular grid, marching-squares style: any cell edge whose
   neighboring cell is either out of range (a true domain edge) or not kept
   is a boundary edge; those edges are stitched end-to-end into one or more
   closed polylines. Returns each traced polyline as a prc_loop_samples
   (caller owns and must free each entry's uv_samples, and the array itself) */
static int
prc_trace_kept_region_boundaries(prc_context *ctx, const uint8_t *cell_inside,
    uint32_t vertex_samples_u, uint32_t vertex_samples_v, uint8_t wrap_u, uint8_t wrap_v,
    const prc_vec2 *grid_uv, prc_loop_samples **out_loops, uint32_t *out_num_loops)
{
    uint32_t cell_count_u = wrap_u ? vertex_samples_u : (vertex_samples_u - 1);
    uint32_t cell_count_v = wrap_v ? vertex_samples_v : (vertex_samples_v - 1);
    uint32_t num_verts = vertex_samples_u * vertex_samples_v;
    uint32_t i, j, k;
    prc_uint32_pair *edges;
    uint32_t num_edges = 0, edges_cap;
    uint32_t *adj0, *adj1;
    uint8_t *visited;
    prc_loop_samples *loops = NULL;
    uint32_t num_loops = 0, loops_cap = 0;

    edges_cap = cell_count_u * cell_count_v * 4 + 4;
    edges = (prc_uint32_pair *)prc_calloc(ctx, edges_cap, sizeof(prc_uint32_pair));
    if (edges == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate edges in prc_trace_kept_region_boundaries\n");
        return PRC_ERROR_MEMORY;
    }

    for (j = 0; j < cell_count_v; j++)
    {
        for (i = 0; i < cell_count_u; i++)
        {
            uint32_t vnext_i, vnext_j;
            int32_t ni_left, ni_right, nj_below, nj_above;

            if (!cell_inside[j * cell_count_u + i])
                continue;

            vnext_i = (i + 1 == vertex_samples_u && wrap_u) ? 0 : (i + 1);
            vnext_j = (j + 1 == vertex_samples_v && wrap_v) ? 0 : (j + 1);

            nj_below = prc_wrap_cell_index((int32_t)j - 1, cell_count_v, wrap_v);
            if (nj_below < 0 || !cell_inside[(uint32_t)nj_below * cell_count_u + i])
            {
                edges[num_edges].a = j * vertex_samples_u + i;
                edges[num_edges].b = j * vertex_samples_u + vnext_i;
                num_edges++;
            }

            nj_above = prc_wrap_cell_index((int32_t)j + 1, cell_count_v, wrap_v);
            if (nj_above < 0 || !cell_inside[(uint32_t)nj_above * cell_count_u + i])
            {
                edges[num_edges].a = vnext_j * vertex_samples_u + vnext_i;
                edges[num_edges].b = vnext_j * vertex_samples_u + i;
                num_edges++;
            }

            ni_left = prc_wrap_cell_index((int32_t)i - 1, cell_count_u, wrap_u);
            if (ni_left < 0 || !cell_inside[j * cell_count_u + (uint32_t)ni_left])
            {
                edges[num_edges].a = vnext_j * vertex_samples_u + i;
                edges[num_edges].b = j * vertex_samples_u + i;
                num_edges++;
            }

            ni_right = prc_wrap_cell_index((int32_t)i + 1, cell_count_u, wrap_u);
            if (ni_right < 0 || !cell_inside[j * cell_count_u + (uint32_t)ni_right])
            {
                edges[num_edges].a = j * vertex_samples_u + vnext_i;
                edges[num_edges].b = vnext_j * vertex_samples_u + vnext_i;
                num_edges++;
            }
        }
    }

    if (num_edges == 0)
    {
        prc_free(ctx, edges);
        *out_loops = NULL;
        *out_num_loops = 0;
        return 0;
    }

    adj0 = (uint32_t *)prc_calloc(ctx, num_verts, sizeof(uint32_t));
    adj1 = (uint32_t *)prc_calloc(ctx, num_verts, sizeof(uint32_t));
    visited = (uint8_t *)prc_calloc(ctx, num_edges, sizeof(uint8_t));
    if (adj0 == NULL || adj1 == NULL || visited == NULL)
    {
        prc_free(ctx, edges);
        prc_free(ctx, adj0);
        prc_free(ctx, adj1);
        prc_free(ctx, visited);
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate adjacency in prc_trace_kept_region_boundaries\n");
        return PRC_ERROR_MEMORY;
    }
    for (k = 0; k < num_verts; k++)
    {
        adj0[k] = (uint32_t)-1;
        adj1[k] = (uint32_t)-1;
    }
    for (k = 0; k < num_edges; k++)
    {
        uint32_t a = edges[k].a, b = edges[k].b;

        if (adj0[a] == (uint32_t)-1)
            adj0[a] = k;
        else
            adj1[a] = k;
        if (adj0[b] == (uint32_t)-1)
            adj0[b] = k;
        else
            adj1[b] = k;
    }

    for (k = 0; k < num_edges; k++)
    {
        uint32_t start, cur_vert, cur_edge;
        prc_vec2 *path = NULL;
        uint32_t path_len = 0, path_cap = 0;

        if (visited[k])
            continue;

        start = edges[k].a;
        cur_vert = edges[k].a;
        cur_edge = k;

        for (;;)
        {
            uint32_t other = (edges[cur_edge].a == cur_vert) ? edges[cur_edge].b : edges[cur_edge].a;
            uint32_t e0, e1, next_edge;

            visited[cur_edge] = 1;

            if (path_len + 1 > path_cap)
            {
                uint32_t new_cap = path_cap ? path_cap * 2 : 16;
                prc_vec2 *new_path = (prc_vec2 *)prc_realloc(ctx, path, new_cap * sizeof(prc_vec2));

                if (new_path == NULL)
                {
                    prc_free(ctx, path);
                    prc_free(ctx, edges);
                    prc_free(ctx, adj0);
                    prc_free(ctx, adj1);
                    prc_free(ctx, visited);
                    prc_error(ctx, PRC_ERROR_MEMORY, "Failed to grow traced path in prc_trace_kept_region_boundaries\n");
                    return PRC_ERROR_MEMORY;
                }
                path = new_path;
                path_cap = new_cap;
            }
            path[path_len++] = grid_uv[cur_vert];

            cur_vert = other;
            if (cur_vert == start)
                break;

            e0 = adj0[cur_vert];
            e1 = adj1[cur_vert];
            next_edge = (e0 != cur_edge && !visited[e0]) ? e0 :
                        (e1 != (uint32_t)-1 && e1 != cur_edge && !visited[e1]) ? e1 : (uint32_t)-1;
            if (next_edge == (uint32_t)-1)
                break;   /* not a clean manifold boundary; stop this loop early rather than spin forever */
            cur_edge = next_edge;
        }

        if (path_len >= 3)
        {
            if (num_loops + 1 > loops_cap)
            {
                uint32_t new_cap = loops_cap ? loops_cap * 2 : 4;
                prc_loop_samples *new_loops = (prc_loop_samples *)prc_realloc(ctx, loops, new_cap * sizeof(prc_loop_samples));

                if (new_loops == NULL)
                {
                    prc_free(ctx, path);
                    prc_free(ctx, edges);
                    prc_free(ctx, adj0);
                    prc_free(ctx, adj1);
                    prc_free(ctx, visited);
                    prc_free(ctx, loops);
                    prc_error(ctx, PRC_ERROR_MEMORY, "Failed to grow traced loops in prc_trace_kept_region_boundaries\n");
                    return PRC_ERROR_MEMORY;
                }
                loops = new_loops;
                loops_cap = new_cap;
            }

            memset(&loops[num_loops], 0, sizeof(prc_loop_samples));
            loops[num_loops].num_samples = path_len + 1;
            loops[num_loops].uv_samples = (prc_vec2 *)prc_calloc(ctx, path_len + 1, sizeof(prc_vec2));
            if (loops[num_loops].uv_samples == NULL)
            {
                prc_free(ctx, path);
                prc_free(ctx, edges);
                prc_free(ctx, adj0);
                prc_free(ctx, adj1);
                prc_free(ctx, visited);
                prc_free(ctx, loops);
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate traced loop uv in prc_trace_kept_region_boundaries\n");
                return PRC_ERROR_MEMORY;
            }
            memcpy(loops[num_loops].uv_samples, path, path_len * sizeof(prc_vec2));
            loops[num_loops].uv_samples[path_len] = path[0];
            num_loops++;
        }

        prc_free(ctx, path);
    }

    prc_free(ctx, edges);
    prc_free(ctx, adj0);
    prc_free(ctx, adj1);
    prc_free(ctx, visited);

    *out_loops = loops;
    *out_num_loops = num_loops;

    return 0;
}

static void
prc_free_loop_array(prc_context *ctx, prc_loop_samples *loops, uint32_t count)
{
    uint32_t k;

    if (loops == NULL)
        return;
    for (k = 0; k < count; k++)
        prc_free(ctx, loops[k].uv_samples);
    prc_free(ctx, loops);
}

/* Assembles the outer boundary for a face whose loop(s) wind around a
   periodic uv axis (wrap_axis: 0 = u, 1 = v) into one flat, ordinary uv
   polygon by cutting a synthetic seam, so it can be handed to the same
   ear-clip machinery used for planar outer loops. Handles the two shapes
   this can take:
     - a single wrapping loop, closed off against the surface's own natural
       domain edge on the other axis (e.g. a cylinder trimmed at one rim
       only, still bounded by the surface's actual v extent on the other
       side)
     - two loops winding oppositely around the same axis, bridged directly
       to each other with no natural domain edge involved (a band trimmed
       by two rims, e.g. the middle section of a cylinder)
   The seam itself becomes a real (if thin) edge of the output mesh, the
   same trade-off the existing hole-bridging code already makes. Caller owns
   and must free outer_loop_out->uv_samples */
static int
prc_build_periodic_outer_loop(prc_context *ctx, uint32_t num_loops, prc_loop_samples *loop_samples,
    uint8_t wrap_axis, const prc_surface_sampling_info *sampling_info,
    prc_loop_samples *outer_loop_out)
{
    uint32_t k;
    uint32_t wrap_indices[2];
    uint32_t num_wrap = 0;
    uint32_t count_a;
    prc_vec2 *combined = NULL;
    uint32_t combined_count = 0;

    for (k = 0; k < num_loops; k++)
    {
        int32_t wind = wrap_axis ? loop_samples[k].wind_v : loop_samples[k].wind_u;

        if (wind != 0)
        {
            if (num_wrap >= 2)
            {
                prc_error(ctx, PRC_ERROR_INTERNAL, "Too many wrapping loops in prc_build_periodic_outer_loop\n");
                return PRC_ERROR_INTERNAL;
            }
            wrap_indices[num_wrap++] = k;
        }
    }

    if (num_wrap == 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "No wrapping loop in prc_build_periodic_outer_loop\n");
        return PRC_ERROR_INTERNAL;
    }

    count_a = (loop_samples[wrap_indices[0]].num_samples > 0) ? (loop_samples[wrap_indices[0]].num_samples - 1) : 0;
    if (count_a < 2)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Degenerate wrapping loop in prc_build_periodic_outer_loop\n");
        return PRC_ERROR_INTERNAL;
    }

    if (num_wrap == 2)
    {
        uint32_t idx_b = wrap_indices[1];
        uint32_t count_b = (loop_samples[idx_b].num_samples > 0) ? (loop_samples[idx_b].num_samples - 1) : 0;
        double a_end_a, a_start_b, shift;
        uint32_t j;

        if (count_b < 2)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Degenerate wrapping loop in prc_build_periodic_outer_loop\n");
            return PRC_ERROR_INTERNAL;
        }

        /* Align loop B into the same unwrapped coordinate frame as loop A
           (each was unwrapped independently, starting from its own first
           sample) so the two meet up at a short seam instead of at
           arbitrary, independently-chosen angle offsets */
        a_end_a = wrap_axis ? loop_samples[wrap_indices[0]].uv_samples[count_a].y
                             : loop_samples[wrap_indices[0]].uv_samples[count_a].x;
        a_start_b = wrap_axis ? loop_samples[idx_b].uv_samples[0].y : loop_samples[idx_b].uv_samples[0].x;
        shift = a_end_a - a_start_b;

        combined_count = count_a + count_b;
        combined = (prc_vec2 *)prc_calloc(ctx, combined_count, sizeof(prc_vec2));
        if (combined == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate combined seam ring in prc_build_periodic_outer_loop\n");
            return PRC_ERROR_MEMORY;
        }

        for (j = 0; j < count_a; j++)
            combined[j] = loop_samples[wrap_indices[0]].uv_samples[j];
        for (j = 0; j < count_b; j++)
        {
            prc_vec2 p = loop_samples[idx_b].uv_samples[j];

            if (wrap_axis)
                p.y += shift;
            else
                p.x += shift;
            combined[count_a + j] = p;
        }
    }
    else /* num_wrap == 1: close against the surface's natural domain edge */
    {
        int32_t wind = wrap_axis ? loop_samples[wrap_indices[0]].wind_v : loop_samples[wrap_indices[0]].wind_u;
        double edge_b, a_start, a_end;
        uint32_t j;
        uint8_t have_point_loop_edge = 0;
        double point_loop_edge = 0.0;

        /* A loop degenerated to a single point (e.g. a cone's apex) tells us
           definitively which domain edge the trimmed region closes against
           -- use it directly rather than the orientation-dependent guess
           below, which relies on loop sample direction matching the
           standard B-rep convention; prc_sample_coedge does not yet
           guarantee that (ignores coedge_orientation) */
        for (j = 0; j < num_loops; j++)
        {
            if (loop_samples[j].is_point_loop)
            {
                point_loop_edge = wrap_axis ? loop_samples[j].uv_samples[0].x : loop_samples[j].uv_samples[0].y;
                have_point_loop_edge = 1;
                break;
            }
        }

        /* Interior-on-left convention (walking the loop in its sampled
           direction, material is on the left in the uv plane): travelling
           in the increasing direction on the wrap axis puts the interior on
           the side of the larger-valued domain edge on the other axis, and
           vice versa. This relies on loop sample direction matching the
           standard B-rep convention, which prc_sample_loop does not yet
           fully guarantee (prc_sample_coedge ignores coedge_orientation) --
           flip this rule if a real file comes out trimmed to the wrong side */
        edge_b = have_point_loop_edge ? point_loop_edge :
                 (wind > 0) ? (wrap_axis ? sampling_info->end_u : sampling_info->end_v)
                             : (wrap_axis ? sampling_info->start_u : sampling_info->start_v);

        a_start = wrap_axis ? loop_samples[wrap_indices[0]].uv_samples[0].y : loop_samples[wrap_indices[0]].uv_samples[0].x;
        a_end = wrap_axis ? loop_samples[wrap_indices[0]].uv_samples[count_a].y : loop_samples[wrap_indices[0]].uv_samples[count_a].x;

        combined_count = count_a + 2;
        combined = (prc_vec2 *)prc_calloc(ctx, combined_count, sizeof(prc_vec2));
        if (combined == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate combined seam ring in prc_build_periodic_outer_loop\n");
            return PRC_ERROR_MEMORY;
        }

        for (j = 0; j < count_a; j++)
            combined[j] = loop_samples[wrap_indices[0]].uv_samples[j];

        /* The two synthetic corners closing the ring against the domain edge */
        if (wrap_axis)
        {
            combined[count_a].x = edge_b;
            combined[count_a].y = a_end;
            combined[count_a + 1].x = edge_b;
            combined[count_a + 1].y = a_start;
        }
        else
        {
            combined[count_a].x = a_end;
            combined[count_a].y = edge_b;
            combined[count_a + 1].x = a_start;
            combined[count_a + 1].y = edge_b;
        }
    }

    memset(outer_loop_out, 0, sizeof(*outer_loop_out));
    outer_loop_out->num_samples = combined_count + 1;   /* +1 re-duplicates the closing point, matching prc_sample_loop's convention */
    outer_loop_out->uv_samples = (prc_vec2 *)prc_calloc(ctx, outer_loop_out->num_samples, sizeof(prc_vec2));
    if (outer_loop_out->uv_samples == NULL)
    {
        prc_free(ctx, combined);
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate outer loop uv_samples in prc_build_periodic_outer_loop\n");
        return PRC_ERROR_MEMORY;
    }
    memcpy(outer_loop_out->uv_samples, combined, combined_count * sizeof(prc_vec2));
    outer_loop_out->uv_samples[combined_count] = combined[0];
    outer_loop_out->is_outer_loop = 1;

    prc_free(ctx, combined);

    return 0;
}

/* A cone's radius is affine in v (radius = base_radius + v*tan(semi_angle)),
   so for fixed u, moving along v traces a straight 3D line (a ruling) all
   the way to the apex -- a straight edge from the apex to any boundary
   sample therefore lies exactly on the surface, with no curvature error.
   When a cone face's only loops are the apex (a single-point loop) and one
   loop winding once around the full circumference, that means a plain fan
   of triangles from the apex to each boundary sample is an exact
   tessellation of the whole face, simpler and more faithful than running it
   through the general grid+clip machinery (which has to treat the apex as
   just another regular-grid row, producing a dense cluster of nearly
   degenerate triangles there).
   The apex vertex is duplicated once per triangle: a true cone's normal
   depends only on u, not v (so it's constant along a whole ruling, and
   evaluating it anywhere on the ruling other than exactly at the apex -
   where the radius is zero and the finite-difference tangent degenerates -
   gives the exact value for that ruling, including at the apex), and each
   fan triangle spans a different u, so sharing one apex vertex/normal
   across all of them would be wrong. Winding/normal convention mirrors
   prc_tessellate_planar_face_from_loops (orientation 1 keeps the loop's own
   sample order, 0 swaps the last two indices) */
static int
prc_tessellate_cone_apex_fan_face(prc_context *ctx, prc_data *data, uint32_t shell_index,
    uint32_t face_index, uint32_t geom_count, uint8_t orientation,
    surface_func surface_eval_func, prc_surface_params *surf_params,
    const prc_surface_sampling_info *sampling_info,
    const prc_loop_samples *point_loop, const prc_loop_samples *wrap_loop)
{
    uint32_t count = (wrap_loop->num_samples > 0) ? (wrap_loop->num_samples - 1) : 0;
    prc_exact_geom_tess_data *tess_data;
    prc_vec3 apex_position;
    uint32_t i;
    int code;

    if (count < 2)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Degenerate wrapping loop in prc_tessellate_cone_apex_fan_face\n");
        return PRC_ERROR_INTERNAL;
    }

    data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data =
        (prc_exact_geom_tess_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_tess_data));
    tess_data = data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data;
    if (tess_data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data in prc_tessellate_cone_apex_fan_face\n");
        return PRC_ERROR_MEMORY;
    }

    tess_data->number_of_vertices = count * 2;   /* shared boundary ring + one apex copy per triangle */
    tess_data->vertices = (prc_exact_geom_vertex *)prc_calloc(ctx, tess_data->number_of_vertices, sizeof(prc_exact_geom_vertex));
    tess_data->number_of_triangles = count;
    tess_data->triangles = (uint32_t *)prc_calloc(ctx, count * 3, sizeof(uint32_t));
    if (tess_data->vertices == NULL || tess_data->triangles == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure in prc_tessellate_cone_apex_fan_face\n");
        return PRC_ERROR_MEMORY;
    }

    apex_position = surface_eval_func(ctx, surf_params, point_loop->uv_samples[0].x, point_loop->uv_samples[0].y);

    for (i = 0; i < count; i++)
    {
        prc_vec3 position = surface_eval_func(ctx, surf_params, wrap_loop->uv_samples[i].x, wrap_loop->uv_samples[i].y);
        prc_vec3 normal;
        uint32_t apex_idx = count + i;
        uint32_t i_next = (i + 1 == count) ? 0 : (i + 1);

        code = prc_compute_loop_vertex_normal(ctx, surface_eval_func, surf_params,
            wrap_loop->uv_samples[i].x, wrap_loop->uv_samples[i].y,
            sampling_info->precision_u, sampling_info->precision_v,
            sampling_info, orientation, &normal);
        if (code < 0)
            return code;

        tess_data->vertices[i].position[0] = (float)position.x;
        tess_data->vertices[i].position[1] = (float)position.y;
        tess_data->vertices[i].position[2] = (float)position.z;
        tess_data->vertices[i].normal[0] = (float)normal.x;
        tess_data->vertices[i].normal[1] = (float)normal.y;
        tess_data->vertices[i].normal[2] = (float)normal.z;

        /* This ruling's normal (computed above, away from the degenerate
           apex) is exact for the apex copy too -- a cone's normal is
           constant along the whole ruling from base to apex */
        tess_data->vertices[apex_idx].position[0] = (float)apex_position.x;
        tess_data->vertices[apex_idx].position[1] = (float)apex_position.y;
        tess_data->vertices[apex_idx].position[2] = (float)apex_position.z;
        tess_data->vertices[apex_idx].normal[0] = (float)normal.x;
        tess_data->vertices[apex_idx].normal[1] = (float)normal.y;
        tess_data->vertices[apex_idx].normal[2] = (float)normal.z;

        if (orientation == 0)
        {
            tess_data->triangles[i * 3 + 0] = apex_idx;
            tess_data->triangles[i * 3 + 1] = i_next;
            tess_data->triangles[i * 3 + 2] = i;
        }
        else
        {
            tess_data->triangles[i * 3 + 0] = apex_idx;
            tess_data->triangles[i * 3 + 1] = i;
            tess_data->triangles[i * 3 + 2] = i_next;
        }
    }

    return 0;
}

/* Recursively splits a uv-space triangle (centroid fan-split, same pattern
   as prc_subdivide_curved_triangle) until each leaf is classified entirely
   inside or entirely outside the trimmed region (prc_point_inside_trimmed_
   region on its 3 corners), discarding outside leaves. Used instead of
   ear-clip-based boundary conformance for trim loops whose uv mapping is
   numerically projected rather than closed-form (NURBS/Blend02): a sharp
   corner or slightly noisy projected loop point there can make the ear-clip
   bridge/triangulate degenerate into long spoke/fan triangles, which this
   sidesteps entirely by never connecting two loop boundary points with a
   chord -- at the cost of a jagged (but, at max depth, far finer than one
   grid cell) boundary instead of an exact one */
static int
prc_subdivide_trim_boundary_triangle(prc_context *ctx, uint32_t num_loops, const prc_loop_samples *loop_samples,
    prc_vec2 uv0, prc_vec2 uv1, prc_vec2 uv2, uint32_t depth, uint32_t max_depth,
    prc_vec2 **out_verts, uint32_t *out_num_verts, uint32_t *out_verts_cap,
    uint32_t **out_tris, uint32_t *out_num_triangles, uint32_t *out_tris_cap)
{
    int in0, in1, in2, all_in, all_out;
    prc_vec2 uvc;
    int code;

    in0 = prc_point_inside_trimmed_region(num_loops, loop_samples, uv0);
    in1 = prc_point_inside_trimmed_region(num_loops, loop_samples, uv1);
    in2 = prc_point_inside_trimmed_region(num_loops, loop_samples, uv2);
    all_in = in0 && in1 && in2;
    all_out = !in0 && !in1 && !in2;

    if (depth >= max_depth || all_in)
    {
        if (!all_in)
        {
            /* Mixed or fully-outside corners at max depth: fall back to one
               centroid sample to decide whether this (by now tiny) leaf
               belongs to the trimmed region at all */
            uvc.x = (uv0.x + uv1.x + uv2.x) / 3.0;
            uvc.y = (uv0.y + uv1.y + uv2.y) / 3.0;
            if (!prc_point_inside_trimmed_region(num_loops, loop_samples, uvc))
                return 0;
        }

        if (*out_num_verts + 3 > *out_verts_cap)
        {
            uint32_t new_cap = (*out_verts_cap == 0) ? 64 : (*out_verts_cap * 2);
            prc_vec2 *new_verts;

            while (new_cap < *out_num_verts + 3)
                new_cap *= 2;
            new_verts = (prc_vec2 *)prc_realloc(ctx, *out_verts, new_cap * sizeof(prc_vec2));
            if (new_verts == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to grow vertices in prc_subdivide_trim_boundary_triangle\n");
                return PRC_ERROR_MEMORY;
            }
            *out_verts = new_verts;
            *out_verts_cap = new_cap;
        }
        if (*out_num_triangles + 1 > *out_tris_cap)
        {
            uint32_t new_cap = (*out_tris_cap == 0) ? 32 : (*out_tris_cap * 2);
            uint32_t *new_tris = (uint32_t *)prc_realloc(ctx, *out_tris, new_cap * 3 * sizeof(uint32_t));

            if (new_tris == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to grow triangles in prc_subdivide_trim_boundary_triangle\n");
                return PRC_ERROR_MEMORY;
            }
            *out_tris = new_tris;
            *out_tris_cap = new_cap;
        }

        (*out_verts)[*out_num_verts + 0] = uv0;
        (*out_verts)[*out_num_verts + 1] = uv1;
        (*out_verts)[*out_num_verts + 2] = uv2;
        (*out_tris)[*out_num_triangles * 3 + 0] = *out_num_verts + 0;
        (*out_tris)[*out_num_triangles * 3 + 1] = *out_num_verts + 1;
        (*out_tris)[*out_num_triangles * 3 + 2] = *out_num_verts + 2;
        *out_num_verts += 3;
        *out_num_triangles += 1;
        return 0;
    }

    if (all_out)
    {
        /* Cheap safety net for a trim feature thinner than this triangle:
           only bother subdividing further if the centroid disagrees with
           the 3 corners */
        uvc.x = (uv0.x + uv1.x + uv2.x) / 3.0;
        uvc.y = (uv0.y + uv1.y + uv2.y) / 3.0;
        if (!prc_point_inside_trimmed_region(num_loops, loop_samples, uvc))
            return 0;
    }
    else
    {
        uvc.x = (uv0.x + uv1.x + uv2.x) / 3.0;
        uvc.y = (uv0.y + uv1.y + uv2.y) / 3.0;
    }

    code = prc_subdivide_trim_boundary_triangle(ctx, num_loops, loop_samples, uv0, uv1, uvc, depth + 1, max_depth,
        out_verts, out_num_verts, out_verts_cap, out_tris, out_num_triangles, out_tris_cap);
    if (code < 0)
        return code;
    code = prc_subdivide_trim_boundary_triangle(ctx, num_loops, loop_samples, uv1, uv2, uvc, depth + 1, max_depth,
        out_verts, out_num_verts, out_verts_cap, out_tris, out_num_triangles, out_tris_cap);
    if (code < 0)
        return code;
    code = prc_subdivide_trim_boundary_triangle(ctx, num_loops, loop_samples, uv2, uv0, uvc, depth + 1, max_depth,
        out_verts, out_num_verts, out_verts_cap, out_tris, out_num_triangles, out_tris_cap);
    if (code < 0)
        return code;

    return 0;
}

/* Tessellates a curved (Cone/Cylinder/Sphere/Torus) face bounded by loop(s),
   whether a genuine simple closed trim polygon or a loop that winds around a
   periodic uv axis (degenerating to a line rather than an enclosed area in
   uv space, e.g. a circle around a cylinder's circumference). Rather than
   triangulating just the loop boundary (which can leave a few large
   ear-clip triangles cutting straight across a curved shape), this
   tessellates the FULL regular surface grid as usual, keeps whichever grid
   cells fall entirely inside the trimmed region, and ear-clip-fills only the
   thin fringe strip between the kept grid and the true trim boundary. The
   same building blocks (prc_build_regular_grid + prc_point_inside_trimmed_
   region) are general enough to later "cut" an already-tessellated surface,
   not just build a trimmed one from scratch.
   use_subdivision_fringe switches the fringe strip itself from that
   ear-clip method to prc_subdivide_trim_boundary_triangle -- used for
   surface types (NURBS/Blend02) whose loop uv mapping is numerically
   projected rather than closed-form, where ear-clip's boundary-conforming
   chords are prone to fan/spike artifacts at sharp corners or noisy
   projected points; Cone/Cylinder/Sphere/Torus keep the original ear-clip
   fringe unchanged */
static int
prc_tessellate_trimmed_face(prc_context *ctx, prc_data *data, uint32_t shell_index,
    uint32_t face_index, uint32_t geom_count, uint8_t orientation,
    surface_func surface_eval_func, prc_surface_params *surf_params,
    const prc_surface_sampling_info *sampling_info,
    uint32_t num_loops, prc_loop_samples *loop_samples, uint8_t has_wrapping_loop, uint8_t wrap_axis,
    uint8_t use_subdivision_fringe)
{
    prc_regular_grid grid = { 0 };
    uint8_t *cell_inside = NULL;
    uint32_t cell_count_u, cell_count_v, i, j, k;
    uint32_t *kept_triangles = NULL;
    uint32_t num_kept_triangles = 0;
    prc_loop_samples *traced_loops = NULL;
    uint32_t num_traced = 0;
    prc_loop_samples true_outer_loop = { 0 };
    uint32_t true_outer_index = (uint32_t)-1;
    prc_loop_samples *combined_loops = NULL;
    uint32_t combined_num_loops;
    uint32_t idx;
    prc_vec2 *fringe_poly_verts = NULL;
    uint32_t fringe_poly_num_verts = 0;
    uint32_t *fringe_poly_triangles = NULL;
    uint32_t fringe_poly_num_triangles = 0;
    prc_vec2 *fringe_verts = NULL;
    uint32_t fringe_num_verts = 0, fringe_verts_cap = 0;
    uint32_t *fringe_triangles = NULL;
    uint32_t fringe_num_triangles = 0, fringe_tris_cap = 0;
    double tolerance;
    prc_exact_geom_tess_data *tess_data;
    uint32_t total_verts, total_triangles, grid_num_verts;
    uint32_t v, t;
    int code;

    code = prc_build_regular_grid(ctx, surface_eval_func, surf_params, sampling_info, orientation, &grid);
    if (code < 0)
        return code;
    grid_num_verts = grid.vertex_samples_u * grid.vertex_samples_v;

    cell_count_u = grid.wrap_u ? grid.vertex_samples_u : (grid.vertex_samples_u - 1);
    cell_count_v = grid.wrap_v ? grid.vertex_samples_v : (grid.vertex_samples_v - 1);

    {
        uint8_t *cell_inside_raw = (uint8_t *)prc_calloc(ctx, cell_count_u * cell_count_v, sizeof(uint8_t));

        cell_inside = (uint8_t *)prc_calloc(ctx, cell_count_u * cell_count_v, sizeof(uint8_t));
        kept_triangles = (uint32_t *)prc_calloc(ctx, cell_count_u * cell_count_v * 6, sizeof(uint32_t));
        if (cell_inside_raw == NULL || cell_inside == NULL || kept_triangles == NULL)
        {
            prc_free(ctx, grid.uv);
            prc_free(ctx, grid.verts);
            prc_free(ctx, cell_inside_raw);
            prc_free(ctx, cell_inside);
            prc_free(ctx, kept_triangles);
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure in prc_tessellate_trimmed_face\n");
            return PRC_ERROR_MEMORY;
        }

        for (j = 0; j < cell_count_v; j++)
        {
            for (i = 0; i < cell_count_u; i++)
            {
                uint32_t vnext_i = (i + 1 == grid.vertex_samples_u && grid.wrap_u) ? 0 : (i + 1);
                uint32_t vnext_j = (j + 1 == grid.vertex_samples_v && grid.wrap_v) ? 0 : (j + 1);
                uint32_t i00 = j * grid.vertex_samples_u + i;
                uint32_t i10 = j * grid.vertex_samples_u + vnext_i;
                uint32_t i11 = vnext_j * grid.vertex_samples_u + vnext_i;
                uint32_t i01 = vnext_j * grid.vertex_samples_u + i;

                if (prc_point_inside_trimmed_region(num_loops, loop_samples, grid.uv[i00]) &&
                    prc_point_inside_trimmed_region(num_loops, loop_samples, grid.uv[i10]) &&
                    prc_point_inside_trimmed_region(num_loops, loop_samples, grid.uv[i11]) &&
                    prc_point_inside_trimmed_region(num_loops, loop_samples, grid.uv[i01]))
                {
                    cell_inside_raw[j * cell_count_u + i] = 1;
                }
            }
        }

        /* Erode the raw classification by one cell in every direction (any
           cell touching a non-kept neighbor, or the true domain edge, is
           excluded) so the kept region's traced outline is always at least
           one cell away from both the true trim boundary and the surface's
           own natural domain edge. Without this, a face whose surface
           domain happens to already match its trim loop (common -- e.g. a
           1/8 torus authored with that as the surface's own domain box)
           leaves an almost-zero-width fringe: the traced outline and the
           true loop become nearly coincident, which is exactly the
           degenerate near-duplicate-collinear-point case that stalls
           ear-clipping. The subdivision fringe doesn't trace an outline or
           ear-clip at all, so it needs none of this -- it just subdivides
           every cell the raw classification didn't already accept whole */
        if (use_subdivision_fringe)
        {
            memcpy(cell_inside, cell_inside_raw, cell_count_u * cell_count_v * sizeof(uint8_t));
        }
        else
        {
            for (j = 0; j < cell_count_v; j++)
            {
                for (i = 0; i < cell_count_u; i++)
                {
                    if (!cell_inside_raw[j * cell_count_u + i])
                        continue;

                    {
                        int32_t ni_left = prc_wrap_cell_index((int32_t)i - 1, cell_count_u, grid.wrap_u);
                        int32_t ni_right = prc_wrap_cell_index((int32_t)i + 1, cell_count_u, grid.wrap_u);
                        int32_t nj_below = prc_wrap_cell_index((int32_t)j - 1, cell_count_v, grid.wrap_v);
                        int32_t nj_above = prc_wrap_cell_index((int32_t)j + 1, cell_count_v, grid.wrap_v);

                        if (ni_left >= 0 && ni_right >= 0 && nj_below >= 0 && nj_above >= 0 &&
                            cell_inside_raw[j * cell_count_u + (uint32_t)ni_left] &&
                            cell_inside_raw[j * cell_count_u + (uint32_t)ni_right] &&
                            cell_inside_raw[(uint32_t)nj_below * cell_count_u + i] &&
                            cell_inside_raw[(uint32_t)nj_above * cell_count_u + i])
                        {
                            cell_inside[j * cell_count_u + i] = 1;
                        }
                    }
                }
            }
        }
        prc_free(ctx, cell_inside_raw);
    }

    for (j = 0; j < cell_count_v; j++)
    {
        for (i = 0; i < cell_count_u; i++)
        {
            if (cell_inside[j * cell_count_u + i])
            {
                uint32_t vnext_i = (i + 1 == grid.vertex_samples_u && grid.wrap_u) ? 0 : (i + 1);
                uint32_t vnext_j = (j + 1 == grid.vertex_samples_v && grid.wrap_v) ? 0 : (j + 1);
                uint32_t i00 = j * grid.vertex_samples_u + i;
                uint32_t i10 = j * grid.vertex_samples_u + vnext_i;
                uint32_t i11 = vnext_j * grid.vertex_samples_u + vnext_i;
                uint32_t i01 = vnext_j * grid.vertex_samples_u + i;

                if (orientation == 0)
                {
                    kept_triangles[num_kept_triangles * 3 + 0] = i00;
                    kept_triangles[num_kept_triangles * 3 + 1] = i01;
                    kept_triangles[num_kept_triangles * 3 + 2] = i10;
                    num_kept_triangles++;
                    kept_triangles[num_kept_triangles * 3 + 0] = i10;
                    kept_triangles[num_kept_triangles * 3 + 1] = i01;
                    kept_triangles[num_kept_triangles * 3 + 2] = i11;
                    num_kept_triangles++;
                }
                else
                {
                    kept_triangles[num_kept_triangles * 3 + 0] = i00;
                    kept_triangles[num_kept_triangles * 3 + 1] = i10;
                    kept_triangles[num_kept_triangles * 3 + 2] = i01;
                    num_kept_triangles++;
                    kept_triangles[num_kept_triangles * 3 + 0] = i10;
                    kept_triangles[num_kept_triangles * 3 + 1] = i11;
                    kept_triangles[num_kept_triangles * 3 + 2] = i01;
                    num_kept_triangles++;
                }
            }
        }
    }

    if (use_subdivision_fringe)
    {
        for (j = 0; j < cell_count_v; j++)
        {
            for (i = 0; i < cell_count_u; i++)
            {
                uint32_t vnext_i, vnext_j, i00, i10, i11, i01;

                if (cell_inside[j * cell_count_u + i])
                    continue;

                vnext_i = (i + 1 == grid.vertex_samples_u && grid.wrap_u) ? 0 : (i + 1);
                vnext_j = (j + 1 == grid.vertex_samples_v && grid.wrap_v) ? 0 : (j + 1);
                i00 = j * grid.vertex_samples_u + i;
                i10 = j * grid.vertex_samples_u + vnext_i;
                i11 = vnext_j * grid.vertex_samples_u + vnext_i;
                i01 = vnext_j * grid.vertex_samples_u + i;

                /* Fixed uv-space CCW winding regardless of orientation,
                   matching the ear-clip fringe's own convention below --
                   the shared triangle-output loop further down applies the
                   orientation flip once, the same way it already does for
                   that fringe */
                code = prc_subdivide_trim_boundary_triangle(ctx, num_loops, loop_samples,
                    grid.uv[i00], grid.uv[i10], grid.uv[i11], 0, PRC_TRIM_BOUNDARY_SUBDIVIDE_MAX_DEPTH,
                    &fringe_verts, &fringe_num_verts, &fringe_verts_cap,
                    &fringe_triangles, &fringe_num_triangles, &fringe_tris_cap);
                if (code >= 0)
                    code = prc_subdivide_trim_boundary_triangle(ctx, num_loops, loop_samples,
                        grid.uv[i00], grid.uv[i11], grid.uv[i01], 0, PRC_TRIM_BOUNDARY_SUBDIVIDE_MAX_DEPTH,
                        &fringe_verts, &fringe_num_verts, &fringe_verts_cap,
                        &fringe_triangles, &fringe_num_triangles, &fringe_tris_cap);
                if (code < 0)
                {
                    prc_free(ctx, grid.uv);
                    prc_free(ctx, grid.verts);
                    prc_free(ctx, cell_inside);
                    prc_free(ctx, kept_triangles);
                    prc_free(ctx, fringe_verts);
                    prc_free(ctx, fringe_triangles);
                    return code;
                }
            }
        }
        goto fringe_done;
    }

    code = prc_trace_kept_region_boundaries(ctx, cell_inside, grid.vertex_samples_u, grid.vertex_samples_v,
        grid.wrap_u, grid.wrap_v, grid.uv, &traced_loops, &num_traced);
    if (code < 0)
    {
        prc_free(ctx, grid.uv);
        prc_free(ctx, grid.verts);
        prc_free(ctx, cell_inside);
        prc_free(ctx, kept_triangles);
        return code;
    }

    /* True outer boundary for the fringe fill: the surface's own trim loop
       (already flagged is_outer_loop) for the closed case, or a synthetic
       seam-cut ring folding the wrapping loop(s) into an ordinary polygon */
    if (has_wrapping_loop)
    {
        code = prc_build_periodic_outer_loop(ctx, num_loops, loop_samples, wrap_axis, sampling_info, &true_outer_loop);
        if (code < 0)
        {
            prc_free(ctx, grid.uv);
            prc_free(ctx, grid.verts);
            prc_free(ctx, cell_inside);
            prc_free(ctx, kept_triangles);
            prc_free_loop_array(ctx, traced_loops, num_traced);
            return code;
        }
    }
    else
    {
        for (k = 0; k < num_loops; k++)
        {
            if (loop_samples[k].is_outer_loop)
            {
                true_outer_loop = loop_samples[k];
                true_outer_index = k;
                break;
            }
        }
        if (true_outer_index == (uint32_t)-1)
        {
            prc_free(ctx, grid.uv);
            prc_free(ctx, grid.verts);
            prc_free(ctx, cell_inside);
            prc_free(ctx, kept_triangles);
            prc_free_loop_array(ctx, traced_loops, num_traced);
            prc_error(ctx, PRC_ERROR_INTERNAL, "No outer loop found in prc_tessellate_trimmed_face\n");
            return PRC_ERROR_INTERNAL;
        }
    }

    /* Assemble: [true outer ring] + [traced kept-grid outline(s), holes] +
       [any additional non-wrapping loop not already the outer boundary,
       also a hole -- e.g. a drilled hole inside the trimmed region] */
    combined_num_loops = 1 + num_traced;
    for (k = 0; k < num_loops; k++)
    {
        if (loop_samples[k].wind_u != 0 || loop_samples[k].wind_v != 0 || k == true_outer_index ||
            loop_samples[k].is_point_loop)
            continue;
        combined_num_loops++;
    }

    combined_loops = (prc_loop_samples *)prc_calloc(ctx, combined_num_loops, sizeof(prc_loop_samples));
    if (combined_loops == NULL)
    {
        prc_free(ctx, grid.uv);
        prc_free(ctx, grid.verts);
        prc_free(ctx, cell_inside);
        prc_free(ctx, kept_triangles);
        prc_free_loop_array(ctx, traced_loops, num_traced);
        if (has_wrapping_loop)
            prc_free(ctx, true_outer_loop.uv_samples);
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate combined loops in prc_tessellate_trimmed_face\n");
        return PRC_ERROR_MEMORY;
    }

    combined_loops[0] = true_outer_loop;
    combined_loops[0].is_outer_loop = 1;
    idx = 1;
    for (k = 0; k < num_traced; k++)
    {
        combined_loops[idx] = traced_loops[k];
        combined_loops[idx].is_outer_loop = 0;
        idx++;
    }
    for (k = 0; k < num_loops; k++)
    {
        if (loop_samples[k].wind_u != 0 || loop_samples[k].wind_v != 0 || k == true_outer_index ||
            loop_samples[k].is_point_loop)
            continue;
        combined_loops[idx] = loop_samples[k];
        combined_loops[idx].is_outer_loop = 0;
        idx++;
    }

    code = prc_triangulate_planar_loops(ctx, combined_num_loops, combined_loops,
        &fringe_poly_verts, &fringe_poly_num_verts, &fringe_poly_triangles, &fringe_poly_num_triangles);
    prc_free(ctx, combined_loops);
    prc_free_loop_array(ctx, traced_loops, num_traced);
    if (has_wrapping_loop)
        prc_free(ctx, true_outer_loop.uv_samples);
    if (code < 0)
    {
        prc_free(ctx, grid.uv);
        prc_free(ctx, grid.verts);
        prc_free(ctx, cell_inside);
        prc_free(ctx, kept_triangles);
        prc_error(ctx, code, "Failed in prc_triangulate_planar_loops\n");
        return code;
    }

    tolerance = (sampling_info->precision_u > sampling_info->precision_v) ?
        sampling_info->precision_u : sampling_info->precision_v;

    for (t = 0; t < fringe_poly_num_triangles; t++)
    {
        uint32_t i0 = fringe_poly_triangles[t * 3 + 0];
        uint32_t i1 = fringe_poly_triangles[t * 3 + 1];
        uint32_t i2 = fringe_poly_triangles[t * 3 + 2];

        code = prc_subdivide_curved_triangle(ctx, surface_eval_func, surf_params,
            fringe_poly_verts[i0], fringe_poly_verts[i1], fringe_poly_verts[i2], tolerance, 0,
            PRC_CURVED_LOOP_SUBDIVIDE_MAX_DEPTH,
            &fringe_verts, &fringe_num_verts, &fringe_verts_cap,
            &fringe_triangles, &fringe_num_triangles, &fringe_tris_cap);
        if (code < 0)
        {
            prc_free(ctx, grid.uv);
            prc_free(ctx, grid.verts);
            prc_free(ctx, cell_inside);
            prc_free(ctx, kept_triangles);
            prc_free(ctx, fringe_poly_verts);
            prc_free(ctx, fringe_poly_triangles);
            prc_free(ctx, fringe_verts);
            prc_free(ctx, fringe_triangles);
            return code;
        }
    }
    prc_free(ctx, fringe_poly_verts);
    prc_free(ctx, fringe_poly_triangles);

fringe_done:
    prc_free(ctx, cell_inside);

    total_verts = grid_num_verts + fringe_num_verts;
    total_triangles = num_kept_triangles + fringe_num_triangles;

    data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data =
        (prc_exact_geom_tess_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_tess_data));
    if (data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data == NULL)
    {
        prc_free(ctx, grid.uv);
        prc_free(ctx, grid.verts);
        prc_free(ctx, kept_triangles);
        prc_free(ctx, fringe_verts);
        prc_free(ctx, fringe_triangles);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data in prc_tessellate_trimmed_face\n");
        return PRC_ERROR_MEMORY;
    }
    tess_data = data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data;

    tess_data->number_of_vertices = total_verts;
    tess_data->vertices = (prc_exact_geom_vertex *)prc_calloc(ctx, total_verts, sizeof(prc_exact_geom_vertex));
    if (tess_data->vertices == NULL)
    {
        prc_free(ctx, grid.uv);
        prc_free(ctx, grid.verts);
        prc_free(ctx, kept_triangles);
        prc_free(ctx, fringe_verts);
        prc_free(ctx, fringe_triangles);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data vertices in prc_tessellate_trimmed_face\n");
        return PRC_ERROR_MEMORY;
    }
    memcpy(tess_data->vertices, grid.verts, grid_num_verts * sizeof(prc_exact_geom_vertex));
    prc_free(ctx, grid.verts);

    for (v = 0; v < fringe_num_verts; v++)
    {
        prc_vec3 position = surface_eval_func(ctx, surf_params, fringe_verts[v].x, fringe_verts[v].y);
        prc_vec3 normal;

        code = prc_compute_loop_vertex_normal(ctx, surface_eval_func, surf_params,
            fringe_verts[v].x, fringe_verts[v].y,
            sampling_info->precision_u, sampling_info->precision_v,
            sampling_info, orientation, &normal);
        if (code < 0)
        {
            prc_free(ctx, grid.uv);
            prc_free(ctx, kept_triangles);
            prc_free(ctx, fringe_verts);
            prc_free(ctx, fringe_triangles);
            return code;
        }

        idx = grid_num_verts + v;
        tess_data->vertices[idx].position[0] = (float)position.x;
        tess_data->vertices[idx].position[1] = (float)position.y;
        tess_data->vertices[idx].position[2] = (float)position.z;
        tess_data->vertices[idx].normal[0] = (float)normal.x;
        tess_data->vertices[idx].normal[1] = (float)normal.y;
        tess_data->vertices[idx].normal[2] = (float)normal.z;
    }
    prc_free(ctx, grid.uv);
    prc_free(ctx, fringe_verts);

    tess_data->number_of_triangles = total_triangles;
    tess_data->triangles = (uint32_t *)prc_calloc(ctx, total_triangles * 3, sizeof(uint32_t));
    if (tess_data->triangles == NULL)
    {
        prc_free(ctx, kept_triangles);
        prc_free(ctx, fringe_triangles);
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data triangles in prc_tessellate_trimmed_face\n");
        return PRC_ERROR_MEMORY;
    }
    memcpy(tess_data->triangles, kept_triangles, num_kept_triangles * 3 * sizeof(uint32_t));
    prc_free(ctx, kept_triangles);

    for (t = 0; t < fringe_num_triangles; t++)
    {
        uint32_t i0 = fringe_triangles[t * 3 + 0] + grid_num_verts;
        uint32_t i1 = fringe_triangles[t * 3 + 1] + grid_num_verts;
        uint32_t i2 = fringe_triangles[t * 3 + 2] + grid_num_verts;
        uint32_t out_idx = (num_kept_triangles + t) * 3;

        if (orientation == 0)
        {
            tess_data->triangles[out_idx + 0] = i0;
            tess_data->triangles[out_idx + 1] = i2;
            tess_data->triangles[out_idx + 2] = i1;
        }
        else
        {
            tess_data->triangles[out_idx + 0] = i0;
            tess_data->triangles[out_idx + 1] = i1;
            tess_data->triangles[out_idx + 2] = i2;
        }
    }
    prc_free(ctx, fringe_triangles);

    return 0;
}

/* A cone's single trim loop can itself detour out to the apex and back
   (e.g. one loop built from a full circumference circle, a line down to
   the apex, a degenerate zero-radius "circle" sampled entirely at the
   apex, then a line back to the circle) instead of the apex being its own
   dedicated loop -- this samples as one combined loop with a long run of
   coincident apex points buried in the middle, which is_point_loop (a
   whole-loop check) does not catch. atan2 is exactly as meaningless there
   as for a whole-loop point loop, and unwrapping straight through that run
   corrupts the whole loop's wind_u. Detect any such embedded run by local
   (post-inverse-transform) radius and split it out into its own point
   loop, leaving the remaining samples as a clean circumference loop -- the
   same shape the dedicated-point-loop case already handles via
   prc_tessellate_cone_apex_fan_face. Reallocates *loop_samples_ptr and
   updates *num_loops_ptr in place only when a split actually happens. */
static int
prc_split_cone_apex_runs(prc_context *ctx, prc_type_surf *surface, uint32_t *num_loops_ptr, prc_loop_samples **loop_samples_ptr)
{
    uint32_t orig_num_loops = *num_loops_ptr;
    prc_loop_samples *loops = *loop_samples_ptr;
    prc_exact_geom_transform inverse_transform;
    uint32_t *run_start_arr = NULL, *run_end_arr = NULL;
    prc_vec3 *apex_point_arr = NULL;
    uint32_t num_splits = 0;
    uint32_t k;
    int code;

    if (surface->surface_type != PRC_TYPE_SURF_Cone || orig_num_loops == 0 || loops == NULL)
        return 0;

    code = prc_get_surface_transform_inverse(ctx, surface, &inverse_transform);
    if (code < 0)
        return code;

    run_start_arr = (uint32_t *)prc_calloc(ctx, orig_num_loops, sizeof(uint32_t));
    run_end_arr = (uint32_t *)prc_calloc(ctx, orig_num_loops, sizeof(uint32_t));
    apex_point_arr = (prc_vec3 *)prc_calloc(ctx, orig_num_loops, sizeof(prc_vec3));
    if (run_start_arr == NULL || run_end_arr == NULL || apex_point_arr == NULL)
    {
        prc_free(ctx, run_start_arr);
        prc_free(ctx, run_end_arr);
        prc_free(ctx, apex_point_arr);
        prc_error(ctx, PRC_ERROR_MEMORY, "Failed in allocation prc_split_cone_apex_runs\n");
        return PRC_ERROR_MEMORY;
    }

    for (k = 0; k < orig_num_loops; k++)
    {
        prc_loop_samples *loop = &loops[k];
        uint32_t n = loop->num_samples;
        uint32_t run_start = (uint32_t)-1, run_end = (uint32_t)-1;
        double max_radius = 0.0;
        uint32_t i;

        run_start_arr[k] = (uint32_t)-1;

        if (loop->is_point_loop || n < 6)
            continue;

        for (i = 0; i < n; i++)
        {
            prc_vec3 p = inverse_transform.is_identity ? loop->samples[i] :
                prc_exact_geom_apply_transform(ctx, &inverse_transform, loop->samples[i]);
            double radius = sqrt(p.x * p.x + p.y * p.y);

            if (radius > max_radius)
                max_radius = radius;
        }

        {
            double tol = fmax(max_radius * 1e-6, 1e-9);

            for (i = 0; i < n; i++)
            {
                prc_vec3 p = inverse_transform.is_identity ? loop->samples[i] :
                    prc_exact_geom_apply_transform(ctx, &inverse_transform, loop->samples[i]);
                double radius = sqrt(p.x * p.x + p.y * p.y);

                if (radius <= tol)
                {
                    if (run_start == (uint32_t)-1)
                        run_start = i;
                    run_end = i;
                }
                else if (run_start != (uint32_t)-1)
                {
                    break;   /* only ever expect one embedded apex run */
                }
            }
        }

        /* Require a genuine interior run: not the whole loop, and not
           touching either end (a run touching the start/end would mean the
           apex coincides with the loop's own closing point, a shape this
           scan is not built to split) */
        if (run_start == (uint32_t)-1 || run_start == 0 || run_end >= n - 1)
            continue;

        run_start_arr[k] = run_start;
        run_end_arr[k] = run_end;
        apex_point_arr[k] = loop->samples[run_start];
        num_splits++;
    }

    if (num_splits == 0)
    {
        prc_free(ctx, run_start_arr);
        prc_free(ctx, run_end_arr);
        prc_free(ctx, apex_point_arr);
        return 0;
    }

    {
        prc_loop_samples *new_loops = (prc_loop_samples *)prc_calloc(ctx, orig_num_loops + num_splits, sizeof(prc_loop_samples));
        uint32_t next_new_loop = orig_num_loops;

        if (new_loops == NULL)
        {
            prc_free(ctx, run_start_arr);
            prc_free(ctx, run_end_arr);
            prc_free(ctx, apex_point_arr);
            prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate split loops in prc_split_cone_apex_runs\n");
            return PRC_ERROR_MEMORY;
        }
        memcpy(new_loops, loops, orig_num_loops * sizeof(prc_loop_samples));
        prc_free(ctx, loops);

        for (k = 0; k < orig_num_loops; k++)
        {
            prc_loop_samples *loop;
            prc_loop_samples *point_loop;
            uint32_t run_start = run_start_arr[k], run_end = run_end_arr[k];
            uint32_t n, kept_count, pos, i;
            prc_vec3 *kept;

            if (run_start == (uint32_t)-1)
                continue;

            loop = &new_loops[k];
            n = loop->num_samples;
            kept_count = n - (run_end - run_start + 1);
            kept = (prc_vec3 *)prc_calloc(ctx, kept_count, sizeof(prc_vec3));
            if (kept == NULL)
            {
                prc_free(ctx, new_loops);
                prc_free(ctx, run_start_arr);
                prc_free(ctx, run_end_arr);
                prc_free(ctx, apex_point_arr);
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate reduced loop in prc_split_cone_apex_runs\n");
                return PRC_ERROR_MEMORY;
            }
            pos = 0;
            for (i = 0; i < run_start; i++)
                kept[pos++] = loop->samples[i];
            for (i = run_end + 1; i < n; i++)
                kept[pos++] = loop->samples[i];

            prc_free(ctx, loop->samples);
            loop->samples = kept;
            loop->num_samples = kept_count;

            point_loop = &new_loops[next_new_loop++];
            memset(point_loop, 0, sizeof(*point_loop));
            point_loop->num_samples = 2;
            point_loop->samples = (prc_vec3 *)prc_calloc(ctx, 2, sizeof(prc_vec3));
            if (point_loop->samples == NULL)
            {
                prc_free(ctx, new_loops);
                prc_free(ctx, run_start_arr);
                prc_free(ctx, run_end_arr);
                prc_free(ctx, apex_point_arr);
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed to allocate apex point loop in prc_split_cone_apex_runs\n");
                return PRC_ERROR_MEMORY;
            }
            point_loop->samples[0] = apex_point_arr[k];
            point_loop->samples[1] = apex_point_arr[k];
            point_loop->is_point_loop = 1;
        }

        *loop_samples_ptr = new_loops;
        *num_loops_ptr = orig_num_loops + num_splits;
    }

    prc_free(ctx, run_start_arr);
    prc_free(ctx, run_end_arr);
    prc_free(ctx, apex_point_arr);
    return 0;
}

static int
prc_tessellate_surface(prc_context *ctx, prc_data *data, uint32_t shell_index,
    uint32_t face_index, prc_topo_face *topo_face,
    uint8_t orientation, prc_nano_brep_ref_data *brep_ref_data,
    prc_topo_context *topo_context)
{
    int code;
    uint32_t k, j;
    prc_surface_params surf_params = { 0 };
    surface_func surface_eval_func = NULL;
    uint32_t geom_count = data->exact_geom_tess_part_count;
    prc_surface_sampling_info sampling_info = { 0 };
    double start_u = 0.0;
    double start_v = 0.0;
    double end_u = 0.0;
    double end_v = 0.0;
    prc_exact_geom_transform exact_geom_trans;
    prc_type_surf surface = topo_face->surface_geometry.surface;
    uint32_t num_loops = topo_face->number_of_loops;
    prc_ptr_topology *loops = topo_face->loops;
    prc_loop_samples *loop_samples = NULL;
    uint8_t topo_context_behavior = 0;

    if (topo_context != NULL)
    {
        topo_context_behavior = topo_context->behavior;
    }

    /* Orientation is either 0 (opposite direction), 1 (same direction), or 2 (unknown.
       If unknown it is needed to do geometric tests to determine the correct orientation.
       The normal should point outside the material of the shell if the shell is closed
       This also sets the transformation */
    code = prc_get_surface_data(ctx, &surface, &sampling_info);
    if (code < 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Failed to get surface data in prc_tessellate_surface\n");
        return code;
    }
    start_u = sampling_info.start_u;
    start_v = sampling_info.start_v;
    end_u = sampling_info.end_u;
    end_v = sampling_info.end_v;

    /* Lets get any loops that may be associated with the surface. Loops
       are curves (or vertices -- e.g. the tip of a cone) that make cuts
       on the parametric surface. I would expect to see them primarily used
       with planar surfaces to cut things like a washer for example. These have
       to be PRC_TYPE_TOPO_Loop a vertex type is supposed to be a line with
       the same starting and ending position */
       /* Lets to a sanity check that indeed they are PRC_TYPE_TOPO_Loop. We will
          not check the is stored values as we should have already tested those */
          /* The check for brep_ref_data not NULL is due to the fact that this
             method is sometimes called from the compressed surface code */
    if (brep_ref_data != NULL)
    {
        for (k = 0; k < num_loops; k++)
        {
            if (!loops[k].is_stored)
            {
                if (loops[k].topo->tag != PRC_TYPE_TOPO_Loop)
                {
                    prc_error(ctx, PRC_ERROR_INTERNAL, "Surface loop is of wrong type\n");
                    return PRC_ERROR_INTERNAL;
                }
            }
        }

        /* For each of these loops we will need to get a set of samples that
           are sufficient for us to approximate them before we can make use of them.
           Lets do that first and store the loops in a sampled form */
        if (num_loops > 0)
        {
            loop_samples = (prc_loop_samples *)prc_calloc(ctx, num_loops, sizeof(prc_loop_samples));
            if (loop_samples == NULL)
            {
                prc_error(ctx, PRC_ERROR_MEMORY, "Failed in allocation prc_tessellate_surface\n");
                return PRC_ERROR_MEMORY;
            }
            for (k = 0; k < num_loops; k++)
            {
                code = prc_sample_loop(ctx, brep_ref_data, topo_face, &loops[k], &loop_samples[k]);
                if (code < 0)
                {
                    for (j = 0; j <= k; j++)
                    {
                        if (loop_samples[j].samples != NULL)
                        {
                            prc_free(ctx, loop_samples[j].samples);
                        }
                    }
                    prc_free(ctx, loop_samples);
                    prc_error(ctx, PRC_ERROR_INTERNAL, "Failed in prc_sample_loop\n");
                    return PRC_ERROR_INTERNAL;
                }
            }

            /* A cone's single trim loop can detour out to the apex and back
               (full circumference, line to apex, degenerate point, line
               back) instead of the apex being its own loop -- split any such
               embedded run out into its own point loop before the uv
               mapping below, which would otherwise try to atan2 straight
               through it. No-op (and cheap) for every other case */
            code = prc_split_cone_apex_runs(ctx, &surface, &num_loops, &loop_samples);
            if (code < 0)
            {
                for (j = 0; j < num_loops; j++)
                {
                    if (loop_samples[j].samples != NULL)
                    {
                        prc_free(ctx, loop_samples[j].samples);
                    }
                }
                prc_free(ctx, loop_samples);
                prc_error(ctx, code, "Failed in prc_split_cone_apex_runs\n");
                return code;
            }

            /* Now lets get the loops onto the parametric surface so they can
               be used as a edge boundary in the tessellation process */
            code = prc_map_loops_to_surface(ctx, topo_face, orientation, &surface,
                num_loops, loop_samples);
            if (code < 0)
            {
                for (j = 0; j < num_loops; j++)
                {
                    if (loop_samples[j].samples != NULL)
                    {
                        prc_free(ctx, loop_samples[j].samples);
                    }
                    if (loop_samples[j].uv_samples != NULL)
                    {
                        prc_free(ctx, loop_samples[j].uv_samples);
                    }
                }
                prc_free(ctx, loop_samples);
                prc_error(ctx, code, "Failed in prc_map_loops_to_surface\n");
                return code;
            }

            /* Now lets see if we can figure out which of these is an outer
               and which is an inner loop */
            code = prc_assign_loop_inner_outer(ctx, topo_face, orientation,
                topo_context_behavior, num_loops, loop_samples, surface.surface_type);
            if (code < 0)
            {
                for (j = 0; j < num_loops; j++)
                {
                    if (loop_samples[j].samples != NULL)
                    {
                        prc_free(ctx, loop_samples[j].samples);
                    }
                    if (loop_samples[j].uv_samples != NULL)
                    {
                        prc_free(ctx, loop_samples[j].uv_samples);
                    }
                }
                prc_free(ctx, loop_samples);
                prc_error(ctx, code, "Failed in prc_assign_loop_inner_outer\n");
                return code;
            }
        }
        surf_params.loop_samples = loop_samples;
        surf_params.num_loops = num_loops;
    }

    switch (surface.surface_type)
    {
    case PRC_TYPE_SURF_FromCurves:
    {
        prc_surf_fromcurves *from_curves = surface.surf_fromcurves;
        prc_uv_parameterization params = from_curves->parameterization;

        surf_params.surface_params = (void *)from_curves;
        surface_eval_func = prc_evaluate_surf_fromcurves;
        break;
    }

    case PRC_TYPE_SURF_Cone:
    {
        prc_surf_cone *cone = surface.surf_cone;
        prc_uv_parameterization params = cone->parameterization;

        surf_params.surface_params = (void *)cone;
        surface_eval_func = prc_evaluate_surf_cone;
        break;
    }

    case PRC_TYPE_SURF_Cylinder:
    {
        prc_surf_cylinder *cylinder = surface.surf_cylinder;
        prc_uv_parameterization params = cylinder->parameterization;

        surf_params.surface_params = (void *)cylinder;
        surface_eval_func = prc_evaluate_surf_cylinder;
        break;
    }

    case PRC_TYPE_SURF_Sphere:
    {
        prc_surf_sphere *sphere = surface.surf_sphere;
        prc_uv_parameterization params = sphere->parameterization;

        surf_params.surface_params = (void *)sphere;
        surface_eval_func = prc_evaluate_surf_sphere;
        break;
    }

    case PRC_TYPE_SURF_Torus:
    {
        prc_surf_torus *torus = surface.surf_torus;
        prc_uv_parameterization params = torus->parameterization;

        surf_params.surface_params = (void *)torus;
        surface_eval_func = prc_evaluate_surf_torus;
        break;
    }

    case PRC_TYPE_SURF_Cylindrical:
    {
        prc_surf_cylindrical *cylindrical = surface.surf_cylindrical;
        prc_uv_parameterization params = cylindrical->parameterization;

        surf_params.surface_params = (void *)cylindrical;
        surface_eval_func = prc_evaluate_surf_cylindrical;
        break;
    }

    case PRC_TYPE_SURF_Extrusion:
    {
        prc_surf_extrusion *extrusion = surface.surf_extrusion;
        prc_uv_parameterization params = extrusion->parameterization;

        surf_params.surface_params = (void *)extrusion;
        surface_eval_func = prc_evaluate_surf_extrusion;
        break;
    }

    case PRC_TYPE_SURF_Revolution:
    {
        prc_surf_revolution *revolution = surface.surf_revolution;
        prc_uv_parameterization params = revolution->parameterization;

        surf_params.surface_params = (void *)revolution;
        surface_eval_func = prc_evaluate_surf_revolution;
        break;
    }

    case PRC_TYPE_SURF_Plane:
    {
        prc_surf_plane *plane = surface.surf_plane;
        prc_domain params = plane->domain;

        surf_params.surface_params = (void *)plane;
        surface_eval_func = prc_evaluate_surf_plane;
        break;
    }

    case PRC_TYPE_SURF_Offset:
    {
        prc_surf_offset *offset = surface.surf_offset;

        surf_params.surface_params = (void *)offset;
        surface_eval_func = prc_evaluate_surf_offset;
        break;
    }

    case PRC_TYPE_SURF_NURBS:
    {
        prc_surf_nurbs *nurbs = surface.surf_nurbs;

        surf_params.surface_params = (void *)nurbs;
        surface_eval_func = prc_evaluate_surf_nurbs;
        break;
    }

    case PRC_TYPE_SURF_Blend02:
    {
        prc_surf_blend02 *blend = surface.surf_blend02;

        surf_params.surface_params = (void *)blend;
        surface_eval_func = prc_evaluate_surf_blend02;
        break;
    }

    case PRC_TYPE_SURF_Blend01:
    {
        prc_surf_blend01 *blend = surface.surf_blend01;

        surf_params.surface_params = (void *)blend;
        surface_eval_func = prc_evaluate_surf_blend01;
        break;
    }

    default:
        for (j = 0; j < num_loops; j++)
        {
            if (loop_samples[j].samples != NULL)
            {
                prc_free(ctx, loop_samples[j].samples);
            }
            if (loop_samples[j].uv_samples != NULL)
            {
                prc_free(ctx, loop_samples[j].uv_samples);
            }
        }
        prc_free(ctx, loop_samples);
        data->exact_geom_tess_part[data->exact_geom_tess_part_count].shells[shell_index].faces[face_index].type = PRC_EXACT_GEOM_UNKNOWN;
        return 0;
    }

    if (surface_eval_func == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid surface evaluation function in prc_tessellate_surface\n");
        return PRC_ERROR_INTERNAL;
    }

    /* A planar face bounded by loops is tessellated straight from those loop
       boundaries (ear-clipped, holes bridged in) instead of the regular grid
       below: the plane's own schema domain need not match the face at all,
       only the loops define its real outer edge and holes */
    if (surface.surface_type == PRC_TYPE_SURF_Plane && num_loops > 0 && loop_samples != NULL)
    {
        code = prc_tessellate_planar_face_from_loops(ctx, data, shell_index, face_index,
            geom_count, orientation, surface_eval_func, &surf_params, &sampling_info,
            num_loops, loop_samples);
        for (j = 0; j < num_loops; j++)
        {
            if (loop_samples[j].samples != NULL)
            {
                prc_free(ctx, loop_samples[j].samples);
            }
            if (loop_samples[j].uv_samples != NULL)
            {
                prc_free(ctx, loop_samples[j].uv_samples);
            }
        }
        prc_free(ctx, loop_samples);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_tessellate_planar_face_from_loops\n");
        }
        return code;
    }

    /* A cone trimmed down to exactly its apex (a single-point loop) plus one
       loop winding once around the full circumference: an exact fan of
       rulings from the apex, see prc_tessellate_cone_apex_fan_face. Anything
       more complex (extra holes, more than one wrapping loop, etc.) falls
       through to the general grid+clip path below like before */
    if (surface.surface_type == PRC_TYPE_SURF_Cone && num_loops == 2 && loop_samples != NULL)
    {
        uint32_t point_idx = (uint32_t)-1, wrap_idx = (uint32_t)-1;

        for (k = 0; k < num_loops; k++)
        {
            if (loop_samples[k].is_point_loop)
                point_idx = k;
            else if (loop_samples[k].wind_u != 0)
                wrap_idx = k;
        }

        if (point_idx != (uint32_t)-1 && wrap_idx != (uint32_t)-1)
        {
            code = prc_tessellate_cone_apex_fan_face(ctx, data, shell_index, face_index,
                geom_count, orientation, surface_eval_func, &surf_params, &sampling_info,
                &loop_samples[point_idx], &loop_samples[wrap_idx]);
            for (j = 0; j < num_loops; j++)
            {
                if (loop_samples[j].samples != NULL)
                {
                    prc_free(ctx, loop_samples[j].samples);
                }
                if (loop_samples[j].uv_samples != NULL)
                {
                    prc_free(ctx, loop_samples[j].uv_samples);
                }
            }
            prc_free(ctx, loop_samples);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_tessellate_cone_apex_fan_face\n");
            }
            return code;
        }
    }

    /* Curved periodic surfaces (Cone/Cylinder/Sphere/Torus) bounded by loops:
       classify each loop by how many times it winds around a periodic uv
       axis before closing up (0 = a genuine simple polygon in uv space, e.g.
       a hole or a self-contained trim region; nonzero = the loop is itself a
       cut across the periodic direction, e.g. a circle around a cylinder's
       circumference, which degenerates to a line rather than an enclosed
       area in uv space). NURBS is included here too -- it is never periodic
       (see prc_get_surface_data), so every loop's wind_u/wind_v is always 0
       and it always takes the num_wrapping == 0 closed-loop branch below,
       the same grid+clip machinery already used for the non-wrapping
       Cone/Cylinder/Sphere/Torus case */
    if (num_loops > 0 && loop_samples != NULL &&
        (surface.surface_type == PRC_TYPE_SURF_Cone || surface.surface_type == PRC_TYPE_SURF_Cylinder ||
            surface.surface_type == PRC_TYPE_SURF_Sphere || surface.surface_type == PRC_TYPE_SURF_Torus ||
            surface.surface_type == PRC_TYPE_SURF_NURBS))
    {
        uint32_t num_wrapping = 0;
        uint8_t wrap_axis = 0;
        uint8_t mixed_axes = 0;
        uint8_t bad_winding = 0;

        for (k = 0; k < num_loops; k++)
        {
            int32_t wu = loop_samples[k].wind_u;
            int32_t wv = loop_samples[k].wind_v;

            if (wu != 0 && wv != 0)
            {
                mixed_axes = 1;
            }
            else if (wu != 0)
            {
                if (num_wrapping > 0 && wrap_axis != 0)
                    mixed_axes = 1;
                wrap_axis = 0;
                if (wu != 1 && wu != -1)
                    bad_winding = 1;
                num_wrapping++;
            }
            else if (wv != 0)
            {
                if (num_wrapping > 0 && wrap_axis != 1)
                    mixed_axes = 1;
                wrap_axis = 1;
                if (wv != 1 && wv != -1)
                    bad_winding = 1;
                num_wrapping++;
            }
        }

        if (num_wrapping == 0 || (!mixed_axes && !bad_winding && num_wrapping <= 2))
        {
            code = prc_tessellate_trimmed_face(ctx, data, shell_index, face_index,
                geom_count, orientation, surface_eval_func, &surf_params, &sampling_info,
                num_loops, loop_samples, num_wrapping > 0, wrap_axis,
                surface.surface_type == PRC_TYPE_SURF_NURBS);
            for (j = 0; j < num_loops; j++)
            {
                if (loop_samples[j].samples != NULL)
                {
                    prc_free(ctx, loop_samples[j].samples);
                }
                if (loop_samples[j].uv_samples != NULL)
                {
                    prc_free(ctx, loop_samples[j].uv_samples);
                }
            }
            prc_free(ctx, loop_samples);
            if (code < 0)
            {
                prc_error(ctx, code, "Failed in prc_tessellate_trimmed_face\n");
            }
            return code;
        }
        /* Anything else (wraps both axes, winds more than once, more than
           two wrapping loops) isn't handled yet -- fall through to the
           regular parametric grid below, same as before this loop support
           existed */
    }

    {
        prc_regular_grid grid = { 0 };
        prc_exact_geom_tess_data *tess_data;

        code = prc_build_regular_grid(ctx, surface_eval_func, &surf_params, &sampling_info, orientation, &grid);
        if (code < 0)
        {
            prc_error(ctx, code, "Failed in prc_tessellate_trimmed_face\n");
            for (j = 0; j < num_loops; j++)
            {
                if (loop_samples[j].samples != NULL)
                {
                    prc_free(ctx, loop_samples[j].samples);
                }
                if (loop_samples[j].uv_samples != NULL)
                {
                    prc_free(ctx, loop_samples[j].uv_samples);
                }
            }
            prc_free(ctx, loop_samples);
            return code;
        }

        /* Per-surface developer trace. Gated, and on stderr rather than stdout:
           it fired 7171 times over a 310-file corpus, and being on stdout it
           corrupts any consumer that emits machine-readable output there -- it
           was found by it landing in the middle of a generated CSV. The env
           lookup is cached so the hot path pays one getenv for the whole run,
           mirroring prc_debug_hooks_init in prc_decode_compressed_tess.c, and
           the whole hook compiles out when PRC_ENABLE_DIAG_ENV is OFF.
           Set PRC_DIAG_TESSELLATE_SURFACE=1 to get the old always-on output. */
        {
            static int surf_trace_read = 0;
            static int surf_trace_on = 0;

            if (!surf_trace_read)
            {
                const char *v = prc_diag_getenv("PRC_DIAG_TESSELLATE_SURFACE");
                surf_trace_read = 1;
                surf_trace_on = (v != NULL && v[0] != 0 && v[0] != '0');
            }

            if (surf_trace_on)
                fprintf(stderr, "prc_tessellate_surface: surface_type=%u orientation=%u wrap_u=%u wrap_v=%u vertex_samples_u=%u vertex_samples_v=%u start_u=%f end_u=%f start_v=%f end_v=%f\n",
                    topo_face->surface_geometry.surface.surface_type,
                    orientation,
                    grid.wrap_u,
                    grid.wrap_v,
                    grid.vertex_samples_u,
                    grid.vertex_samples_v,
                    start_u,
                    end_u,
                    start_v,
                    end_v);
        }

        data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data =
            (prc_exact_geom_tess_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_tess_data));
        if (data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data == NULL)
        {
            prc_free(ctx, grid.uv);
            prc_free(ctx, grid.verts);
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data in prc_tessellate_surface\n");
            return PRC_ERROR_MEMORY;
        }
        tess_data = data->exact_geom_tess_part[geom_count].shells[shell_index].faces[face_index].tess_data;

        tess_data->number_of_vertices = grid.vertex_samples_u * grid.vertex_samples_v;
        tess_data->vertices = grid.verts;
        prc_free(ctx, grid.uv);

        for (j = 0; j < num_loops; j++)
        {
            if (loop_samples[j].samples != NULL)
            {
                prc_free(ctx, loop_samples[j].samples);
            }
            if (loop_samples[j].uv_samples != NULL)
            {
                prc_free(ctx, loop_samples[j].uv_samples);
            }
        }
        prc_free(ctx, loop_samples);

        code = prc_emit_grid_all_triangles(ctx, &grid, orientation, &tess_data->triangles, &tess_data->number_of_triangles);
        if (code < 0)
            return code;
    }
    return 0;
}

static uint32_t
prc_count_faces_in_shell(prc_context *ctx, prc_topo_shell *shell)
{
    if (shell->tag == PRC_TYPE_TOPO_Shell)
    {
        return shell->number_of_faces;
    }
    return 0;
}

static void
prc_count_shells_faces_in_topo(prc_context *ctx, prc_topo *topo,
    uint32_t *num_shells, uint32_t *num_faces)
{
    *num_shells = 0;
    *num_faces = 0;

    if (topo->tag == PRC_TYPE_TOPO_BrepData)
    {
        prc_topo_brep_data *brep_data = topo->topo_brep_data;
        if (brep_data->number_of_connex > 0)
        {
            /* Do we need to worry about multiple connex here? */
            prc_topo_connex *connex = brep_data->connex[0].topo->topo_connex;
            *num_shells = connex->number_of_shells;
            for (uint32_t i = 0; i < connex->number_of_shells; i++)
            {
                prc_topo_shell *shell = connex->shells[i].topo->topo_shell;
                *num_faces += prc_count_faces_in_shell(ctx, shell);
            }
        }
    }
    else if (topo->tag == PRC_TYPE_TOPO_BrepDataCompress)
    {
        /* The number of faces in the compressed brep is calculated as the
           number of faces in all of the shells in all of the connex entities.*/
        prc_topo_brep_data_compress *brep_data_comp = topo->topo_brep_data_compress;

        /* We will handle just the single_connex compressed at this time. */
        if (!brep_data_comp->single_connex_test)
        {
            prc_error(ctx, PRC_ERROR_INTERNAL, "Multi-connex in compressed brep not yet supported");
            return;
        }
        /* In this case we just have a single shell and how every many faces that shell has */
        *num_shells = 1;
        *num_faces = brep_data_comp->number_of_faces;
    }
}

static uint32_t
prc_count_wires_in_topo(prc_context *ctx, prc_topo *topo)
{   
    if (topo->tag == PRC_TYPE_TOPO_SingleWireBody)
    {
        prc_topo_single_wire_body *body = topo->topo_single_wire_body;
        /* This one could be referenced... */
        if (body->wire_body.is_stored == 0)
        {
            if (body->wire_body.topo->tag == PRC_TYPE_TOPO_WireEdge)
            {
                return 1;
            }
        }
    }
    else if (topo->tag == PRC_TYPE_TOPO_SingleWireBodyCompress)
    {
        /* I *think* this is never referenced */
        return 1;
    }
    return 0;
}

int
prc_approximate_objects_exact_geom(prc_context *ctx, prc_api_data data_in, uint32_t *num_tessellations)
{
    prc_data *data = (prc_data *)data_in;
    uint32_t geom_count = data->exact_geom_tess_part_count;
    uint32_t file_index = data->exact_geom_tess_part[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess_part[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess_part[geom_count].body_index;
    int code;
    uint32_t num_shells, num_faces, num_wires;
    uint32_t i, j;

    /* Lets figure out if we are doing a curve or a triangle tessellation */
    prc_topo *topo = &data->file_struct[file_index].geometry->exact_geometry.topo_contexts[topo_index].bodies[body_index];

    /* Find out how many shells and how many faces we are dealing with here */
    prc_count_shells_faces_in_topo(ctx, topo, &num_shells, &num_faces);
    //num_faces = 3; /* For debug testing */
    num_wires = prc_count_wires_in_topo(ctx, topo);

    /* I *think* we do not have wires AND faces */
    if (num_wires > 0 && num_faces > 0)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Error: Found both wires and faces in the same topo. This is not supported.\n");
        return PRC_ERROR_INTERNAL;
    }

    /* If we have faces and shells allocate accordingly. If we have a wire also 
       allocate */
    if (num_wires == 1)
    {
        num_faces = 1;
        num_shells = 1;
    }

    /* Allocate shells and faces */
    data->exact_geom_tess_part[geom_count].shells = (prc_exact_geom_shell *)prc_calloc(ctx, num_shells, sizeof(prc_exact_geom_shell));
    if (data->exact_geom_tess_part[geom_count].shells == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of shells in prc_approximate_objects_exact_geom\n");
        return PRC_ERROR_MEMORY;
    }
    data->exact_geom_tess_part[geom_count].number_of_shells = num_shells;
    for (i = 0; i < num_shells; i++)
    {
        uint32_t num_faces_in_shell = 0;

        if (topo->tag == PRC_TYPE_TOPO_BrepData)
        {
            num_faces_in_shell = (num_wires == 1) ? 1 : prc_count_faces_in_shell(ctx, topo->topo_brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell);
        }
        else if (topo->tag == PRC_TYPE_TOPO_BrepDataCompress)
        {
            /* Note we only handle the single connex case for now.. */
            num_faces_in_shell = topo->topo_brep_data_compress->single_connex.number_of_faces;
        }
        else if (topo->tag == PRC_TYPE_TOPO_SingleWireBodyCompress || topo->tag == PRC_TYPE_TOPO_SingleWireBody)
        {
            num_faces_in_shell = 1;
        }
        data->exact_geom_tess_part[geom_count].shells[i].faces = (prc_exact_geom_face *)prc_calloc(ctx, num_faces, sizeof(prc_exact_geom_face));
        if (data->exact_geom_tess_part[geom_count].shells[i].faces == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of faces in prc_approximate_objects_exact_geom\n");
            return PRC_ERROR_MEMORY;
        }
        //num_faces_in_shell = num_faces; /* For debug testing */
        data->exact_geom_tess_part[geom_count].shells[i].number_of_faces = num_faces_in_shell;
    }

    /* Now loop on the shells and the faces */
    for (i = 0; i < num_shells; i++)
    {
        for (j = 0; j < data->exact_geom_tess_part[geom_count].shells[i].number_of_faces; j++)
        {
            switch (topo->tag)
            {
            case PRC_TYPE_TOPO_SingleWireBodyCompress:
            {
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_WIRE;
                 
                prc_topo_single_wire_compress *body = topo->topo_single_wire_compress;
                code = prc_sample_compressed_curve(ctx, data, i, j, &body->compressed_curve, body->curve_tolerance);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_sample_compressed_curve\n");
                    return code;
                }
                (*num_tessellations)++;
                break;
            }
            case PRC_TYPE_TOPO_SingleWireBody:
            {
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_WIRE;

                prc_topo_single_wire_body *body = topo->topo_single_wire_body;
                if (body->wire_body.is_stored == 1)
                {
                    /* We have to find this one. For now we skip this case */
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }

                /* Get the type of topo that this is. For now we just do wire edge.. */
                switch (body->wire_body.topo->tag)
                {
                case PRC_TYPE_TOPO_WireEdge:
                {
                    prc_topo_wire_edge *wire_edge = body->wire_body.topo->topo_wire_edge;
                    prc_exact_geom_wire_data *wire_data =
                        (prc_exact_geom_wire_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_wire_data));
                    if (data->exact_geom_tess_part[geom_count].shells[i].faces[j].wire_data == NULL)
                    {
                        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of wire_data\n");
                        return PRC_ERROR_MEMORY;
                    }
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].wire_data = wire_data;
                    code = prc_sample_curve(ctx, &wire_edge->curve, wire_data);
                    if (code < 0)
                    {
                        /* Clean up wire data */
                        if (wire_data->points != NULL)
                        {
                            prc_free(ctx, wire_data->points);
                            prc_free(ctx, wire_data);
                            data->exact_geom_tess_part[geom_count].shells[i].faces[j].wire_data = NULL;
                        }
                        prc_error(ctx, code, "Failed in prc_sample_curve\n");
                        return code;
                    }

                    (*num_tessellations)++;
                    break;
                }
                default:
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                break;
            }
            case PRC_TYPE_TOPO_BrepData:
            {
                /* Lets get all the way to the surface geometry that we need to tessellate */
                /* But we need to handle multiple faces */
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_3D;
                prc_topo_brep_data *brep_data = topo->topo_brep_data;
                uint8_t orientation;

                /* Skip a number of cases as we learn to walk before running */
                if (brep_data->number_of_connex > 1 ||
                    brep_data->number_of_connex == 0)
                {
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                if (brep_data->connex[0].is_stored == 1)
                {
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                if (brep_data->connex[0].topo->topo_connex->shells[i].is_stored == 1)
                {
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                if (brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell->faces[j].face.is_stored == 1)
                {
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                orientation = brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell->faces[j].orientation;
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].orientation = orientation;
                prc_topo_face *topo_face = brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell->faces[j].face.topo->topo_face;
                prc_topo_context *topo_context = brep_data->connex[0].topo->topo_context; /* Not sure which topo context I should use for loop information */
                code = prc_tessellate_surface(ctx, data, i, j, topo_face, orientation, topo->brep_ref_data, topo_context);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_sample_curve\n");
                    return code;
                }
                (*num_tessellations)++;
                break;
            }
            case PRC_TYPE_TOPO_WireEdge:
            case PRC_TYPE_TOPO_Edge:
            case PRC_TYPE_TOPO_CoEdge:
            case PRC_TYPE_TOPO_Loop:
            case PRC_TYPE_TOPO_WireBody:
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                break;

            case PRC_TYPE_TOPO_BrepDataCompress:
            {
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_3D;
                prc_topo_brep_data_compress *brep_data_comp = topo->topo_brep_data_compress;
                uint8_t orientation = 0;
                prc_compressed_face *compressed_face;

                /* Skip a number of cases as we learn to walk before running */
                if (!brep_data_comp->single_connex_test)
                {
                    /* We don't handle multi-connex here yet */
                    data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }

                data->exact_geom_tess_part[geom_count].shells[i].faces[j].orientation = orientation;
                compressed_face = &brep_data_comp->single_connex.faces[j];
                code = prc_tessellate_compressed_face(ctx, data, i, j,
                                    compressed_face, brep_data_comp->ref_data);
                if (code < 0)
                {
                    prc_error(ctx, code, "Failed in prc_sample_curve\n");
                    return code;
                }
                (*num_tessellations)++;
                break;
            }
            case PRC_TYPE_TOPO_Body:
            case PRC_TYPE_TOPO_Face:
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                break;

            default:
                data->exact_geom_tess_part[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
            }
        }
    }

    return 0;
}