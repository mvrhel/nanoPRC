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

#define CURVE_SAMPLES 256
#define SURFACE_SAMPLES 32
#define CURVE_PRECISION 1e-6
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

/* A standard type for curve sampling */
typedef prc_vec3 (*curve_func)(prc_context *ctx, void *params, double input);

/* And for surfaces */
typedef prc_vec3 (*surface_func)(prc_context *ctx, void *params, double u, double v);

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

/* Forward declaration - populates sampling_info (including the valid parametric domain)
   for any prc_type_surf; needed early by the Blend02 bound-projection helpers */
static int prc_get_surface_data(prc_context *ctx, prc_type_surf *surface,
    prc_surface_sampling_info *sampling_info);

/* Forward declaration - used by prc_evaluate_composite before the function body appears */
static int prc_get_curve_sample_info(prc_context *ctx, prc_data *data, prc_ptr_curve *ptr_curve,
    prc_curve_sampling_info *sample_info);

/* Forward declaration - used in curves before surfaces occur */
static int prc_get_surface_eval_func(prc_context *ctx, prc_type_surf *surface,
    surface_func *eval_func, void **params);

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

/* Evaluate parabola at a single point using the modified spec formula (spec has an error) */
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
        output.y = 2.0 * focal_length * sqrt(p2 / focal_length);
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
   and falling along the circle arc. */
static prc_vec3
prc_evaluate_circle_compressed(prc_context *ctx, void *params, double input)
{
    prc_vec3 output = { 0 };
    prc_hcg_circle *circle = (prc_hcg_circle *)params;
    prc_vec3 point1;
    prc_vec3 point2;

    return output;
}

/* For the compressed line, which has the start_end_data as its parameters
   and we just run from zero to one.  We really should only be here if the
   data is given as a point not a vertex. */
static prc_vec3
prc_evaluate_line_compressed(prc_context *ctx, void *params, double input)
{
    prc_vec3 output;
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
    code = prc_get_curve_sample_info(ctx, NULL, &subcurve->ptr_curve, &subcurve_info);
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
prc_get_compressed_curve_sample_info(prc_context *ctx, prc_data *data, prc_compressed_curve *curve,
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
               specified */
            sample_info->curve_params = &curve->hcg_circle;
            sample_info->curve_eval_func = prc_evaluate_circle_compressed;
            sample_info->start = 0;
            sample_info->end = 1;
            sample_info->num_samples = CURVE_SAMPLES;
            break;
        }

        case PRC_HCG_BsplineHermiteCurve:
        {
            break;
        }

        case PRC_HCG_CompositeCurve:
        {
            break;
        }

        default:
            prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid base base curve type in prc_get_compressed_curve_sample_info\n");
            return PRC_ERROR_INTERNAL;

        }
    return 0;
}

static int 
prc_get_curve_sample_info(prc_context *ctx, prc_data *data, prc_ptr_curve *ptr_curve,
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

            sample_info->curve_params = NULL;
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
            code = prc_get_curve_sample_info(ctx, data, &base_curve, sample_info);
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
            code = prc_get_curve_sample_info(ctx, data, &onsurf->uv_curve, &base_curve_sample_info);
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

static int
prc_sample_compressed_curve(prc_context *ctx, prc_data *data, uint32_t shell_index,
    uint32_t face_index, prc_compressed_curve *curve)
{
    uint32_t geom_count = data->exact_geom_tess_count;
    uint32_t file_index = data->exact_geom_tess[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess[geom_count].body_index;
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

    code = prc_get_compressed_curve_sample_info(ctx, data, curve, &sample_info);
    if (code < 0)
    {
        return code;
    }
    start = sample_info.start;
    end = sample_info.end;
    num_samples = sample_info.num_samples;
    curve_params = sample_info.curve_params;
    curve_eval_func = sample_info.curve_eval_func;

    return 0;
}

static int
prc_sample_curve(prc_context *ctx, prc_data *data, uint32_t shell_index,
    uint32_t face_index, prc_content_wire_edge *curve)
{
    uint32_t geom_count = data->exact_geom_tess_count;
    uint32_t file_index = data->exact_geom_tess[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess[geom_count].body_index;
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

    code = prc_get_curve_sample_info(ctx, data, &curve->ptr_curve, &sample_info);
    if (code < 0)
    {
        return code;
    }
    start = sample_info.start;
    end = sample_info.end;
    num_samples = sample_info.num_samples;
    curve_params = sample_info.curve_params;
    curve_eval_func = sample_info.curve_eval_func;

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
                code = prc_get_curve_sample_info(ctx, data, &sub_curve->ptr_curve, &sub_curve_info);
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
            data->exact_geom_tess[geom_count].shells[shell_index].faces[face_index].type = PRC_EXACT_GEOM_UNKNOWN;
            return 0;
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
    data->exact_geom_tess[geom_count].shells[shell_index].faces[face_index].wire_data =
        (prc_exact_geom_wire_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_wire_data));
    if (data->exact_geom_tess[geom_count].shells[shell_index].faces[face_index].wire_data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of wire_data in prc_sample_curve\n");
        return PRC_ERROR_MEMORY;
    }

    prc_exact_geom_wire_data *wire_data = data->exact_geom_tess[geom_count].shells[shell_index].faces[face_index].wire_data;
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
    prc_surf_torus *torus = (prc_surf_torus *)params;
    double major_radius = torus->major_radius;
    double minor_radius = torus->minor_radius;
    double radius;

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
    prc_surf_fromcurves *surf = (prc_surf_fromcurves *)params;
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
    prc_surf_cone *cone = (prc_surf_cone *)params;
    double bottom_radius = cone->radius;
    double semi_angle = cone->semi_angle;
    double radius;

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
    prc_surf_sphere *sphere = (prc_surf_sphere *)params;
    double radius = sphere->radius;

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
    prc_surf_cylinder *cylinder = (prc_surf_cylinder *)params;
    double radius = cylinder->radius;

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
    prc_surf_plane *plane = (prc_surf_plane *)params;
    double u_parameter_coeff_a = plane->u_parameter_coeff_a;
    double v_parameter_coeff_a = plane->v_parameter_coeff_a;
    double u_parameter_coeff_b = plane->u_parameter_coeff_b;
    double v_parameter_coeff_b = plane->v_parameter_coeff_b;

    output.x = u * u_parameter_coeff_a + u_parameter_coeff_b;
    output.y = v * v_parameter_coeff_a + v_parameter_coeff_b;
    output.z = 0;

    if (!plane->exact_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &plane->exact_transform, output);
    }
    return output;
}

static prc_vec3
prc_evaluate_surf_nurbs(prc_context *ctx, void *params, double u, double v)
{
    prc_surf_nurbs *nurbs = (prc_surf_nurbs *)params;
    prc_vec3 output = { 0.0, 0.0, 0.0 };
    double Nu[PRC_BSPLINE_MAX_DEGREE + 1];
    double Nv[PRC_BSPLINE_MAX_DEGREE + 1];
    uint32_t num_ctrl_u = nurbs->highest_index_of_control_points_u + 1;
    uint32_t num_ctrl_v = nurbs->highest_index_of_control_points_v + 1;
    uint32_t span_u, span_v, i, j;
    double x = 0.0, y = 0.0, z = 0.0, weight_sum = 0.0;

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
            double basis = Nu[i] * Nv[j] * weight;

            x += basis * cp->x;
            y += basis * cp->y;
            z += basis * cp->z;
            weight_sum += basis;
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
    prc_surf_extrusion *extrusion = (prc_surf_extrusion *)params;
    curve_func base_curve_func = NULL;
    void *base_curve_params = NULL;
    prc_vec3 sweep_vector = extrusion->sweep_vector;
    double curve_max_u = 0.0;
    double curve_min_u = 0.0;
    int code;

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
    prc_surf_revolution *revolution = (prc_surf_revolution *)params;
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

/* Minimizes the squared distance from point to surface_eval_func(u,v) via coarse grid
   sampling followed by Gauss-Newton refinement (tangents from finite differences) */
static prc_vec3
prc_project_point_onto_surface(prc_context *ctx, surface_func eval_func, void *surface_params,
    double min_u, double max_u, double min_v, double max_v, prc_vec3 point)
{
    const uint32_t coarse_samples = 12;
    const uint32_t max_iterations = 30;
    double hu = (max_u - min_u) * 1e-5;
    double hv = (max_v - min_v) * 1e-5;
    double best_u = min_u, best_v = min_v, best_dist_sq = -1.0;
    uint32_t i, j, iter;
    double u, v;

    if (hu < 1e-9)
        hu = 1e-9;
    if (hv < 1e-9)
        hv = 1e-9;

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

    u = best_u;
    v = best_v;
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

    return eval_func(ctx, surface_params, u, v);
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
            sampling_info.start_v, sampling_info.end_v, center);
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

    base_point = base_eval_func(ctx, base_params, u, v);
    output.x = base_point.x * cos(base_point.y);
    output.y = base_point.x * sin(base_point.y);
    output.z = base_point.z;

    if (cylindrical->has_transform && !cylindrical->exact_geom_transform.is_identity)
    {
        output = prc_exact_geom_apply_transform(ctx, &cylindrical->exact_geom_transform, output);
    }

    return output;
}

static prc_vec3
prc_evaluate_surface_grid_point(prc_context *ctx, surface_func surface_eval_func,
    void *surface_params, double start_u, double end_u, double start_v, double end_v,
    uint32_t i, uint32_t j, uint32_t num_samples_u, uint32_t num_samples_v)
{
    double u_den = (num_samples_u > 1) ? (double)(num_samples_u - 1) : 1.0;
    double v_den = (num_samples_v > 1) ? (double)(num_samples_v - 1) : 1.0;
    double u = start_u + (end_u - start_u) * ((double)i / u_den);
    double v = start_v + (end_v - start_v) * ((double)j / v_den);

    return surface_eval_func(ctx, surface_params, u, v);
}

static void
prc_average_four_points(prc_vec3 p00, prc_vec3 p10, prc_vec3 p01, prc_vec3 p11,
    prc_vec3 *avg)
{
    avg->x = 0.25 * (p00.x + p10.x + p01.x + p11.x);
    avg->y = 0.25 * (p00.y + p10.y + p01.y + p11.y);
    avg->z = 0.25 * (p00.z + p10.z + p01.z + p11.z);
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
            if (has_transform)
            {
                prc_trans = &cone->transform;
                exact_geom_trans = &cone->exact_geom_transform;
            }

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
            if (has_transform)
            {
                prc_trans = &cylinder->transform;
                exact_geom_trans = &cylinder->exact_geom_transform;
            }
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
            if (has_transform)
            {
                prc_trans = &blend->transform;
                exact_geom_trans = &blend->exact_geom_transform;
            }

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
            if (has_transform)
            {
                prc_trans = &blend->transform;
                exact_geom_trans = &blend->exact_geom_transform;
            }

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
            if (has_transform)
            {
                prc_trans = &blend->transform;
                exact_geom_trans = &blend->exact_geom_transform;
            }
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
            if (has_transform)
            {
                prc_trans = &cylindrical->transform;
                exact_geom_trans = &cylindrical->exact_geom_transform;
            }
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
            if (has_transform)
            {
                prc_trans = &offset->transform;
                exact_geom_trans = &offset->exact_geom_transform;
            }

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
            if (has_transform)
            {
                prc_trans = &pipe->transform;
                exact_geom_trans = &pipe->exact_geom_transform;
            }
            break;
        }
        case PRC_TYPE_SURF_Plane:
        {
            /* This one does not have the has_transform bit*/
            prc_surf_plane *plane = surface->surf_plane;
            domain = plane->domain;
            has_transform = 1; /* Always has a transform */
            prc_trans = &plane->transform;
            exact_geom_trans = &plane->exact_transform;
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
            if (has_transform)
            {
                prc_trans = &ruled->transform;
                exact_geom_trans = &ruled->exact_geom_transform;
            }
            break;
        }
        case PRC_TYPE_SURF_Sphere:
        {
            prc_surf_sphere *sphere = surface->surf_sphere;
            params = sphere->parameterization;
            has_transform = sphere->has_transform;
            if (has_transform)
            {
                prc_trans = &sphere->transform;
                exact_geom_trans = &sphere->exact_geom_transform;
            }
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
            if (has_transform)
            {
                prc_trans = &revolution->transform;
                exact_geom_trans = &revolution->exact_geom_transform;
            }

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

            if (has_transform)
            {
                prc_trans = &extrusion->transform;
                exact_geom_trans = &extrusion->exact_geom_transform;
            }
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
            if (has_transform)
            {
                prc_trans = &from_curves->transform;
                exact_geom_trans = &from_curves->exact_geom_transform;
            }

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
            if (has_transform)
            {
                prc_trans = &torus->transform;
                exact_geom_trans = &torus->exact_geom_transform;
            }
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
            if (has_transform)
            {
                prc_trans = &transform->transform;
                exact_geom_trans = &transform->exact_geom_transform;
            }
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
prc_tessellate_surface(prc_context *ctx, prc_data *data, uint32_t shell_index, uint32_t face_index,
                            prc_topo_face *topo_face, uint8_t orientation)
{
    int code;
    void *surface_params = NULL;
    surface_func surface_eval_func = NULL;
    uint32_t num_samples_u = 0;
    uint32_t num_samples_v = 0;
    uint32_t geom_count = data->exact_geom_tess_count;
    uint8_t surface_approx_good = 0;
    prc_surface_sampling_info sampling_info = {0};
    double start_u = 0.0;
    double start_v = 0.0;
    double end_u = 0.0;
    double end_v = 0.0;
    double precision_u = SURFACE_PRECISION;
    double precision_v = SURFACE_PRECISION;
    uint32_t max_samples_u = SURFACE_MAX_SAMPLES;
    uint32_t max_samples_v = SURFACE_MAX_SAMPLES;
    uint8_t wrap_u;
    uint8_t wrap_v;
    uint32_t vertex_samples_u;
    uint32_t vertex_samples_v;
    uint32_t cell_count_u;
    uint32_t cell_count_v;
    prc_exact_geom_transform exact_geom_trans;
    prc_type_surf surface = topo_face->surface_geometry.surface;

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
    num_samples_u = sampling_info.num_samples_u;
    num_samples_v = sampling_info.num_samples_v;
    precision_u = sampling_info.precision_u;
    precision_v = sampling_info.precision_v;
    max_samples_u = sampling_info.max_samples_u;
    max_samples_v = sampling_info.max_samples_v;
    start_u = sampling_info.start_u;
    start_v = sampling_info.start_v;
    end_u = sampling_info.end_u;
    end_v = sampling_info.end_v;

    switch (surface.surface_type)
    {
        case PRC_TYPE_SURF_FromCurves:
        {
            prc_surf_fromcurves *from_curves = surface.surf_fromcurves;
            prc_uv_parameterization params = from_curves->parameterization;

            surface_params = (void *)from_curves;
            surface_eval_func = prc_evaluate_surf_fromcurves;
            break;
        }

        case PRC_TYPE_SURF_Cone:
        {
            prc_surf_cone *cone = surface.surf_cone;
            prc_uv_parameterization params = cone->parameterization;

            surface_params = (void *)cone;
            surface_eval_func = prc_evaluate_surf_cone;
            break;
        }

        case PRC_TYPE_SURF_Cylinder:
        {
            prc_surf_cylinder *cylinder = surface.surf_cylinder;
            prc_uv_parameterization params = cylinder->parameterization;

            surface_params = (void *)cylinder;
            surface_eval_func = prc_evaluate_surf_cylinder;
            break;
        }

        case PRC_TYPE_SURF_Sphere:
        {
            prc_surf_sphere *sphere = surface.surf_sphere;
            prc_uv_parameterization params = sphere->parameterization;

            surface_params = (void *)sphere;
            surface_eval_func = prc_evaluate_surf_sphere;
            break;
        }

        case PRC_TYPE_SURF_Torus:
        {
            prc_surf_torus *torus = surface.surf_torus;
            prc_uv_parameterization params = torus->parameterization;

            surface_params = (void *)torus;
            surface_eval_func = prc_evaluate_surf_torus;
            break;
        }

        case PRC_TYPE_SURF_Cylindrical:
        {
            prc_surf_cylindrical *cylindrical = surface.surf_cylindrical;
            prc_uv_parameterization params = cylindrical->parameterization;

            surface_params = (void *)cylindrical;
            surface_eval_func = prc_evaluate_surf_cylindrical;
            break;
        }

        case PRC_TYPE_SURF_Extrusion:
        {
            prc_surf_extrusion *extrusion = surface.surf_extrusion;
            prc_uv_parameterization params = extrusion->parameterization;

            surface_params = (void *)extrusion;
            surface_eval_func = prc_evaluate_surf_extrusion;
            break;
        }

        case PRC_TYPE_SURF_Revolution:
        {
            prc_surf_revolution *revolution = surface.surf_revolution;
            prc_uv_parameterization params = revolution->parameterization;

            surface_params = (void *)revolution;
            surface_eval_func = prc_evaluate_surf_revolution;
            break;
        }

        case PRC_TYPE_SURF_Plane:
        {
            prc_surf_plane *plane = surface.surf_plane;
            prc_domain params = plane->domain;

            surface_params = (void *)plane;
            surface_eval_func = prc_evaluate_surf_plane;
            break;
        }

        case PRC_TYPE_SURF_Offset:
        {
            prc_surf_offset *offset = surface.surf_offset;

            surface_params = (void *)offset;
            surface_eval_func = prc_evaluate_surf_offset;
            break;
        }

        case PRC_TYPE_SURF_NURBS:
        {
            prc_surf_nurbs *nurbs = surface.surf_nurbs;

            surface_params = (void *)nurbs;
            surface_eval_func = prc_evaluate_surf_nurbs;
            break;
        }

        case PRC_TYPE_SURF_Blend02:
        {
            prc_surf_blend02 *blend = surface.surf_blend02;

            surface_params = (void *)blend;
            surface_eval_func = prc_evaluate_surf_blend02;
            break;
        }

        case PRC_TYPE_SURF_Blend01:
        {
            prc_surf_blend01 *blend = surface.surf_blend01;

            surface_params = (void *)blend;
            surface_eval_func = prc_evaluate_surf_blend01;
            break;
        }

        default:
            data->exact_geom_tess[data->exact_geom_tess_count].shells[shell_index].faces[face_index].type = PRC_EXACT_GEOM_UNKNOWN;
            return 0;
    }

    if (surface_eval_func == NULL)
    {
        prc_error(ctx, PRC_ERROR_INTERNAL, "Invalid surface evaluation function in prc_tessellate_surface\n");
        return PRC_ERROR_INTERNAL;
    }

    wrap_u = prc_surface_axis_wraps(start_u, end_u, sampling_info.u_periodic,
        sampling_info.u_period);
    wrap_v = prc_surface_axis_wraps(start_v, end_v, sampling_info.v_periodic,
        sampling_info.v_period);
    vertex_samples_u = wrap_u ? (num_samples_u - 1) : num_samples_u;
    vertex_samples_v = wrap_v ? (num_samples_v - 1) : num_samples_v;
    cell_count_u = wrap_u ? vertex_samples_u : (vertex_samples_u - 1);
    cell_count_v = wrap_v ? vertex_samples_v : (vertex_samples_v - 1);

    /* Now we need to do a tesselation of the surface_eval_func across the start_u, end_u,
       start_v, end_v domain.  The surface_eval_func will return 3D vectors.  Note that
       the range may wrap around (for example 0 maps to 2pi) so we need to check the edges
       of the domain to see if that gets us to the same values (or very close).  For each
       triangle we can do a linearity check to see if that triangle needs to be further
       subdivided.  Once we have sufficiently sampled the domain to get a small error in
       the range then we can go head and create the tessellation */
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
                    prc_vec3 p0 = surface_eval_func(ctx, surface_params, u0, v);
                    prc_vec3 p1 = surface_eval_func(ctx, surface_params, u1, v);
                    prc_vec3 mid = surface_eval_func(ctx, surface_params, 0.5 * (u0 + u1), v);
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
                    prc_vec3 p0 = surface_eval_func(ctx, surface_params, u, v0);
                    prc_vec3 p1 = surface_eval_func(ctx, surface_params, u, v1);
                    prc_vec3 mid = surface_eval_func(ctx, surface_params, u, 0.5 * (v0 + v1));
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
            fprintf(stderr, "prc_tessellate_surface: surface_type=%u orientation=%u wrap_u=%u wrap_v=%u u_linear=%u v_linear=%u num_samples_u=%u num_samples_v=%u vertex_samples_u=%u vertex_samples_v=%u cell_count_u=%u cell_count_v=%u start_u=%f end_u=%f start_v=%f end_v=%f\n",
                topo_face->surface_geometry.surface.surface_type,
                orientation,
                wrap_u,
                wrap_v,
                sampling_info.u_linear,
                sampling_info.v_linear,
                num_samples_u,
                num_samples_v,
                vertex_samples_u,
                vertex_samples_v,
                cell_count_u,
                cell_count_v,
                start_u,
                end_u,
                start_v,
                end_v);
    }

    /* At this point we have the tessellation data */
    data->exact_geom_tess[geom_count].shells[shell_index].faces[face_index].tess_data =
        (prc_exact_geom_tess_data *)prc_calloc(ctx, 1, sizeof(prc_exact_geom_tess_data));
    if (data->exact_geom_tess[geom_count].shells[shell_index].faces[face_index].tess_data == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data in prc_tessellate_surface\n");
        return PRC_ERROR_MEMORY;
    }

    prc_exact_geom_tess_data *tess_data = data->exact_geom_tess[geom_count].shells[shell_index].faces[face_index].tess_data;
    tess_data->number_of_vertices = vertex_samples_u * vertex_samples_v;
    tess_data->vertices =
        (prc_exact_geom_vertex *)prc_calloc(ctx,
            tess_data->number_of_vertices,
            sizeof(prc_exact_geom_vertex));
    if (tess_data->vertices == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data vertices in prc_tessellate_surface\n");
        return PRC_ERROR_MEMORY;
    }

    tess_data->number_of_triangles =
        cell_count_u * cell_count_v * 2;
    tess_data->triangles =
        (uint32_t *)prc_calloc(ctx,
            tess_data->number_of_triangles * 3,
            sizeof(uint32_t));
    if (tess_data->triangles == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of tess_data triangles in prc_tessellate_surface\n");
        return PRC_ERROR_MEMORY;
    }

    {
        uint32_t i, j;
        double du = (vertex_samples_u > 1) ? fabs(end_u - start_u) /
            (double)(wrap_u ? vertex_samples_u : (vertex_samples_u - 1)) : 1.0;
        double dv = (vertex_samples_v > 1) ? fabs(end_v - start_v) /
            (double)(wrap_v ? vertex_samples_v : (vertex_samples_v - 1)) : 1.0;
        uint32_t triangle_index = 0;

        for (j = 0; j < vertex_samples_v; j++)
        {
            for (i = 0; i < vertex_samples_u; i++)
            {
                uint32_t vertex_index = j * vertex_samples_u + i;
                prc_vec3 position;
                prc_vec3 normal;
                double u = prc_get_surface_param(start_u, end_u, i, vertex_samples_u, wrap_u);
                double v = prc_get_surface_param(start_v, end_v, j, vertex_samples_v, wrap_v);

                position = surface_eval_func(ctx, surface_params, u, v);

                code = prc_compute_surface_normal(ctx, surface_eval_func, surface_params,
                    u, v, du, dv, start_u, end_u, start_v, end_v, &sampling_info,
                    orientation, &normal);
                if (code < 0)
                {
                    return code;
                }

                tess_data->vertices[vertex_index].position[0] = (float)position.x;
                tess_data->vertices[vertex_index].position[1] = (float)position.y;
                tess_data->vertices[vertex_index].position[2] = (float)position.z;
                tess_data->vertices[vertex_index].normal[0] = (float)normal.x;
                tess_data->vertices[vertex_index].normal[1] = (float)normal.y;
                tess_data->vertices[vertex_index].normal[2] = (float)normal.z;
            }
        }

        for (j = 0; j < cell_count_v; j++)
        {
            for (i = 0; i < cell_count_u; i++)
            {
                uint32_t next_i = (i + 1 == vertex_samples_u && wrap_u) ? 0 : (i + 1);
                uint32_t next_j = (j + 1 == vertex_samples_v && wrap_v) ? 0 : (j + 1);
                uint32_t i00 = j * vertex_samples_u + i;
                uint32_t i10 = j * vertex_samples_u + next_i;
                uint32_t i01 = next_j * vertex_samples_u + i;
                uint32_t i11 = next_j * vertex_samples_u + next_i;
                uint32_t *triangles = tess_data->triangles;

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
            prc_topo_connex *connex = brep_data->connex[0].topo->topo_connex;
            *num_shells = connex->number_of_shells;
            for (uint32_t i = 0; i < connex->number_of_shells; i++)
            {
                prc_topo_shell *shell = connex->shells[i].topo->topo_shell;
                *num_faces += prc_count_faces_in_shell(ctx, shell);
            }
        }
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
    uint32_t geom_count = data->exact_geom_tess_count;
    uint32_t file_index = data->exact_geom_tess[geom_count].file_index;
    uint32_t topo_index = data->exact_geom_tess[geom_count].topo_context_index;
    uint32_t body_index = data->exact_geom_tess[geom_count].body_index;
    int code;
    uint32_t num_shells, num_faces, num_wires;
    uint32_t i, j;

    /* Lets figure out if we are doing a curve or a triangle tessellation */
    prc_topo *topo = &data->file_struct[file_index].geometry->exact_geometry.topo_contexts[topo_index].bodies[body_index];

    /* Find out how many shells and how many faces we are dealing with here */
    prc_count_shells_faces_in_topo(ctx, topo, &num_shells, &num_faces);
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
    data->exact_geom_tess[geom_count].shells = (prc_exact_geom_shell *)prc_calloc(ctx, num_shells, sizeof(prc_exact_geom_shell));
    if (data->exact_geom_tess[geom_count].shells == NULL)
    {
        prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of shells in prc_approximate_objects_exact_geom\n");
        return PRC_ERROR_MEMORY;
    }
    data->exact_geom_tess[geom_count].number_of_shells = num_shells;
    for (i = 0; i < num_shells; i++)
    {
        uint32_t num_faces_in_shell = (num_wires == 1) ? 1 : prc_count_faces_in_shell(ctx, topo->topo_brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell);
        data->exact_geom_tess[geom_count].shells[i].faces = (prc_exact_geom_face *)prc_calloc(ctx, num_faces, sizeof(prc_exact_geom_face));
        if (data->exact_geom_tess[geom_count].shells[i].faces == NULL)
        {
            prc_error(ctx, PRC_ERROR_MEMORY, "Allocation failure of faces in prc_approximate_objects_exact_geom\n");
            return PRC_ERROR_MEMORY;
        }
        data->exact_geom_tess[geom_count].shells[i].number_of_faces = num_faces_in_shell;
    }

    /* Now loop on the shells and the faces */
    for (i = 0; i < num_shells; i++)
    {
        for (j = 0; j < data->exact_geom_tess[geom_count].shells[i].number_of_faces; j++)
        {
            switch (topo->tag)
            {
            case PRC_TYPE_TOPO_SingleWireBodyCompress:
            {
                data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_WIRE;
                 
                prc_topo_single_wire_compress *body = topo->topo_single_wire_compress;
                code = prc_sample_compressed_curve(ctx, data, i, j, &body->compressed_curve);
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
                data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_WIRE;

                prc_topo_single_wire_body *body = topo->topo_single_wire_body;
                if (body->wire_body.is_stored == 1)
                {
                    /* We have to find this one. For now we skip this case */
                    data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }

                /* Get the type of topo that this is. For now we just do wire edge.. */
                switch (body->wire_body.topo->tag)
                {
                case PRC_TYPE_TOPO_WireEdge:
                {
                    prc_topo_wire_edge *wire_edge = body->wire_body.topo->topo_wire_edge;
                    code = prc_sample_curve(ctx, data, i, j, &wire_edge->curve);
                    if (code < 0)
                    {
                        prc_error(ctx, code, "Failed in prc_sample_curve\n");
                        return code;
                    }
                    (*num_tessellations)++;
                    break;
                }
                default:
                    data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                break;
            }
            case PRC_TYPE_TOPO_BrepData:
            {
                /* Lets get all the way to the surface geometry that we need to tessellate */
                /* But we need to handle multiple faces */
                data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_3D;
                prc_topo_brep_data *brep_data = topo->topo_brep_data;
                uint8_t orientation;

                /* Skip a number of cases as we learn to walk before running */
                if (brep_data->number_of_connex > 1 ||
                    brep_data->number_of_connex == 0)
                {
                    data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                if (brep_data->connex[0].is_stored == 1)
                {
                    data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                if (brep_data->connex[0].topo->topo_connex->shells[i].is_stored == 1)
                {
                    data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                if (brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell->faces[j].face.is_stored == 1)
                {
                    data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                    return 0;
                }
                orientation = brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell->faces[j].orientation;
                data->exact_geom_tess[geom_count].shells[i].faces[j].orientation = orientation;
                prc_topo_face *topo_face = brep_data->connex[0].topo->topo_connex->shells[i].topo->topo_shell->faces[j].face.topo->topo_face;
                code = prc_tessellate_surface(ctx, data, i, j, topo_face, orientation);
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
                data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                break;

            case PRC_TYPE_TOPO_Body:
            case PRC_TYPE_TOPO_BrepDataCompress:
            case PRC_TYPE_TOPO_Face:
                data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
                break;

            default:
                data->exact_geom_tess[geom_count].shells[i].faces[j].type = PRC_EXACT_GEOM_UNKNOWN;
            }
        }
    }

    return 0;
}