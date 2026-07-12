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

#define VERTEX_DEBUG 0

#include "prc_parse_tess.h"
#include "prc_parse_common.h"
#include "prc_decode_compressed_tess.h"
#include "debug.h"
#include <string.h>
#include <float.h>
#include "prc_vector_util.h"
#include "prc_huff.h"
#include <stdio.h>
//#include "prc_json_debug.h"  /* Used for debug */

#define PRC_NORMAL_INDICES_CAPACITY 3
typedef struct prc_normal_indices_list_s prc_normal_indices_list;
struct prc_normal_indices_list_s
{
    int count;
    int capacity;
    int *indices;
};

static int
push_stack(prc_context *ctx, prc_triangle_stack *stack, prc_treated_details *details)
{
    prc_treated_details *new_details;

    new_details = (prc_treated_details *)prc_malloc(ctx, sizeof(prc_treated_details));
    if (new_details == NULL)
        return PRC_ERROR_MEMORY;

    memcpy(new_details, details, sizeof(prc_treated_details));
    new_details->next = stack->details;
    stack->details = new_details;

    stack->size++;

    return 0;
}

static int
pop_stack(prc_context *ctx, prc_triangle_stack *stack, prc_treated_details *details)
{
    prc_treated_details *old_details;

    if (stack->details == NULL)
        return PRC_ERROR_PARSE;

    old_details = stack->details;
    memcpy(details, old_details, sizeof(prc_treated_details));
    stack->details = old_details->next;
    prc_free(ctx, old_details);

    stack->size--;

    return 0;
}

static double prc_taylor_sin(double x);

static double
prc_taylor_cos(double x)
{
    if (x > PRC_PI_OVER_4)
        return prc_taylor_sin(PRC_HALF_PI - x);
    else
        return 1.0 - (x * x) / 2.0 + (x * x * x * x) / 24.0 - (x * x * x * x * x * x) / 720.0;
}

static double
prc_taylor_sin(double x)
{
    if (x > PRC_PI_OVER_4)
        return prc_taylor_cos(PRC_HALF_PI - x);
    else
        return x - (x * x * x) / 6.0 + (x * x * x * x * x) / 120.0 - (x * x * x * x * x * x * x) / 5040.0;
}

/* Compute the normal vector from the triangle sorted vertices, the vertex normal
   angles, and the vertex normal bits */
static int
prc_compute_vertex_normal(prc_vec3 V1, prc_vec3 V2, prc_vec3 V3, double theta,
    double phi, uint8_t tri_reversed, uint8_t x_reversed, uint8_t y_reversed,
    prc_vec3 *vertex_normal)
{
    prc_vec3 V1_norm, V2_norm, V3_norm, temp1, temp2;
    int code;
    double theta1 = 0, theta2 = 0, theta3 = 0;
    prc_vec3 X_norm, Y_norm, Z_norm;
    double cos_theta, sin_theta, cos_phi, sin_phi, factor1, factor2;
    uint8_t computed_basis = 0;

    prc_vec_sub(V2, V1, &V1_norm);
    code = prc_vec_normalize(&V1_norm);
    if (code < 0)
    {
        return code;
    }

    prc_vec_sub(V3, V1, &V2_norm);
    code = prc_vec_normalize(&V2_norm);
    if (code < 0)
    {
        return code;
    }

    prc_vec_sub(V3, V2, &V3_norm);
    code = prc_vec_normalize(&V3_norm);
    if (code < 0)
    {
        return code;
    }

    code = prc_vec_angle_between_vectors_normal(V1_norm, V2_norm, &theta1);
    if (code < 0)
    {
        return code;
    }

    prc_vec_copy(V1_norm, &temp1, 1);
    code = prc_vec_angle_between_vectors_normal(V3_norm, temp1, &theta2);
    if (code < 0)
    {
        return code;
    }

    prc_vec_copy(V2_norm, &temp1, 1);
    prc_vec_copy(V3_norm, &temp2, 1);
    code = prc_vec_angle_between_vectors_normal(temp1, temp2, &theta3);
    if (code < 0)
    {
        return code;
    }

    if ((theta1 < theta2) && (theta1 < theta3))
    {
        DEBUG_LOG("Normal (theta1 < theta2) && (theta1 < theta3), so z = cross prod(v1, v2)\n");
        prc_vec_copy(V1_norm, &X_norm, 0); /* Per message from Ian */
        prc_vec_cross(V1_norm, V2_norm, &Z_norm);
    }
    else
    {
        if (theta2 < theta3) /* Different than spec */
        {
            DEBUG_LOG("Normal (theta2 < theta3), so z = cross prod(-v3, v1)\n");
            prc_vec_copy(V3_norm, &X_norm, 0);
            prc_vec_copy(V3_norm, &temp1, 1);
            prc_vec_cross(temp1, V1_norm, &Z_norm);
        }
        else
        {
            prc_vec_copy(V2_norm, &X_norm, 1);
            prc_vec_cross(V2_norm, V3_norm, &Z_norm);
        }
    }

    code = prc_vec_normalize(&Z_norm);
    if (code < 0)
    {
        prc_basis basis;

        basis.X = X_norm;

        /* X_norm is either V1_norm, V2_norm, or V3_norm all of which have
           already been normalized */
        int code2 = prc_vec_make_orth_basis_normals(&basis);
        if (code2 < 0)
        {
            return code2;
        }
        computed_basis = 1;
        Y_norm = basis.Y;
        Z_norm = basis.Z;

        /* Now reverse Z if needed */
        if (tri_reversed)
        {
            prc_vec_negate(&Z_norm);
        }

        /* Now get Y from Z and X */
        prc_vec_cross(Z_norm, X_norm, &Y_norm);
        code = prc_vec_normalize(&Y_norm);
        if (code < 0)
        {
            return code;
        }

        if (x_reversed)
        {
            prc_vec_negate(&X_norm);
        }

        if (y_reversed)
        {
            prc_vec_negate(&Y_norm);
        }

        DEBUG_LOG("theta1 = %f, theta2 = %f, theta3 = %f\n", theta1, theta2, theta3);
        DEBUG_LOG("x_reversed = %d, y_reversed = %d, tri_reversed = %d\n", x_reversed, y_reversed, tri_reversed);
    }

    if (!computed_basis)
    {
        /* Reverse triangle */
        if (tri_reversed)
        {
            DEBUG_LOG("Normal triangle reversed, so z vector reversed\n");
            prc_vec_negate(&Z_norm);
        }

        prc_vec_cross(Z_norm, X_norm, &Y_norm);

        code = prc_vec_normalize(&Y_norm);
        if (code < 0)
        {
            return code;
        }

         /* Now apply the normal bits */
         /* Reverse X */
        if (x_reversed)
        {
            DEBUG_LOG("Normal x_reversed = true, reverse x vector\n");
            prc_vec_negate(&X_norm);
        }
        /* Reverse Y (this could be a double reverse if computed basis and
           x was reversed) */
        if (y_reversed)
        {
            DEBUG_LOG("Normal y_reversed = true, reverse y vector\n");
            prc_vec_negate(&Y_norm);
        }
    }

    cos_theta = prc_taylor_cos(theta);
    sin_theta = prc_taylor_sin(theta);
    cos_phi = prc_taylor_cos(phi);
    sin_phi = prc_taylor_sin(phi);

    factor1 = cos_theta * cos_phi;
    factor2 = sin_theta * cos_phi;

    prc_vec_scale(factor1, &X_norm);
    prc_vec_scale(factor2, &Y_norm);
    prc_vec_add(X_norm, Y_norm, &temp1);
    prc_vec_scale(sin_phi, &Z_norm);

    prc_vec_add(temp1, Z_norm, vertex_normal);

    /* Need to do one more normalization */
    code = prc_vec_normalize(vertex_normal);
    if (code < 0)
    {
        return code;
    }

    return 0;
}

static void
prc_set_treated_details(prc_context *ctx, prc_treated_details *treated_details,
    prc_vec3 x, prc_vec3 y, prc_vec3 z, prc_vec3 origin, prc_vec3 V0, prc_vec3 V1,
    int treat_index0, int treat_index1, int normal_index0, int normal_index1)
{
    treated_details->x_basis = x;
    treated_details->y_basis = y;
    treated_details->z_basis = z;
    treated_details->origin = origin;
    treated_details->V0 = V0;
    treated_details->V1 = V1;
    treated_details->index0 = treat_index0;
    treated_details->index1 = treat_index1;
    treated_details->normal_index0 = normal_index0;
    treated_details->normal_index1 = normal_index1;
    treated_details->next = NULL;
}

/* Decode the point with the basis and origin and apply tolerance now */
static void
prc_decode_next_point_post_scale(prc_vec3 *new_point, prc_vec3 x, prc_vec3 y,
    prc_vec3 z, prc_vec3 origin, int32_t current_point[], double tolerance)
{
    prc_vec3 temp, temp2;

    x.x = (double)current_point[0] * tolerance * x.x;
    x.y = (double)current_point[0] * tolerance * x.y;
    x.z = (double)current_point[0] * tolerance * x.z;

    y.x = (double)current_point[1] * tolerance * y.x;
    y.y = (double)current_point[1] * tolerance * y.y;
    y.z = (double)current_point[1] * tolerance * y.z;

    z.x = (double)current_point[2] * tolerance * z.x;
    z.y = (double)current_point[2] * tolerance * z.y;
    z.z = (double)current_point[2] * tolerance * z.z;

    prc_vec_add(origin, x, &temp);
    prc_vec_add(temp, y, &temp2);
    prc_vec_add(temp2, z, new_point);
}

static void
prc_set_treated_triangle(prc_context *ctx, treated_triangle *triangle, prc_vec3 V0,
                         prc_vec3 V1, prc_vec3 V2, int index0, int index1, int index2,
                         int normal_index0, int normal_index1, int normal_index2,
                         int uses_reference)
{
    triangle->points[0] = V0;
    triangle->points[1] = V1;
    triangle->points[2] = V2;
    triangle->treated_index[0] = index0;
    triangle->treated_index[1] = index1;
    triangle->treated_index[2] = index2;
    triangle->normal_indices[0] = normal_index0;
    triangle->normal_indices[1] = normal_index1;
    triangle->normal_indices[2] = normal_index2;
    triangle->uses_reference = uses_reference;
}

static void
prc_scale_data_points(prc_context *ctx, const prc_tess_3d_compressed *data,
                            prc_vec3 *point_array_scaled)
{
    int k;

    for (k = 0; k < data->point_array_size / 3; k++)
    {
        point_array_scaled[k].x = ((double) data->point_array[3 * k]) * data->tolerance;
        point_array_scaled[k].y = ((double) data->point_array[3 * k + 1]) * data->tolerance;
        point_array_scaled[k].z = ((double) data->point_array[3 * k + 2]) * data->tolerance;
    }
}

/* Used to compute the normal of the triangle */
static int
prc_derive_normal(prc_context *ctx, const prc_tess_3d_compressed *data,
    uint32_t triangle_index, treated_triangle *treated_tri, prc_vec3 *normal)
{
    prc_vec3 midpoint, v1, v2;
    int code = 0;
    prc_basis basis;

    prc_vec_avg(treated_tri->points[0], treated_tri->points[1], &midpoint);
    prc_vec_sub(treated_tri->points[1], midpoint, &v1);
    prc_vec_sub(treated_tri->points[2], midpoint, &v2);

    prc_vec_cross(v1, v2, normal);

    code = prc_vec_normalize(normal);
    if (code < 0)
    {
        /* When this failure occurs we need to use the other method to make a basis
           Note that v1 has not yet been normalized */
        basis.X = v1;
        code = prc_vec_make_orth_basis_normals(&basis);
        *normal = basis.Z;
    }

    treated_tri->triangle_index = triangle_index;
    treated_tri->normal_was_reversed = data->normal_is_reversed[triangle_index];

    /* We negate the normal if it is NOT reversed */
    if (!data->normal_is_reversed[triangle_index])
    {
        prc_vec_negate(normal);
    }
    return code;
}

/* Compute the first triangle */
static void
prc_compute_first_triangle(prc_context *ctx, const prc_tess_3d_compressed *data,
                                prc_vec3 *point_array_scaled, uint32_t *point_array_index,
                                treated_triangle *treated_tri)
{
    prc_vec3 V0, V1, V2, temp;

    /* Note: Origin is not scaled by the tolerance.
       First point, V0 is only scaled by the tolerance and difference encoded
       I.e. DV0 = (V0 - Origin) / tolerance. */
    prc_vec_add(point_array_scaled[*point_array_index], data->origin_array, &V0);

    /* Point 1 is DV1 = (V1-V0) / tolerance */
    prc_vec_add(point_array_scaled[*point_array_index + 1], V0, &V1);

    /* Point 2 is DV2 = (V2 - (V0 + V1) / 2) / tolerance */
    prc_vec_avg(V0, V1, &temp);
    prc_vec_add(point_array_scaled[*point_array_index + 2], temp, &V2);

    /* Current triangle. Treatment order this one is always 0 1 2 */
    prc_set_treated_triangle(ctx, treated_tri, V0, V1, V2, *point_array_index,
        *point_array_index + 1, *point_array_index + 2, 0, 0, 0, 0);

    *point_array_index += 3;
}

static void
prc_store_triangle_indices(prc_context *ctx, treated_triangle *treated_tri,
    uint32_t *triangle_indices, uint32_t *vertex_normal_indices, int *triangle_indice_count)
{
    if (treated_tri->normal_was_reversed)
    {
        triangle_indices[*triangle_indice_count] = treated_tri->treated_index[0];
        triangle_indices[*triangle_indice_count + 1] = treated_tri->treated_index[1];
        triangle_indices[*triangle_indice_count + 2] = treated_tri->treated_index[2];

        vertex_normal_indices[*triangle_indice_count] = treated_tri->normal_indices[0];
        vertex_normal_indices[*triangle_indice_count + 1] = treated_tri->normal_indices[1];
        vertex_normal_indices[*triangle_indice_count + 2] = treated_tri->normal_indices[2];
    }
    else
    {
        triangle_indices[*triangle_indice_count] = treated_tri->treated_index[0];
        triangle_indices[*triangle_indice_count + 1] = treated_tri->treated_index[2];
        triangle_indices[*triangle_indice_count + 2] = treated_tri->treated_index[1];

        vertex_normal_indices[*triangle_indice_count] = treated_tri->normal_indices[0];
        vertex_normal_indices[*triangle_indice_count + 1] = treated_tri->normal_indices[2];
        vertex_normal_indices[*triangle_indice_count + 2] = treated_tri->normal_indices[1];
    }

    DEBUG_LOG("\ntreatment [%d, %d, %d]\n", triangle_indices[*triangle_indice_count],
        triangle_indices[*triangle_indice_count + 1], triangle_indices[*triangle_indice_count + 2]);

    *triangle_indice_count += 3;
}

static void
prc_store_vertices(prc_context *ctx, treated_triangle treated_tri,
    prc_vec3 *vertices_out, int *vertex_treatment_count)
{
    vertices_out[*vertex_treatment_count] = treated_tri.points[0];
    vertices_out[*vertex_treatment_count + 1] = treated_tri.points[1];
    vertices_out[*vertex_treatment_count + 2] = treated_tri.points[2];

    *vertex_treatment_count += 3;
}

/* This looks at the average of the three vectors to determine if the face
   is reversed. */
static void
prc_is_normal_reversed3_facecase(prc_context *ctx, treated_triangle *treated_tri,
    const prc_vec3 computed_normal, const prc_vec3 normal0, const prc_vec3 normal1,
    const prc_vec3 normal2)
{
    /* Calculate the normal from the current triangle edges */
    prc_vec3 midpoint, v1, v2;
    prc_vec3 cross1;
    double dot_product;

    prc_vec_avg(treated_tri->points[0], treated_tri->points[1], &midpoint);
    prc_vec_sub(treated_tri->points[1], treated_tri->points[0], &v1);
    prc_vec_sub(treated_tri->points[2], midpoint, &v2);

    prc_vec_cross(v2, v1, &cross1);

    /* Compute the dot product. Store in the treated triangle structure */
    dot_product = prc_vec_dot_product(cross1, computed_normal);
    treated_tri->face_was_reversed = dot_product < 0;
    DEBUG_LOG("    is face reversed? per-vertex mode, based on 3x normal average: \n");
    DEBUG_LOG("      [%.17f %.17f %.17f]\n", computed_normal.x, computed_normal.y, computed_normal.z);
    if (treated_tri->face_was_reversed)
    {
        DEBUG_LOG("      answer = true\n");
    }
    else
    {
        DEBUG_LOG("      answer = false\n");
    }
}

/* Only used if must calculate normals is false. This compares the angle encoded
   normal to the one that we get from the vertex winding. */
static void
prc_is_normal_reversed3(prc_context *ctx, treated_triangle *treated_tri,
    const prc_vec3 computed_normal, const prc_vec3 normal0, const prc_vec3 normal1,
    const prc_vec3 normal2)
{
    /* Calculate the normal from the current triangle edges */
    prc_vec3 midpoint, v1, v2;
    prc_vec3 cross1;
    double dot_product;

    prc_vec_avg(treated_tri->points[0], treated_tri->points[1], &midpoint);
    prc_vec_sub(treated_tri->points[1], treated_tri->points[0], &v1);
    prc_vec_sub(treated_tri->points[2], midpoint, &v2);
    prc_vec_cross(v2, v1, &cross1);

    /* Compute the dot product. Store in the treated triangle structure */
    dot_product = prc_vec_dot_product(cross1, computed_normal);

    DEBUG_LOG("   midpoint [%.17f %.17f %.17f]\n", midpoint.x, midpoint.y, midpoint.z);
    DEBUG_LOG("   v1 [%.17f %.17f %.17f]\n", v1.x, v1.y, v1.z);
    DEBUG_LOG("   v2 [%.17f %.17f %.17f]\n", v2.x, v2.y, v2.z);
    DEBUG_LOG("   cross_prod(v2,v1) [%.17e %.17e %.17e]\n", cross1.x, cross1.y, cross1.z);
    DEBUG_LOG("   stored normal [%.17f %.17f %.17f]\n", computed_normal.x,
        computed_normal.y, computed_normal.z);
    DEBUG_LOG("   dot_product(stored,computed) %.17f\n", dot_product);

    treated_tri->normal_was_reversed = dot_product < 0;
}

/* Only used if must calculate normals is false. This compares the angle encoded
   normal to the one that we get from the vertex winding. This has the correct
   way to compute the cross and inner products without normalizations. This
   approach should be used on the averaging one too */
static void
prc_is_normal_reversed(prc_context *ctx, treated_triangle *treated_tri,
    const prc_vec3 computed_normal)
{
    /* Calculate the normal from the current triangle edges */
    prc_vec3 midpoint, v1, v2;
    prc_vec3 cross1;
    double dot_product;

    prc_vec_avg(treated_tri->points[0], treated_tri->points[1], &midpoint);
    prc_vec_sub(treated_tri->points[1], treated_tri->points[0], &v1);
    prc_vec_sub(treated_tri->points[2], midpoint, &v2);

    DEBUG_LOG("   midpoint [%.17f %.17f %.17f]\n", midpoint.x, midpoint.y, midpoint.z);
    DEBUG_LOG("   v1 [%.17f %.17f %.17f]\n", v1.x, v1.y, v1.z);
    DEBUG_LOG("   v2 [%.17f %.17f %.17f]\n", v2.x, v2.y, v2.z);

    prc_vec_cross(v2, v1, &cross1);
    DEBUG_LOG("   cross_prod(v2,v1) [%.17f %.17f %.17f]\n", cross1.x, cross1.y, cross1.z);
    DEBUG_LOG("   stored normal [%.17f %.17f %.17f]\n", computed_normal.x, computed_normal.y, computed_normal.z);
    dot_product = prc_vec_dot_product(cross1, computed_normal);
    DEBUG_LOG("   dot_product(stored,computed) %.17f\n", dot_product);
    if (dot_product < 0)
    {
        DEBUG_LOG("Normal is reversed\n");
    }
    else
    {
        DEBUG_LOG("Normal is not reversed\n");
    }
    treated_tri->normal_was_reversed = dot_product < 0;

    return;
}

static int
prc_set_left_right_edge_indices(prc_context *ctx, const prc_tess_3d_compressed *data,
    int triangle_count, treated_triangle *treated_tri, const prc_treated_details *treated_details,
    const prc_vec3 *normals_vertex)
{
    int right_base[3], left_base[3];
    int temp_base[3];
    uint32_t temp;
    int edge_count = 0;
    uint8_t normal_is_reversed;
    uint8_t right_treatment_swap = 0;
    uint8_t left_treatment_swap = 0;

    /* Triangle treatment indices should be fixed in relation to left and right */
    treated_tri->left_edge.edge_treatement_x = treated_tri->treated_index[0];
    treated_tri->left_edge.edge_treatement_y = treated_tri->treated_index[2];
    treated_tri->left_edge.edge_treatement_z = treated_tri->treated_index[1];

    treated_tri->right_edge.edge_treatement_x = treated_tri->treated_index[1];
    treated_tri->right_edge.edge_treatement_y = treated_tri->treated_index[2];
    treated_tri->right_edge.edge_treatement_z = treated_tri->treated_index[0];

    /* If normal reversed swap left and right */
    if (treated_tri->normal_was_reversed)
    {
        DEBUG_LOG("Normal reversed, swapping left and right bases\n");
        temp = treated_tri->left_edge.edge_treatement_x;
        treated_tri->left_edge.edge_treatement_x = treated_tri->right_edge.edge_treatement_x;
        treated_tri->right_edge.edge_treatement_x = temp;

        temp = treated_tri->left_edge.edge_treatement_y;
        treated_tri->left_edge.edge_treatement_y = treated_tri->right_edge.edge_treatement_y;
        treated_tri->right_edge.edge_treatement_y = temp;

        temp = treated_tri->left_edge.edge_treatement_z;
        treated_tri->left_edge.edge_treatement_z = treated_tri->right_edge.edge_treatement_z;
        treated_tri->right_edge.edge_treatement_z = temp;
    }

    DEBUG_LOG("Edge status: [L R] = [%d %d]\n",
        (data->edge_status_array[triangle_count] & 2) / 2,
        (data->edge_status_array[triangle_count] & 1));

    /* swap x, y if needed */
    if (treated_tri->right_edge.edge_treatement_x > treated_tri->right_edge.edge_treatement_y)
    {
        DEBUG_LOG("right base x treatment > y treatment, swapping x-y on right base\n");
        temp = treated_tri->right_edge.edge_treatement_x;
        treated_tri->right_edge.edge_treatement_x = treated_tri->right_edge.edge_treatement_y;
        treated_tri->right_edge.edge_treatement_y = temp;
        right_treatment_swap = 1;
    }

    if (treated_tri->left_edge.edge_treatement_x > treated_tri->left_edge.edge_treatement_y)
    {
        DEBUG_LOG("left base x treatment > y treatment, swapping x-y on left base\n");
        temp = treated_tri->left_edge.edge_treatement_x;
        treated_tri->left_edge.edge_treatement_x = treated_tri->left_edge.edge_treatement_y;
        treated_tri->left_edge.edge_treatement_y = temp;
        left_treatment_swap = 1;
    }

    /* From spec |0x1 if triangle has a right neighbor |0x2 if it has a left neighbor */
    if (data->edge_status_array[triangle_count] & 1)
    {
        treated_tri->right_edge.edge_status = PRC_EDGE_NOT_TREATED;
        edge_count++;
    }
    else
    {
        treated_tri->right_edge.edge_status = PRC_EDGE_TREATED;
    }
    if (data->edge_status_array[triangle_count] & 2)
    {
        treated_tri->left_edge.edge_status = PRC_EDGE_NOT_TREATED;
        edge_count++;
    }
    else
    {
        treated_tri->left_edge.edge_status = PRC_EDGE_TREATED;
    }

    if (!treated_tri->normal_was_reversed && right_treatment_swap &&
        left_treatment_swap && ((data->edge_status_array[triangle_count] & 2) == 2) &&
        ((data->edge_status_array[triangle_count] & 1) == 0) && 0)
    {
        /* This is in the Auto suspension file. There is a case where the normal
           is not reversed, AND right base x treatment > y AND left base x treatment > y
           AND we have a left edge but not a right edge. In this case, we need to
           process the right edge but not the left edge. Use the original data to
           determine if this is the case */
        DEBUG_LOG("Special case: Forcing right edge to be treated and left edge to be not treated\n");
        treated_tri->right_edge.edge_status = PRC_EDGE_NOT_TREATED;
        treated_tri->left_edge.edge_status = PRC_EDGE_TREATED;
    }
    else if (!treated_tri->normal_was_reversed && right_treatment_swap &&
        left_treatment_swap && ((data->edge_status_array[triangle_count] & 2) == 0) &&
        ((data->edge_status_array[triangle_count] & 1) == 1) && 0)
    {
        /* If normal is reversed AND right base x treatment > y AND left base x treatment > y
           AND we have a right edge but not a left edge. In this case, we need to
           process the left edge but not the right edge. Use the original data to
           determine if this is the case*/
        DEBUG_LOG("Special case: Forcing left edge to be treated and right edge to be not treated\n");
        treated_tri->right_edge.edge_status = PRC_EDGE_TREATED;
        treated_tri->left_edge.edge_status = PRC_EDGE_NOT_TREATED;
    }
    return edge_count;
}

static int
prc_stack_empty(prc_context *ctx, prc_triangle_stack *stack)
{
    return stack->details == NULL;
}

/* This is for when the stack is empty and we have one reference point and two
   encoded points */
static int
prc_set_one_ref_treated_triangle(prc_context *ctx, const prc_tess_3d_compressed *data,
    prc_vec3 *point_array_scaled, uint32_t *point_array_count, prc_vec3 *vertices_out,
    uint8_t *point_is_ref, treated_triangle *treated_tri, int *reference_array_count,
    int *vertex_treatment_count)
{
    prc_vec3 encoded_point1, encoded_point2, ref_point, V1, V2, temp;
    int nz_ref0;
    prc_vec3 V0;
    int code;
    uint32_t num_points = data->point_array_size / 3;

    /* First get the two encoded points */
    encoded_point1 = point_array_scaled[*point_array_count];
    encoded_point2 = point_array_scaled[*point_array_count + 1];

    /* point_reference_array entries and reference_array_count are attacker-
       controlled/derived from the file. Validate before indexing into
       point_reference_array and before using the value to index vertices_out,
       otherwise a crafted file causes an out-of-bounds heap read. */
    if (*reference_array_count < 0 ||
        (uint32_t)(*reference_array_count) >= data->point_reference_array_size)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "reference_array_count out of range\n");
        return PRC_ERROR_PARSE;
    }

    /* And now the one reference point */
    nz_ref0 = data->point_reference_array[*reference_array_count];
    if (nz_ref0 < 0 || (uint32_t)nz_ref0 >= num_points)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "point_reference_array value out of range\n");
        return PRC_ERROR_PARSE;
    }
    ref_point = vertices_out[nz_ref0];

    /* The options are that we can have 0 0 1, 1 0 0, or 0 1 0
        where the 0 represents the encoded points and the 1
        represents the reference point.  The decode depends upon
        the order */
    if (point_is_ref[0] == 1)
    {
        /* 1 0 0 case */
        /* First encoded value is difference of first decoded value and
            reference value.  So to decode we add the reference to the
            encoded value */
        prc_vec_add(encoded_point1, ref_point, &V1);

        /* The second encoded point is the difference between the third
            point and the average of the first two */
        prc_vec_avg(ref_point, V1, &temp);
        prc_vec_add(encoded_point2, temp, &V2);

        prc_set_treated_triangle(ctx, treated_tri, ref_point, V1, V2,
                nz_ref0, *point_array_count, *point_array_count+1, 0, 0, 0, 0);

        DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count);
        DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", V1.x, V1.y, V1.z);

        DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count + 1);
        DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", V2.x, V2.y, V2.z);

        /* Add the new points to the list of vertices */
        prc_vec_copy(V1, &vertices_out[*vertex_treatment_count], 0);
        *vertex_treatment_count += 1;
        prc_vec_copy(V2, &vertices_out[*vertex_treatment_count], 0);
        *vertex_treatment_count += 1;
    }
    else if (point_is_ref[1] == 1)
    {
        /* 0 1 0 case. Have not seen this one yet */
        /* A guess is that we have to add in the origin to the first decoded
           point to get V0. */
        prc_vec_add(encoded_point1, data->origin_array, &V0);

        /* At this point we have 1, 1, 0 */
		/* The second encoded point is the difference between the third
			point and the average of the first two */
		prc_vec_avg(ref_point, V0, &temp);
		prc_vec_add(encoded_point2, temp, &V2);

		prc_set_treated_triangle(ctx, treated_tri, V0, ref_point, V2,
			*point_array_count, nz_ref0, *point_array_count + 1, 0, 0, 0, 0);

		DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count);
		DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", V0.x, V0.y, V0.z);

		DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count + 1);
		DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", V2.x, V2.y, V2.z);

		/* Add the new points to the list of vertices */
		prc_vec_copy(V0, &vertices_out[*vertex_treatment_count], 0);
		*vertex_treatment_count += 1;
		prc_vec_copy(V2, &vertices_out[*vertex_treatment_count], 0);
		*vertex_treatment_count += 1;
    }
    else if (point_is_ref[2] == 1)
    {
        /* 0 0 1 case */
        /* This is the same as the start up of the initial
           triangle. We have to add in the origin to the first
           decoded point to get V0 and then */
        prc_vec_add(encoded_point1, data->origin_array, &V0);
        prc_vec_add(encoded_point2, V0, &V1);

        prc_set_treated_triangle(ctx, treated_tri, V0, V1, ref_point,
                *point_array_count, *point_array_count+1, nz_ref0, 0, 0, 0, 0);

        DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count);
        DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", V0.x, V0.y, V0.z);

        DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count + 1);
        DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", V1.x, V1.y, V1.z);

        /* Add the new points to the list of vertices */
        prc_vec_copy(V0, &vertices_out[*vertex_treatment_count], 0);
        *vertex_treatment_count += 1;
        prc_vec_copy(V1, &vertices_out[*vertex_treatment_count], 0);
        *vertex_treatment_count += 1;
    }
    else
    {
        /* This can't be */
        return PRC_ERROR_PARSE;
    }

    /* One reference point and two decoded points */
    *reference_array_count += 1;
    *point_array_count += 2;

    return 0;
}

/* This is for when the stack is empty and we have two reference points and one
   encoded point. The two reference points have to be the base of the new triangle.
   We need to know which triangle this base came from. */
static int
prc_set_two_ref_treated_triangle(prc_context *ctx, const prc_tess_3d_compressed *data,
    prc_vec3 *point_array_scaled, uint32_t *point_array_count, prc_vec3 *vertices_out,
    uint8_t *point_is_ref, treated_triangle *treated_tri, int *reference_array_count,
    int *vertex_treatment_count)
{
    prc_vec3 encoded_point1, new_point, V0, V1, V2, temp2;
    int nz_ref0, nz_ref1, nz_ref2;
    uint32_t edge_index = 0;
    uint32_t num_points = data->point_array_size / 3;

    /* We have two references and one point to decode */

    /* point_reference_array entries and reference_array_count are attacker-
       controlled/derived from the file. Validate before reading the two
       entries this function consumes, otherwise a crafted file causes an
       out-of-bounds heap read below. */
    if (*reference_array_count < 0 ||
        (uint32_t)(*reference_array_count) + 1 >= data->point_reference_array_size)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "reference_array_count out of range\n");
        return PRC_ERROR_PARSE;
    }

    if (point_is_ref[0] == 0)
    {
        /* Had not seen this one yet. Guess is that the first point is offset
           from the origin */
		prc_vec_add(point_array_scaled[*point_array_count], data->origin_array, &V0);
		nz_ref1 = data->point_reference_array[*reference_array_count];
		nz_ref2 = data->point_reference_array[*reference_array_count + 1];
		if (nz_ref1 < 0 || (uint32_t)nz_ref1 >= num_points ||
		    nz_ref2 < 0 || (uint32_t)nz_ref2 >= num_points)
		{
		    prc_error(ctx, PRC_ERROR_PARSE, "point_reference_array value out of range\n");
		    return PRC_ERROR_PARSE;
		}
		V1 = vertices_out[nz_ref1];
		V2 = vertices_out[nz_ref2];
        prc_vec_copy(V0, &new_point, 0);
        DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count);
        DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", new_point.x, new_point.y, new_point.z);

        /* The new point (V0) goes at the start? */
		prc_set_treated_triangle(ctx, treated_tri, V0, V1, V2, *point_array_count,
			nz_ref1, nz_ref2, 0, 0, 0, 0);
    }
    else if (point_is_ref[1] == 0)
    {
        /* Non-ref point is the second one */
        nz_ref0 = data->point_reference_array[*reference_array_count];
        nz_ref2 = data->point_reference_array[*reference_array_count + 1];
        if (nz_ref0 < 0 || (uint32_t)nz_ref0 >= num_points ||
            nz_ref2 < 0 || (uint32_t)nz_ref2 >= num_points)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "point_reference_array value out of range\n");
            return PRC_ERROR_PARSE;
        }
        V0 = vertices_out[nz_ref0];
        V2 = vertices_out[nz_ref2];
        encoded_point1 = point_array_scaled[*point_array_count];
        prc_vec_add(V0, encoded_point1, &V1);
        prc_vec_copy(V1, &new_point, 0);
        DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count);
        DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", new_point.x, new_point.y, new_point.z);

        /* To make new one compatible the new point (V1) goes in the middle */
        prc_set_treated_triangle(ctx, treated_tri, V0, V1, V2, nz_ref0,
            *point_array_count, nz_ref2, 0, 0, 0, 0);
    }
    else if (point_is_ref[2] == 0)
    {
        /* Non-ref point is the second one */
        /* In this case the encoded point is difference encoded from the
            average of the first two reference points */
        nz_ref0 = data->point_reference_array[*reference_array_count];
        nz_ref1 = data->point_reference_array[*reference_array_count + 1];
        if (nz_ref0 < 0 || (uint32_t)nz_ref0 >= num_points ||
            nz_ref1 < 0 || (uint32_t)nz_ref1 >= num_points)
        {
            prc_error(ctx, PRC_ERROR_PARSE, "point_reference_array value out of range\n");
            return PRC_ERROR_PARSE;
        }
        V0 = vertices_out[nz_ref0];
        V1 = vertices_out[nz_ref1];
        encoded_point1 = point_array_scaled[*point_array_count];
        prc_vec_avg(V0, V1, &temp2);
        prc_vec_add(encoded_point1, temp2, &V2);
        prc_vec_copy(V2, &new_point, 0);
        DEBUG_LOG("Next Pt: %d\n", *vertex_treatment_count);
        DEBUG_LOG("      [%1.5f %1.5f %1.5f]\n", new_point.x, new_point.y, new_point.z);

        /* The new point (V2) goes at the end */
        prc_set_treated_triangle(ctx, treated_tri, V0, V1, V2, nz_ref0,
            nz_ref1, *point_array_count, 0, 0, 0, 0);
    }
    else
    {
        /* This can't be */
        return PRC_ERROR_PARSE;
    }

    *point_array_count += 1;
    *reference_array_count += 2;

    /* Add the new point to the list of vertices */
    prc_vec_copy(new_point, &vertices_out[*vertex_treatment_count], 0);

    /* This number should be the same as the point_array_count.... */
    *vertex_treatment_count += 1;

    return 0;
}

static int
prc_set_three_ref_treated_triangle(prc_context *ctx, const prc_tess_3d_compressed *data,
     prc_vec3 *vertices_out, treated_triangle *treated_tri, int *reference_array_count)
{
    int32_t nz_ref0, nz_ref1, nz_ref2;
    prc_vec3 V0, V1, V2;
    uint32_t num_points = data->point_array_size / 3;

    /* point_reference_array entries and reference_array_count are attacker-
       controlled/derived from the file. Validate before reading the three
       entries this function consumes, otherwise a crafted file causes an
       out-of-bounds heap read below. */
    if (*reference_array_count < 0 ||
        (uint32_t)(*reference_array_count) + 2 >= data->point_reference_array_size)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "reference_array_count out of range\n");
        return PRC_ERROR_PARSE;
    }

    nz_ref0 = data->point_reference_array[*reference_array_count];
    nz_ref1 = data->point_reference_array[*reference_array_count + 1];
    nz_ref2 = data->point_reference_array[*reference_array_count + 2];
    if (nz_ref0 < 0 || (uint32_t)nz_ref0 >= num_points ||
        nz_ref1 < 0 || (uint32_t)nz_ref1 >= num_points ||
        nz_ref2 < 0 || (uint32_t)nz_ref2 >= num_points)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "point_reference_array value out of range\n");
        return PRC_ERROR_PARSE;
    }
    V0 = vertices_out[nz_ref0];
    V1 = vertices_out[nz_ref1];
    V2 = vertices_out[nz_ref2];

    prc_set_treated_triangle(ctx, treated_tri, V0, V1, V2, nz_ref0,
                                nz_ref1, nz_ref2, 0, 0, 0, 0);
    *reference_array_count += 3;
    return 0;
}

static int
prc_compute_triangle_basis(prc_context *ctx, const prc_vec3 *vertices_out,
    const treated_triangle *treated_tri, prc_treated_details *treated_details,
    prc_edge_case_t edge_case)
{
    prc_vec3 V0, V1, V3;  /* Already treated triangle, like in the specification */
    prc_vec3 y_copy;
    prc_vec3 x, y, z, z_temp, w;
    prc_vec3 origin_new;
    int code;
    uint8_t use_alternate_basis = 0;
    prc_basis basis;

    if (edge_case != PRC_EDGE_RIGHT && edge_case != PRC_EDGE_LEFT)
    {
        return PRC_ERROR_PARSE;
    }
    treated_details->edge_case = edge_case;
    if (edge_case == PRC_EDGE_RIGHT)
    {
        V0 = vertices_out[treated_tri->right_edge.edge_treatement_x];
        V1 = vertices_out[treated_tri->right_edge.edge_treatement_y];
        V3 = vertices_out[treated_tri->right_edge.edge_treatement_z];
        DEBUG_LOG("****Computing RIGHT Basis\n");
    }
    else
    {
        V0 = vertices_out[treated_tri->left_edge.edge_treatement_x];
        V1 = vertices_out[treated_tri->left_edge.edge_treatement_y];
        V3 = vertices_out[treated_tri->left_edge.edge_treatement_z];
        DEBUG_LOG("****Computing LEFT Basis\n");
    }

    DEBUG_LOG("V0: [%.17f %.17f %.17f]\n", V0.x, V0.y, V0.z);
    DEBUG_LOG("V1: [%.17f %.17f %.17f]\n", V1.x, V1.y, V1.z);
    DEBUG_LOG("V3: [%.17f %.17f %.17f]\n", V3.x, V3.y, V3.z);

    prc_vec_avg(V0, V1, &origin_new);

    DEBUG_LOG("Origin: [%.17f %.17f %.17f]\n", origin_new.x, origin_new.y, origin_new.z);
    prc_vec_sub(V0, V1, &x);

    DEBUG_LOG("X: [%.17f %.17f %.17f]\n", x.x, x.y, x.z);
    code = prc_vec_normalize(&x);
    if (code < 0)
    {
        return code;
    }
    DEBUG_LOG("X normalized: [%.17e %.17e %.17e]\n", x.x, x.y, x.z);
    prc_vec_sub(V3, origin_new, &z_temp);
    prc_vec_cross(z_temp, x, &z);

    DEBUG_LOG("Z: [%.17f %.17f %.17f]\n", z.x, z.y, z.z);

    code = prc_vec_normalize(&z);
    DEBUG_LOG("Z normalized: [%.17e %.17e %.17e]\n", z.x, z.y, z.z);

    if (code < 0)
    {
        use_alternate_basis = true;
    }

    if (!use_alternate_basis)
    {
        /* Z and X are already orthonormal. So Y will also be */
        prc_vec_cross(z, x, &y);
        DEBUG_LOG("Y: [%.17f %.17f %.17f]\n", y.x, y.y, y.z);


        /* Check the length of y */
        code = prc_vec_normalize(&y);
        DEBUG_LOG("Y normalized: [%.17e %.17e %.17e]\n", y.x, y.y, y.z);

        if (code < 0)
        {
            use_alternate_basis = true;
        }
    }

    if (use_alternate_basis)
    {
        basis.X = x;
        /* Note that x was normalized above */
        code = prc_vec_make_orth_basis(&basis);
        if (code < 0)
            return code;
        y = basis.Y;
        z = basis.Z;

        /* Odd to me that I have to do this*/
        prc_vec_cross(z, x, &y);
        DEBUG_LOG("Y: [%.17f %.17f %.17f]\n", y.x, y.y, y.z);

        /* Check the length of y */
        code = prc_vec_normalize(&y);
        DEBUG_LOG("Y normalized: [%.17e %.17e %.17e]\n", y.x, y.y, y.z);
    }

    prc_vec_sub(origin_new, V3, &w);
    if (prc_vec_dot_product(y, w) > 0.0)
    {
        prc_vec_negate(&z);
        prc_vec_negate(&y);
    }

    /* Store all the details in treated_details */
    if (edge_case == PRC_EDGE_RIGHT)
    {
        prc_set_treated_details(ctx, treated_details, x, y, z, origin_new, V0, V1,
            treated_tri->right_edge.edge_treatement_x,
            treated_tri->right_edge.edge_treatement_y,
            treated_tri->right_edge.edge_treatement_x,
            treated_tri->right_edge.edge_treatement_y);
    }
    else
    {
        prc_set_treated_details(ctx, treated_details, x, y, z, origin_new, V0, V1,
            treated_tri->left_edge.edge_treatement_x,
            treated_tri->left_edge.edge_treatement_y,
            treated_tri->left_edge.edge_treatement_x,
            treated_tri->left_edge.edge_treatement_y);
    }

    DEBUG_LOG("****End of Basis Calculation\n");
    return 0;
}

static int
prc_handle_empty_stack_decode(prc_context *ctx, prc_tess_3d_compressed *data,
    prc_vec3 *point_array_scaled, prc_vec3 *vertices_out,
    treated_triangle *treated_tri, uint32_t *point_array_count, int *reference_array_count,
    int *vertex_treatment_count, int *points_is_reference_index)
{
    int code;
    uint8_t point_is_ref[3], num_ref_points;

    /* triangle_face_array_size (which drives how many times this function gets
       called) and reference_array_size (which bounds points_is_reference_array)
       are independently attacker-controlled counts read from the file with no
       relation to each other. Validate the index here instead of trusting them
       to stay in sync, otherwise a crafted file walks this read past the end
       of points_is_reference_array. */
    if (*points_is_reference_index < 0 ||
        (uint32_t)(*points_is_reference_index) + 3 > data->reference_array_size)
    {
        prc_error(ctx, PRC_ERROR_PARSE, "points_is_reference_array index out of range\n");
        return PRC_ERROR_PARSE;
    }

    /* Get the next three references */
    point_is_ref[0] = data->points_is_reference_array[*points_is_reference_index];
    point_is_ref[1] = data->points_is_reference_array[*points_is_reference_index + 1];
    point_is_ref[2] = data->points_is_reference_array[*points_is_reference_index + 2];

    DEBUG_LOG("Current Point %d is Ref:%d\n", *points_is_reference_index, point_is_ref[0]);
    DEBUG_LOG("Current Point %d is Ref:%d\n", *points_is_reference_index + 1, point_is_ref[1]);
    DEBUG_LOG("Current Point %d is Ref:%d\n", *points_is_reference_index + 2, point_is_ref[2]);

    num_ref_points = point_is_ref[0] + point_is_ref[1] + point_is_ref[2];
    *points_is_reference_index += 3;

    if (num_ref_points == 0)
    {
        /* In this case, we are essentially rebooting with a fresh
            initial triangle that decodes the next three points */
        prc_compute_first_triangle(ctx, data, point_array_scaled,
            point_array_count, treated_tri);

        /* Store the vertices of the first triangle */
        prc_store_vertices(ctx, *treated_tri, vertices_out, vertex_treatment_count);
    }
    else if (num_ref_points == 1)
    {
        code = prc_set_one_ref_treated_triangle(ctx, data, point_array_scaled,
            point_array_count, vertices_out, point_is_ref, treated_tri,
            reference_array_count, vertex_treatment_count);
        if (code < 0)
        {
            return code;
        }
    }
    else if (num_ref_points == 3)
    {
        code = prc_set_three_ref_treated_triangle(ctx, data, vertices_out,
            treated_tri, reference_array_count);
        if (code < 0)
        {
            return code;
        }
    }
    else
    {
        /* Two reference points and one decode point */
        code = prc_set_two_ref_treated_triangle(ctx, data, point_array_scaled,
            point_array_count, vertices_out, point_is_ref, treated_tri,
            reference_array_count, vertex_treatment_count);
        if (code < 0)
        {
            return code;
        }
    }

    /* We may need to do an adjustment if the first treatement index is greater
       than the second treatment index. This is seen in 3D-PDF-Sample_Aero-Composite-Part
       from tetra4d */
    if (treated_tri->treated_index[0] > treated_tri->treated_index[1])
    {
		DEBUG_LOG("Swapping first two points\n");
        DEBUG_LOG("As first treatement index > 2nd\n");
		int temp = treated_tri->treated_index[0];
		treated_tri->treated_index[0] = treated_tri->treated_index[1];
		treated_tri->treated_index[1] = temp;

		prc_vec3 temp_point = treated_tri->points[0];
		treated_tri->points[0] = treated_tri->points[1];
		treated_tri->points[1] = temp_point;

		int temp_normal = treated_tri->normal_indices[0];
		treated_tri->normal_indices[0] = treated_tri->normal_indices[1];
		treated_tri->normal_indices[1] = temp_normal;
    }
    return 0;
}

/* Here we decode the normals rather than do a computation of the normal
   from the triangle winding. */
static int
prc_decode_normal(prc_context *ctx, prc_tess_3d_compressed *data,
    uint8_t tri_reversed, uint8_t x_reversed, uint8_t y_reversed,
    treated_triangle *treated_tri, prc_normal_state *normal_state,
    uint8_t debug_tess)
{
    prc_vec3 vertex_normal;
    int code;
    double theta, phi;

    /* Get the angles */
    theta = normal_state->decoded_angles[normal_state->angle_index];
    phi = normal_state->decoded_angles[normal_state->angle_index + 1];
    normal_state->angle_index += 2;

    /* Compute the normal vector using the sorted vertices, the angles and the current_bits
        These last two are unique to the current vertex, so that we are computing a normal
        vector for that vertex */
    code = prc_compute_vertex_normal(treated_tri->points[0], treated_tri->points[1],
                                    treated_tri->points[2], theta, phi, tri_reversed,
                                    x_reversed, y_reversed, &vertex_normal);
    if (code < 0)
    {
        return code;
    }

    /* Store this normal */
    DEBUG_LOG("normal_state->normals_vertex_count = %d\n", normal_state->normals_vertex_count);
    DEBUG_LOG("Normal: [%.17f %.17f %.17f]\n", vertex_normal.x, vertex_normal.y, vertex_normal.z);

#if VERTEX_DEBUG
    if (debug_tess)
    {
        if (normal_state->actual_normals != NULL)
        {
            /* Compare the normal to the actual normal */
            if (fabs(vertex_normal.x - normal_state->actual_normals[normal_state->normals_vertex_count].x) > 0.00000000000000100 ||
                fabs(vertex_normal.y - normal_state->actual_normals[normal_state->normals_vertex_count].y) > 0.00000000000000100 ||
                fabs(vertex_normal.z - normal_state->actual_normals[normal_state->normals_vertex_count].z) > 0.00000000000000100)
            {
                DEBUG_LOG("ERROR: Normal %d does not match\n", normal_state->normals_vertex_count);
                DEBUG_LOG("    [%.17f %.17f %.17f]\n", vertex_normal.x, vertex_normal.y, vertex_normal.z);
                DEBUG_LOG("    [%.17f %.17f %.17f]\n", normal_state->actual_normals[normal_state->normals_vertex_count].x,
                    normal_state->actual_normals[normal_state->normals_vertex_count].y, normal_state->actual_normals[normal_state->normals_vertex_count].z);
                //vertex_normal = normal_state->actual_normals[normal_state->normals_vertex_count];
            }
        }
    }
#endif

    normal_state->normals_vertex[normal_state->normals_vertex_count] = vertex_normal;
    normal_state->normals_vertex_count += 1;
    normal_state->normal = vertex_normal; /* This is the normal for the current vertex */

    return 0;
}

/* Decode the angles if we are not calculating the normals */
static int
prc_decode_angles(prc_context *ctx, prc_tess_3d_compressed *data,
    double *decoded_angles)
{
    uint32_t normal_angle_number_of_bits = data->normal_angle_number_of_bits;
    uint32_t normal_angle_array_size = data->normal_angle_array_size;
    uint8_t *normal_binary_data = data->normal_binary_data;
    uint32_t normal_bin_data_index = 0;
    int k;

    for (k = 0; k < normal_angle_array_size; k++)
    {
        decoded_angles[k] = data->normal_angle_array[k] *
            (PRC_PI / 2.0) / ((double)pow(2, normal_angle_number_of_bits) - 1);
    }

    return 0;
}

static uint8_t
prc_face_is_planar(prc_context *ctx, prc_tess_3d_compressed *data, uint32_t triangle_index)
{
    if (data->has_faces && data->is_face_planar != NULL && data->triangle_face_array != NULL &&
        data->is_face_planar[data->triangle_face_array[triangle_index]])
    {
        return 1;
    }
    return 0;
}

static void
prc_is_face_reversed_average(prc_context *ctx, treated_triangle *treated_tri,
    prc_normal_state *normal_state)
{
    /* Compute the average of the last three normals */
    prc_vec_avg3(normal_state->normals_vertex[treated_tri->normal_indices[0]],
    normal_state->normals_vertex[treated_tri->normal_indices[1]],
    normal_state->normals_vertex[treated_tri->normal_indices[2]],
    &normal_state->averaged_normal);

    /* Compare the average of the last three normals to the computed normal to
    *  determine if the triangle was reversed */
    prc_is_normal_reversed3_facecase(ctx, treated_tri, normal_state->averaged_normal,
        normal_state->normals_vertex[treated_tri->normal_indices[0]],
        normal_state->normals_vertex[treated_tri->normal_indices[1]],
        normal_state->normals_vertex[treated_tri->normal_indices[2]]);
}

/* Update the treated triangle with the last three normal indices AND
    compare the average of those normals to the computed normal to
    determine if the triangle was reversed. We do that here after computing
    the normal for EACH of the vertices. */
static void
prc_is_normal_reversed_single_normal(prc_context *ctx, treated_triangle *treated_tri,
                                 prc_normal_state *normal_state)
{
    /* Original behavior: use the normal with the lowest index. */
    /*
    if (treated_tri->normal_indices[0] < treated_tri->normal_indices[1] &&
        treated_tri->normal_indices[0] < treated_tri->normal_indices[2])
    {
        prc_vec_copy(normal_state->normals_vertex[treated_tri->normal_indices[0]],
            &normal_state->averaged_normal, 0);
    }
    else if (treated_tri->normal_indices[1] < treated_tri->normal_indices[0] &&
        treated_tri->normal_indices[1] < treated_tri->normal_indices[2])
    {
        prc_vec_copy(normal_state->normals_vertex[treated_tri->normal_indices[1]],
            &normal_state->averaged_normal, 0);
    }
    else
    {
        prc_vec_copy(normal_state->normals_vertex[treated_tri->normal_indices[2]],
            &normal_state->averaged_normal, 0);
    }
    */
    /* New behavior: Always use the one at V[0] */
    prc_vec_copy(normal_state->normals_vertex[treated_tri->normal_indices[0]],
        &normal_state->averaged_normal, 0);

    /* Compare the average of the last three normals to the computed normal to
    *  determine if the triangle was reversed */
    prc_is_normal_reversed3(ctx, treated_tri, normal_state->averaged_normal,
        normal_state->normals_vertex[treated_tri->normal_indices[0]],
        normal_state->normals_vertex[treated_tri->normal_indices[1]],
        normal_state->normals_vertex[treated_tri->normal_indices[2]]);
}

/* This will return a single normal in normal_state. If we are calculating normals,
   then each triangle has one normal and that will be applied to all three vertices
   of the triangle. It instead we are decoding normals that are in the PRC file then
   there are two cases. In one case, we have a face_is_planar is true and when we
   encounter the first vertex of the first triangle of the face we decode the normal
   and we use that normal for ALL vertices on the face.  If instead face_is_planar
   is false then we decode for each vertex of the triangle. Now, we have to take care
   as some vertices have multiple normals and some do not in this case. That information
   is stored in *normal_bin_data which is indexed by normal_bin_data_index both
   of which are maintained in *normal_state */
static int
prc_handle_normal_calculation(prc_context *ctx, prc_tess_3d_compressed *data,
    treated_triangle *treated_tri, uint32_t *vertex_normal_indices,
    uint32_t triangle_index, prc_normal_state *normal_state, uint8_t debug_tess)
{
    int code, k;
    uint32_t face_index;
    uint8_t *normal_binary_data = data->normal_binary_data;
    uint8_t must_calculate_normals = data->must_recalculate_normals;
    uint8_t *normal_bin_data = &normal_state->normal_bin_data[normal_state->normal_bin_data_index];
    uint32_t vertex_index;
    decoded_normal_info *multiple_normals = normal_state->multiple_normals;
    uint8_t has_multiple_normals, is_a_reference, is_reversed, x_is_reversed;
    uint8_t y_is_reversed;
    uint32_t norm_index;

    /* We need to check if this face is planar based upon the triangle index
       and the arrays in data */
    normal_state->face_is_planar = prc_face_is_planar(ctx, data, triangle_index);

    /* In this case we don't use normal_bin_data */
    if (must_calculate_normals)
    {
        /* Compute the normal */
        code = prc_derive_normal(ctx, data, triangle_index, treated_tri,
                                 &normal_state->normal);
        if (code < 0)
        {
            return code;
        }

        /* Set all three in the triangle to the same normal */
        treated_tri->normal_indices[0] = normal_state->normals_vertex_count;
        treated_tri->normal_indices[1] = normal_state->normals_vertex_count;
        treated_tri->normal_indices[2] = normal_state->normals_vertex_count;

        /* Store the normal that we are indexing */
        normal_state->normals_vertex[normal_state->normals_vertex_count] = normal_state->normal;
        normal_state->normals_vertex_count += 1;

        /* Set the triangle is reversed based upon the triangle is reversed bit.
           This is the only case that uses this data */
        treated_tri->normal_was_reversed = normal_state->normal_is_reversed[triangle_index];

        /* All three vertices have the same normal */
        DEBUG_LOG("Normal is calculated as [%3.5f %3.5f %3.5f]\n", normal_state->normal.x,
                        normal_state->normal.y, normal_state->normal.z);
        DEBUG_LOG("Normal is reversed = %d\n", treated_tri->normal_was_reversed);
    }
    else
    {
        /* Now we need to make use of normal_bin_data.  Note that we can have planar
           normal data mixed in with non-planar normal data. And the non-planar
           data can reference the planar normals in the references. So we need
           to add these to the reference list of normals */
        if (normal_state->face_is_planar)
        {
            /* We have one normal per face */
            /* Get the index of the face and check if we have decoded it yet */
            face_index = data->triangle_face_array[triangle_index];
            DEBUG_LOG("Face Index = %d\n", face_index);
            if (normal_state->face_normal_decoded[face_index])
            {
                DEBUG_LOG("Normal per face mode, ignore normal as not first normal for this face\n");
                /* This face normal has been decoded already so just use that one. */
                prc_vec_copy(normal_state->face_normals[face_index], &normal_state->normal, 0);

                /* And be sure to set the normal indices for the triangle */
                treated_tri->normal_indices[0] = normal_state->face_normal_indices[face_index];
                treated_tri->normal_indices[1] = normal_state->face_normal_indices[face_index];
                treated_tri->normal_indices[2] = normal_state->face_normal_indices[face_index];
            }
            else
            {
                /* First time on this face. */
                /* It is possible that the first vertex of the triangle in
                   non-multiple and has already been set. If it has been,
                   then just use that for the face IF it was not a multiple */

                DEBUG_LOG("Normal per face, first time on this face\n");

                if (multiple_normals[treated_tri->treated_index[0]].vertex_normal_state == PRC_VERTEX_NORM_IS_NOT_MULTIPLE)
                {

                    DEBUG_LOG("Normal per face, norm is not multiple, but was already calculated, use existing normal\n");
                    /* Just use the existing norm of this vertex for the face.*/
                    norm_index = normal_state->multiple_normals[treated_tri->treated_index[0]].non_multiple_normal_index;
                    normal_state->normal = normal_state->normals_vertex[norm_index];

                    /* Set the face normal */
                    normal_state->face_normal_decoded[face_index] = 1;
                    normal_state->face_normals[face_index] = normal_state->normal;

                    /* Set the normal indices for the triangle to what we had used prior */
                    treated_tri->normal_indices[0] = norm_index;
                    treated_tri->normal_indices[1] = norm_index;
                    treated_tri->normal_indices[2] = norm_index;

                    /* And be sure to set what indices the face is using so that
                       when we encounter it again we point to the correct normals */
                    normal_state->face_normal_indices[face_index] = norm_index;
                }
                else if (multiple_normals[treated_tri->treated_index[0]].vertex_normal_state == PRC_VERTEX_NORM_NOT_ENCOUNTERED)
                {
                    /* Do the decode and then store it.Ie.
                    this stores the normal.  Question is can faces reference other
                    faces?  I would say no. As this is the first time we have
                    encountered this face.  All the other triangles on this face
                    share this normal.  However, other triangles NOT on this face
                    can reference this normal that we decode here, AND it is possible
                    that the face can have multiple normals! */
                    has_multiple_normals = normal_bin_data[0];
                    is_reversed = normal_bin_data[1];
                    x_is_reversed = normal_bin_data[2];
                    y_is_reversed = normal_bin_data[3];
                    normal_bin_data += 4;
                    normal_state->normal_bin_data_index += 4;

                    DEBUG_LOG("In PRC_VERTEX_NORM_FACE_FIRST_TIME\n");
                    DEBUG_LOG("has_multiple_normals = %d\n", has_multiple_normals);
                    DEBUG_LOG("is_reversed = %d\n", is_reversed);
                    DEBUG_LOG("x_is_reversed = %d\n", x_is_reversed);
                    DEBUG_LOG("y_is_reversed = %d\n", y_is_reversed);

                    code = prc_decode_normal(ctx, data, is_reversed, x_is_reversed,
                        y_is_reversed, treated_tri, normal_state, debug_tess);

                    if (code < 0)
                    {
                        return code;
                    }
                    prc_vec_copy(normal_state->normals_vertex[normal_state->normals_vertex_count - 1],
                        &normal_state->normal, 0);
                    normal_state->face_normal_decoded[face_index] = 1;

                    normal_state->face_normals[face_index] = normal_state->normal;

                    /* Set the face normal indice location so we can reference it
                       the next time we run into this face */
					normal_state->face_normal_indices[face_index] = normal_state->normals_vertex_count - 1;

                    /* Also add this to the multiple_normals structure */
                    vertex_index = treated_tri->treated_index[0];
                    if (has_multiple_normals)
                    {
                        multiple_normals[vertex_index].vertex_normal_state = PRC_VERTEX_NORM_IS_MULTIPLE;
                        multiple_normals[vertex_index].num_already_stored_normals_on_vertex = 1;
                        multiple_normals[vertex_index].normal_indices =
                            (uint32_t*)prc_calloc(ctx, PRC_INITIAL_MULTI_NORMAL_CAPACITY, sizeof(uint32_t));
                        if (multiple_normals[vertex_index].normal_indices == NULL)
                        {
                            return PRC_ERROR_MEMORY;
                        }
                        multiple_normals[vertex_index].normal_indices[0] = normal_state->normals_vertex_count - 1;
                        multiple_normals[vertex_index].normal_indice_capacity = PRC_INITIAL_MULTI_NORMAL_CAPACITY;
                    }
                    else
                    {
                        multiple_normals[vertex_index].vertex_normal_state = PRC_VERTEX_NORM_IS_NOT_MULTIPLE;
                        multiple_normals[vertex_index].non_multiple_normal_index = normal_state->normals_vertex_count - 1;
                    }

                    /* Set the normal indices for the triangle to what we just decoded */
                    treated_tri->normal_indices[0] = normal_state->normals_vertex_count - 1;
                    treated_tri->normal_indices[1] = normal_state->normals_vertex_count - 1;
                    treated_tri->normal_indices[2] = normal_state->normals_vertex_count - 1;
                }
                else
                {
                    /* In this case, the first vertex of the first triangle on
                       face must have multiple normals we are either going to
                       reference an existing one, or add one to this vertex.
                       Regardless, that normal is used for the entire face */
                    is_a_reference = normal_bin_data[0];
                    normal_bin_data += 1;
                    vertex_index = treated_tri->treated_index[0];
                    normal_state->normal_bin_data_index += 1;
                    if (is_a_reference)
                    {
                        uint32_t num_stored_normals = multiple_normals[vertex_index].num_already_stored_normals_on_vertex;
                        uint32_t number_bits = get_number_bits_to_store_unsigned_integer2(num_stored_normals - 1);
                        uint32_t read_index = 0;
                        uint32_t ref_index;
                        uint32_t existing_normal_index;

                        DEBUG_LOG("In PRC_VERTEX_NORM_IS_MULTIPLE Ref case FACE Case\n");
                        for (int bit_index = 0; bit_index < number_bits; bit_index++)
                        {
                            read_index |= (normal_bin_data[bit_index] << bit_index);
                        }
                        /* max ref index minus what we read.  We need a check here
                        * for overflow */
                        if (read_index >= num_stored_normals)
                        {
                            ref_index = 0;
                            DEBUG_LOG("ERROR: read_index >= num_stored_normals\n");
                        }
                        else
                        {
                            ref_index = (num_stored_normals - 1) - read_index;
                        }
                        DEBUG_LOG("ref_index decoded as %d\n", ref_index);

                        existing_normal_index =
                            multiple_normals[vertex_index].normal_indices[ref_index];

                        /* Set the normal indices for the triangle */
                        treated_tri->normal_indices[0] = existing_normal_index;
                        treated_tri->normal_indices[1] = existing_normal_index;
                        treated_tri->normal_indices[2] = existing_normal_index;

                        normal_state->face_normal_decoded[face_index] = 1;
						normal_state->face_normals[face_index] =
                            normal_state->normals_vertex[existing_normal_index];
                        normal_state->face_normal_indices[face_index] = existing_normal_index;

                        normal_bin_data += number_bits;
                        normal_state->normal_bin_data_index += number_bits;
                        normal_state->normal = normal_state->face_normals[face_index];
                    }
                    else
                    {
                        /* Decode the normal */
                        DEBUG_LOG("In PRC_VERTEX_NORM_IS_MULTIPLE non-ref case FACE Case\n");
                        is_reversed = normal_bin_data[0];
                        x_is_reversed = normal_bin_data[1];
                        y_is_reversed = normal_bin_data[2];
                        normal_bin_data += 3;
                        normal_state->normal_bin_data_index += 3;
                        DEBUG_LOG("is_reversed = %d\n", is_reversed);
                        DEBUG_LOG("x_is_reversed = %d\n", x_is_reversed);
                        DEBUG_LOG("y_is_reversed = %d\n", y_is_reversed);

                        /* Decode the normal (and store the normal) */
                        code = prc_decode_normal(ctx, data, is_reversed, x_is_reversed,
                            y_is_reversed, treated_tri, normal_state, debug_tess);
                        if (code < 0)
                        {
                            return code;
                        }

                        /* Store the normal index.  I assume we store it to this
                           vertex, even though we are dealing with a face. */
                        if (multiple_normals[vertex_index].num_already_stored_normals_on_vertex >=
                            multiple_normals[vertex_index].normal_indice_capacity)
                        {
                            uint32_t *new_normal_indices = (uint32_t *)prc_realloc(ctx,
                                multiple_normals[vertex_index].normal_indices,
                                multiple_normals[vertex_index].normal_indice_capacity * 2 * sizeof(uint32_t));
                            if (new_normal_indices == NULL)
                            {
                                return PRC_ERROR_MEMORY;
                            }
                            multiple_normals[vertex_index].normal_indices = new_normal_indices;
                            multiple_normals[vertex_index].normal_indice_capacity *= 2;
                        }
                        multiple_normals[vertex_index].normal_indices[
                            multiple_normals[vertex_index].num_already_stored_normals_on_vertex] =
                                normal_state->normals_vertex_count - 1;
                            multiple_normals[vertex_index].num_already_stored_normals_on_vertex += 1;

                        /* Set the treated triangle normal indices */
                        treated_tri->normal_indices[0] = normal_state->normals_vertex_count - 1;
                        treated_tri->normal_indices[1] = normal_state->normals_vertex_count - 1;
                        treated_tri->normal_indices[2] = normal_state->normals_vertex_count - 1;

                        normal_state->face_normal_decoded[face_index] = 1;
                        normal_state->face_normals[face_index] =
                            normal_state->normals_vertex[normal_state->normals_vertex_count - 1];
                        normal_state->face_normal_indices[face_index] =
                            normal_state->normals_vertex_count - 1;
                        normal_state->normal = normal_state->face_normals[face_index];
                    }
                }
            }

            /* Calculate if the triangle is reversed */
            prc_is_normal_reversed(ctx, treated_tri, normal_state->normal);

            /* Calculate if face is reversed (uses per-vertex mode based on 3x
               normal average */
            //prc_is_face_reversed(ctx, treated_tri, normal_state->normal);

            DEBUG_LOG("Normal is reversed = %d\n", treated_tri->normal_was_reversed);
            treated_tri->face_was_reversed = 0; /* This is really set in the above value */
        }
        else
        {
            /* Every vertex has an encoded normal.  However, some vertices may not
               have multiple normals and may reference the prior computed normal.
               The logic is all stored in bin normal_bin_data array. If we are at
               a brand new vertex that we have not encoded before, the first bit
               is the has_multiple_normal bit. This is followed by triangle_normal_reversed
               x_is_reversed and y_is_reversed. If we are at a previously decoded
               index, then we need to look at the previous index and see if it had
               the has_multiple_normal bit set.  If it did then the next bit is
               has_multiple_normal and the next three bits are like what we have above.
               If the previous index did NOT have has_multiple_normals set then
               the next bit is a is_a_reference bit.  If is_a_reference is 0 then
               we have the same three bits stored. If instead is_a_reference is 1
               then the index number to the normal is pickled in the normal_bit_data
               and we get the normal for this vertex from there. */
            for (k = 0; k < 3; k++)
            {
                vertex_index = treated_tri->treated_index[k];
                if (multiple_normals[vertex_index].vertex_normal_state ==
                                            PRC_VERTEX_NORM_NOT_ENCOUNTERED)
                {
                    /* We have not processed this normal yet in the bin data
                       lets do that now */
                    has_multiple_normals = normal_bin_data[0];
                    is_reversed = normal_bin_data[1];
                    x_is_reversed = normal_bin_data[2];
                    y_is_reversed = normal_bin_data[3];
                    normal_bin_data += 4;
                    normal_state->normal_bin_data_index += 4;

                    DEBUG_LOG("In PRC_VERTEX_NORM_NOT_ENCOUNTERED\n");
                    DEBUG_LOG("has_multiple_normals = %d\n", has_multiple_normals);
                    DEBUG_LOG("is_reversed = %d\n", is_reversed);
                    DEBUG_LOG("x_is_reversed = %d\n", x_is_reversed);
                    DEBUG_LOG("y_is_reversed = %d\n", y_is_reversed);

                    if (has_multiple_normals)
                    {
                        multiple_normals[vertex_index].vertex_normal_state =
                            PRC_VERTEX_NORM_IS_MULTIPLE;
                        multiple_normals[vertex_index].num_already_stored_normals_on_vertex = 1;

                        multiple_normals[vertex_index].normal_indices =
                            (uint32_t *)prc_calloc(ctx,
                            PRC_INITIAL_MULTI_NORMAL_CAPACITY, sizeof(uint32_t));
                        if (multiple_normals[vertex_index].normal_indices == NULL)
                        {
                            return PRC_ERROR_MEMORY;
                        }
                        multiple_normals[vertex_index].normal_indices[0] =
                            normal_state->normals_vertex_count;
                        multiple_normals[vertex_index].normal_indice_capacity =
                            PRC_INITIAL_MULTI_NORMAL_CAPACITY;
                    }
                    else
                    {
                        multiple_normals[vertex_index].vertex_normal_state =
                            PRC_VERTEX_NORM_IS_NOT_MULTIPLE;
                        multiple_normals[vertex_index].non_multiple_normal_index =
                            normal_state->normals_vertex_count;
                    }

                    /* Set the treated triangles normal indice */
                    treated_tri->normal_indices[k] =
                        normal_state->normals_vertex_count;

                    /* Decode the normal (and store the normal) */
                    code = prc_decode_normal(ctx, data, is_reversed, x_is_reversed,
                                             y_is_reversed, treated_tri, normal_state,
                                             debug_tess);
                    if (code < 0)
                    {
                        return code;
                    }
                }
                /* We have already encountered this one. We have three possibile
                   choices. If this vertex had multiple normals then either we
                   are adding a new one OR we are referencing ONE of the the
                   previously decoded normals for this vertex. IF instead this
                   vertex did NOT have multiple normals then we just use the
                   normal index we previously used for this vertex */
                else if (multiple_normals[vertex_index].vertex_normal_state ==
                            PRC_VERTEX_NORM_IS_NOT_MULTIPLE)
                {
                    treated_tri->normal_indices[k] =
                        multiple_normals[vertex_index].non_multiple_normal_index;
                }
                else
                {
                    /* This vertex has multiple normals. We need to check if
                       we are adding a new one or referencing a previous one */
                    is_a_reference = normal_bin_data[0];
                    normal_bin_data += 1;
                    normal_state->normal_bin_data_index += 1;
                    if (is_a_reference)
                    {
                        uint32_t num_stored_normals = multiple_normals[vertex_index].num_already_stored_normals_on_vertex;
                        uint32_t number_bits = get_number_bits_to_store_unsigned_integer2(num_stored_normals - 1);
                        uint32_t read_index = 0;
                        uint32_t ref_index;
                        uint32_t existing_normal_index;

                        DEBUG_LOG("In PRC_VERTEX_NORM_IS_MULTIPLE Ref case\n");
                        for (int bit_index = 0; bit_index < number_bits; bit_index++)
                        {
                            read_index |= (normal_bin_data[bit_index] << bit_index);
                        }
                        /* max ref index minus what we read.  We need a check here
						* for overflow */
						if (read_index >= num_stored_normals)
						{
							ref_index = 0;
							DEBUG_LOG("ERROR: read_index >= num_stored_normals\n");
						}
                        else
                        {
                            ref_index = (num_stored_normals - 1) - read_index;
                        }
                        DEBUG_LOG("ref_index decoded as %d\n", ref_index);

                        existing_normal_index =
                            multiple_normals[vertex_index].normal_indices[ref_index];
                        treated_tri->normal_indices[k] = existing_normal_index;
                        normal_bin_data += number_bits;
                        normal_state->normal_bin_data_index += number_bits;
                    }
                    else
                    {
                        /* Decode the normal */
                        DEBUG_LOG("In PRC_VERTEX_NORM_IS_MULTIPLE non-ref case\n");
                        is_reversed = normal_bin_data[0];
                        x_is_reversed = normal_bin_data[1];
                        y_is_reversed = normal_bin_data[2];
                        normal_bin_data += 3;
                        normal_state->normal_bin_data_index += 3;
                        DEBUG_LOG("is_reversed = %d\n", is_reversed);
                        DEBUG_LOG("x_is_reversed = %d\n", x_is_reversed);
                        DEBUG_LOG("y_is_reversed = %d\n", y_is_reversed);

                        /* Decode the normal (and store the normal) */
                        code = prc_decode_normal(ctx, data, is_reversed, x_is_reversed,
                                                 y_is_reversed, treated_tri, normal_state,
                                                 debug_tess);
                        if (code < 0)
                        {
                            return code;
                        }

                        /* Store the normal index */
                        if (multiple_normals[vertex_index].num_already_stored_normals_on_vertex >=
                            multiple_normals[vertex_index].normal_indice_capacity)
                        {
                            uint32_t *new_normal_indices = (uint32_t *)prc_realloc(ctx,
                                multiple_normals[vertex_index].normal_indices,
                                multiple_normals[vertex_index].normal_indice_capacity * 2 * sizeof(uint32_t));
                            if (new_normal_indices == NULL)
                            {
                                return PRC_ERROR_MEMORY;
                            }
                            multiple_normals[vertex_index].normal_indices = new_normal_indices;
                            multiple_normals[vertex_index].normal_indice_capacity *= 2;
                        }
                        multiple_normals[vertex_index].normal_indices[
                            multiple_normals[vertex_index].num_already_stored_normals_on_vertex] =
                            normal_state->normals_vertex_count - 1;
                        multiple_normals[vertex_index].num_already_stored_normals_on_vertex += 1;

                        /* Set the treated triangles normal indice */
                        treated_tri->normal_indices[k] = normal_state->normals_vertex_count - 1;
                    }
                }
            }
            /* Calculate if the triangle is reversed based on the normal with
               the lowest index */
            prc_is_normal_reversed_single_normal(ctx, treated_tri, normal_state);

            /* Also calculate if the face is reversed based upon 3x averaging */
            prc_is_face_reversed_average(ctx, treated_tri, normal_state);
        }
    }
    return 0;
}

static void
prc_initialize_normal_state(prc_context *ctx, prc_tess_3d_compressed *data,
    prc_normal_state *normal_state, prc_vec3 *normals_vertex, double *decoded_angles,
    uint8_t *face_normal_decoded, prc_vec3 *face_normals,
    decoded_normal_info *multiple_norms, uint32_t *face_normal_indices)
{
    normal_state->must_calculate_normals = data->must_recalculate_normals;
    normal_state->normals_vertex_count = 0;
    normal_state->normal_bin_data_index = 0;
    normal_state->normal_bin_data = data->normal_binary_data;
    normal_state->angle_index = 0;
    normal_state->face_is_planar = prc_face_is_planar(ctx, data, 0);
    normal_state->decoded_angles = decoded_angles;
    normal_state->face_normal_decoded = face_normal_decoded;
    normal_state->face_normals = face_normals;
	normal_state->face_normal_indices = face_normal_indices;
    normal_state->normals_vertex = normals_vertex;
    normal_state->normal_is_reversed = data->normal_is_reversed;
    normal_state->multiple_normals = multiple_norms;
    normal_state->actual_normals = NULL;
}

static int
prc_add_edge_to_list(prc_context *ctx, treated_edge_list *edge_list,
    const comp_edge *edge_in)
{
	uint32_t min_index, max_index;

	if (edge_in->edge_treatement_x < edge_in->edge_treatement_y)
	{
		min_index = edge_in->edge_treatement_x;
		max_index = edge_in->edge_treatement_y;
	}
	else
	{
		min_index = edge_in->edge_treatement_y;
		max_index = edge_in->edge_treatement_x;
	}

	if (edge_list->capacity <= min_index)
	{
		uint32_t old_capacity = edge_list->capacity;
		treated_edge *new_edge_list = (treated_edge *)prc_realloc(ctx,
            edge_list->edge, min_index * 2 * sizeof(treated_edge));
		if (new_edge_list == NULL)
		{
			return PRC_ERROR_MEMORY;
		}
		edge_list->edge = new_edge_list;
		edge_list->capacity = min_index * 2;

		/* Initialize the new edge list */
		for (int k = old_capacity; k < edge_list->capacity; k++)
		{
			edge_list->edge[k].num_second_indices = 0;
			edge_list->edge[k].capacity = 0;
			edge_list->edge[k].indice1 = NULL;
		}
	}
    if (edge_list->edge[min_index].capacity == 0)
    {
        edge_list->edge[min_index].indice1 = (uint32_t *)prc_calloc(ctx,
            PRC_INITIAL_TREATED_EDGE_LIST_CAPACITY, sizeof(uint32_t));
        if (edge_list->edge[min_index].indice1 == NULL)
        {
            return PRC_ERROR_MEMORY;
        }
        edge_list->edge[min_index].capacity = PRC_INITIAL_TREATED_EDGE_LIST_CAPACITY;
    }
	else if (edge_list->edge[min_index].num_second_indices >= edge_list->edge[min_index].capacity)
	{
        uint32_t old_capacity = edge_list->edge[min_index].capacity;
        uint32_t new_capacity;
        uint32_t *new_indices;

        if (old_capacity > UINT32_MAX / 2)
        {
            return PRC_ERROR_MEMORY;
        }

        new_capacity = old_capacity * 2;
        new_indices = (uint32_t *)prc_calloc(ctx, new_capacity, sizeof(uint32_t));
		if (new_indices == NULL)
		{
			return PRC_ERROR_MEMORY;
		}

        memcpy(new_indices,
            edge_list->edge[min_index].indice1,
            old_capacity * sizeof(uint32_t));
        prc_free(ctx, edge_list->edge[min_index].indice1);

		edge_list->edge[min_index].indice1 = new_indices;
        edge_list->edge[min_index].capacity = new_capacity;
	}
	edge_list->edge[min_index].indice1[edge_list->edge[min_index].num_second_indices] = max_index;
	edge_list->edge[min_index].num_second_indices += 1;
	edge_list->edge[min_index].indice0 = min_index;  /* Not really needed but here for debug help */

    return 0;
}

static int
prc_add_edges_to_list(prc_context *ctx, treated_edge_list *edge_list,
    const treated_triangle *tri_in)
{
	if (tri_in->left_edge.edge_status == PRC_EDGE_TREATED)
	{
		int code = prc_add_edge_to_list(ctx, edge_list, &tri_in->left_edge);
		if (code < 0)
		{
			return code;
		}
	}
	if (tri_in->right_edge.edge_status == PRC_EDGE_TREATED)
	{
		int code = prc_add_edge_to_list(ctx, edge_list, &tri_in->right_edge);
		if (code < 0)
		{
			return code;
		}
	}
    return 0;
}

/* Search through the edge list for the existence of an edge */
static void
prc_check_for_treated_edge(prc_context *ctx, treated_edge_list *edge_list,
    prc_treated_details *treated_details, uint8_t *edge_found)
{
    uint32_t min_index, max_index;

    if (treated_details->index0 < treated_details->index1)
    {
		min_index = treated_details->index0;
		max_index = treated_details->index1;
	}
    else
    {
        min_index = treated_details->index1;
        max_index = treated_details->index0;
    }

    if (min_index >= edge_list->capacity)
    {
        *edge_found = 0;
        return;
    }

	if (edge_list->edge[min_index].num_second_indices == 0)
	{
		*edge_found = 0;
		return;
	}

	for (int k = 0; k < edge_list->edge[min_index].num_second_indices; k++)
	{
		if (edge_list->edge[min_index].indice1[k] == max_index)
		{
			*edge_found = 1;
			return;
		}
	}
	*edge_found = 0;
}

static void
prc_store_triangle_style(prc_context *ctx, prc_tess_3d_compressed *data,
    uint32_t tri_num, uint32_t *face_styles, uint8_t *face_encountered,
    uint32_t *triangle_style_array, uint32_t *style_index)
{
	uint32_t face_num = data->triangle_face_array[tri_num];
    uint8_t is_multiple_style = 0;

	if (data->is_multiple_line_attribute)
	{
		is_multiple_style = data->is_multiple_line_attribute_on_face[face_num];
	}

	if (is_multiple_style)
	{
        /* In this case for this particular face, each triangle style is stored
           in the line_attributes_array */
		triangle_style_array[tri_num] = data->line_attribute_array[*style_index];
		*style_index += 1;
	}
    else if (data->line_attribute_array_size == 0)
    {
        /* No per-triangle/per-face style data recorded at all (e.g. a
           writer, like this codebase's own, that relies entirely on the
           owning representation item/part's style reference rather than
           per-triangle styles within the tessellation itself).
           line_attribute_array is NULL/empty in this case -- 0 is this
           level's existing "no style" sentinel (see the unbiasing check a
           few hundred lines below: triangle_style_array[k] == 0 is left
           as 0 rather than decremented), so use it instead of
           dereferencing a NULL array. */
        triangle_style_array[tri_num] = 0;
    }
    else
    {
        if (data->line_attribute_array_size == 1)
        {
            triangle_style_array[tri_num] = data->line_attribute_array[0];
        }
        else
        {
            /* First see if we have encountered this face before. Then we just set
               this triangles style to what the face is set to */
            if (!face_encountered[face_num])
            {
                face_styles[face_num] = data->line_attribute_array[*style_index];
                face_encountered[face_num] = 1;
                *style_index += 1;
            }
            triangle_style_array[tri_num] = face_styles[face_num];
        }
	}
}

/* This handles all the cases. With and without faces, calculating normals or
   just decoding normals */
int
prc_decode_compressed_tess(prc_context *ctx, prc_tess_3d_compressed *data, uint8_t debug_tess)
{
    int k;
    prc_vec3 *point_array_scaled;
    int num_triangles;
    uint32_t *triangle_indices;
    int num_points = data->point_array_size / 3;
    prc_vec3 *normals_vertex;
    uint32_t point_array_count = 0;
    int reference_array_count = 0;
    treated_triangle treated_tri;
    int points_is_reference_index = 3;
    int vertex_treatment_count = 0;
    int triangle_indice_count = 0;
    int code;
    prc_treated_details treated_details;
    prc_triangle_stack stack;
    prc_vec3 new_point;
    prc_vec3 *vertices_out;
    uint32_t *vertex_normal_indices;
    int vertex_count;
    int current_bits_zero[4] = {0, 0, 0, 0};
    int new_normal_index;
    int number_of_normals;
    int edge_count;
    int new_indice_index;
    prc_normal_state normal_state;
    decoded_normal_info *multiple_normals = NULL;
    uint32_t multiple_normals_size = 0;
    treated_edge_list edge_list;
	int num_edges = 0;
    uint8_t stack_was_empty = 0;
    uint32_t *face_styles = NULL;
	uint8_t *face_encountered = NULL;
	uint32_t *triangle_style_array = NULL;
    uint32_t style_index = 0;

    /* Initially set the edge_list to be of size that is half the number of
       points since we will be referencing by the smallest index */

	num_edges = num_points / 2;
	edge_list.edge = (treated_edge *)prc_calloc(ctx, num_edges, sizeof(treated_edge));
	if (edge_list.edge == NULL)
		return PRC_ERROR_MEMORY;
	edge_list.capacity = num_edges;

    /* Items needed if the normals are compressed in the stream and do
       not need to be calculated. We still need to do calculations, just
       different ones... */
    int must_calculate_normals = data->must_recalculate_normals;
    uint32_t normal_angle_array_size = data->normal_angle_array_size;
    int normal_bin_data_index = 0;
    double *decoded_angles = NULL;

    /* Items needed to deal with case where we have planar faces. In this
       case, we have one normal per face */
    uint8_t has_faces = data->has_faces;
    uint32_t number_faces = data->face_number;
    uint32_t triangle_face_array_size = data->triangle_face_array_size;
    uint32_t *triangle_face_array = data->triangle_face_array;
    uint8_t *face_normal_decoded = NULL;
    prc_vec3 *face_normals = NULL;
	uint32_t *face_normal_indices = NULL;
    uint8_t face_is_planar = 0;
    uint8_t previous_face_normal = 0;
    uint8_t at_least_one_face_not_planar = 0;
    uint8_t at_least_one_face_is_planar = 0;

    /* The line attribute array must have at least as many entries as there are
       faces. When we encounter the first first triangle of a face
       that is when we will get the style for that face from the style.  We also
       need to consider the case where each triangle has its own style. This
       can occur if we have is_multiple_attribute TRUE.  If that master boolean
       is true, then when we encounter a face, we need to look
       at the setting of the boolean value for the face which is set in
       is_multiple_line_attribute_on_face.  If that is false then we just have
       a single global style for the face. If that is true, then each triangle
       in the face has its style added to line_attribute array. Note that face
       triangles need not be stored in one complete group.  That is we can
       have a face array (which defines the face number for the triangles we are
       decoding) which is 2 2 2 7 7 7 6 6 2 2 7 7 6 9  for example.  So we need
       to know if the one we are working on is per triangle or per face so we
       will know to get the style from the line_attribute array or from what we
       had previously read for that face. Regardless, we store a style for every
       single triangle */
    if (has_faces)
    {
        face_styles = (uint32_t *)prc_calloc(ctx, number_faces, sizeof(uint32_t));
        if (face_styles == NULL)
            return PRC_ERROR_MEMORY;

		triangle_style_array = (uint32_t *)prc_calloc(ctx,
            data->triangle_face_array_size, sizeof(uint32_t));
		if (triangle_style_array == NULL)
			return PRC_ERROR_MEMORY;

		face_encountered = (uint8_t *)prc_calloc(ctx, number_faces, sizeof(uint8_t));
		if (face_encountered == NULL)
			return PRC_ERROR_MEMORY;
    }

    /* Can we have a case where we have one normal per face AND we
       have to calculate the normals? Throw an error for now for that case. There
       could be some complexity here where we have multiple faces and some are
       planar and some are not. */
    if (has_faces && data->is_face_planar != NULL && data->triangle_face_array != NULL &&
        data->is_face_planar[data->triangle_face_array[0]] && must_calculate_normals)
    {
        return PRC_ERROR_PARSE;
    }

    if (!must_calculate_normals)
    {
        decoded_angles = (double *)prc_calloc(ctx, sizeof(double), data->normal_angle_array_size);
        if (decoded_angles == NULL)
            return PRC_ERROR_MEMORY;
        prc_decode_angles(ctx, data, decoded_angles);
    }

    stack.details = NULL;
    stack.size = 0;

    /* Allocate the vertices */
    point_array_scaled = (prc_vec3 *)prc_calloc(ctx, sizeof(prc_vec3), num_points);
    if (point_array_scaled == NULL)
        return PRC_ERROR_MEMORY;

    /* Allocate the indices for the triangles.  Number of triangles is the size of
       the triangle_face_array */
    num_triangles = data->triangle_face_array_size;
    triangle_indices = (uint32_t *)prc_calloc(ctx, sizeof(uint32_t), num_triangles * 3);
    if (triangle_indices == NULL)
        return PRC_ERROR_MEMORY;

    /* Allocate a normal index for each of the triangle vertices (num_triangles * 3) */
    vertex_normal_indices = (uint32_t *)prc_calloc(ctx, sizeof(uint32_t), num_triangles * 3);
    if (vertex_normal_indices == NULL)
        return PRC_ERROR_MEMORY;

    /* output vertices */
    vertices_out = (prc_vec3 *)prc_calloc(ctx, sizeof(prc_vec3), num_points);
    if (vertices_out == NULL)
        return PRC_ERROR_MEMORY;

    /* The normal vectors that we index. Number of angles divided by 2 */
    /* Or if we have to compute them, it is the number of triangles times three. */
    normals_vertex = (prc_vec3 *)prc_calloc(ctx, sizeof(prc_vec3), num_triangles * 3);
    if (normals_vertex == NULL)
        return PRC_ERROR_MEMORY;

    /* multiple_norms is to keep track of info about calculated normals. Mainly about
       if a vertex has multiple normals or not. If it does then when we are decoding
       the normal we read more data. If it does not we use the already computed normal
       for that vertex. In tri-mesh traversal we only consume the next point but
       if we are doing a 2 refs and 1 decoded or 1 ref and 2 decoded etc we may or
       may not consume data for the refs. That is determined by the has_multiple_normals
       bit for those ref points.  A wrinkle in all of this is that in the tessellation
       we can have multiple faces and different triangles go into different faces.
       And some faces could be planar and others not planar. fun!.
       Right now we have data->is_face_planar[k] where k is the face number which
       tells us if this face is planar. We can index into this based upon
       data->triangle_face_array which given a triangle index gives us an index
       into data->is_face_planar */

    /* If any of the faces are not planar and we are not recalculating the normals
       then allocate the multiple_norms array so that we can track which vertices
       have multiple norms and we know when to decode */

#if 0
    if (!data->must_recalculate_normals && data->has_faces)
    {
        for (k = 0; k < data->face_number; k++)
        {
            if (!data->is_face_planar[k])
            {
                at_least_one_face_not_planar = 1;
                break;
            }
        }
    }

    if (at_least_one_face_not_planar)
    {
        multiple_normals = (decoded_normal_info *)prc_calloc(ctx, num_triangles * 3,
                                                        sizeof(decoded_normal_info));
        if (multiple_normals == NULL)
            return PRC_ERROR_MEMORY;
        multiple_normals_size = num_triangles * 3;
    }

    if (!data->must_recalculate_normals && multiple_normals == NULL)
    {
        /* In this case, we are decoding the normals for each face, but we could
           possibily have binary_data that crazily encodes multiple normals
           so we need to plan for that */
        multiple_normals = (decoded_normal_info *)prc_calloc(ctx, number_faces,
            sizeof(decoded_normal_info));
        if (multiple_normals == NULL)
            return PRC_ERROR_MEMORY;
        multiple_normals_size = number_faces;
    }
#endif

    if (!data->must_recalculate_normals)
    {
        multiple_normals = (decoded_normal_info *)prc_calloc(ctx, num_triangles * 3,
            sizeof(decoded_normal_info));
        if (multiple_normals == NULL)
            return PRC_ERROR_MEMORY;
        multiple_normals_size = num_triangles * 3;
    }

    if (!data->must_recalculate_normals && data->has_faces)
    {
        for (k = 0; k < data->face_number; k++)
        {
            if (data->is_face_planar[k])
            {
                at_least_one_face_is_planar = 1;
                break;
            }
        }
    }

    /* If we have faces, we will decode as we come to each face and we need to keep
       track of which ones have been decoded. Note that we don't have to allocate
       this if NONE of the faces are planar */
    if (at_least_one_face_is_planar)
    {
        face_normal_decoded = (uint8_t *)prc_calloc(ctx, sizeof(uint8_t), number_faces);
        if (face_normal_decoded == NULL)
            return PRC_ERROR_MEMORY;

        face_normals = (prc_vec3 *)prc_calloc(ctx, sizeof(prc_vec3), number_faces);
        if (face_normals == NULL)
            return PRC_ERROR_MEMORY;

		face_normal_indices = (uint32_t*)prc_calloc(ctx, sizeof(uint32_t), number_faces);
		if (face_normal_indices == NULL)
			return PRC_ERROR_MEMORY;
    }

#if VERTEX_DEBUG

    /* Load list of vertices that we know exist for this */
    prc_vec3 *actual_vertices = NULL;
    prc_vec3 *actual_normals = NULL;
    uint32_t *actual_indices = NULL;
    FILE *fid = NULL;
    FILE *fid2 = NULL;

    if (debug_tess)
    {
        code = debug_prc_read_vertices_from_JSON_file(ctx, "Add file here", &actual_vertices, num_points);
        if (code < 0)
        {
            return code;
        }

        code = debug_prc_read_vertices_from_JSON_file(ctx, "Add file here", &actual_normals, 1857);
        if (code < 0)
        {
            return code;
        }

        /* Open the file */
        fid = fopen("vertex_difference_CARB.txt", "w");
        if (fid == NULL)
        {
            prc_error(ctx, PRC_ERROR_FILE, "Failed to open vertex difference file\n");
            return PRC_ERROR_FILE;
        }
    }
#endif

    /* Go ahead and scale all the compressed values by the tolerance. */
    prc_scale_data_points(ctx, data, point_array_scaled);

    /* Compute the vertices of the first triangle. */
    prc_compute_first_triangle(ctx, data, point_array_scaled,
                                    &point_array_count, &treated_tri);

    /* Store the vertices of the first triangle */
    prc_store_vertices(ctx, treated_tri, vertices_out, &vertex_treatment_count);

    /* Initialize the normal structure */
    prc_initialize_normal_state(ctx, data, &normal_state, normals_vertex, decoded_angles,
        face_normal_decoded, face_normals, multiple_normals, face_normal_indices);

#if VERTEX_DEBUG
    if (debug_tess)
    {
        if (actual_normals != NULL)
        {
            normal_state.actual_normals = actual_normals;
        }
    }
#endif

    /* Deal with the normal calculation for all the various cases */
    code = prc_handle_normal_calculation(ctx, data, &treated_tri, vertex_normal_indices,
                                             0,  &normal_state, debug_tess);
    if (code < 0)
    {
        return code;
    }

    /* Store the triangle vertex indices and the normal indices (should be 0, 1, 2) */
    prc_store_triangle_indices(ctx, &treated_tri, triangle_indices,
                                    vertex_normal_indices, &triangle_indice_count);

	if (has_faces)
	{
		/* Store the face style for the first triangle */
		prc_store_triangle_style(ctx, data, 0, face_styles, face_encountered,
            triangle_style_array, &style_index);
	}

    /* Start at triangle 2 since we already computed one */
    for (k = 1; k < num_triangles; k++)
    {
         /* k - 1 since we are looking at the previous treated triangle */
         edge_count = prc_set_left_right_edge_indices(ctx, data, k - 1, &treated_tri,
                                    &treated_details, normals_vertex);

        /* Any treated edges must be added to the edge_list structure */
        if (edge_count < 2)
        {
            code = prc_add_edges_to_list(ctx, &edge_list, &treated_tri);
            if (code < 0)
            {
                return code;
            }
        }

        DEBUG_LOG("\nTriangle %d\n", k - 1);
        DEBUG_LOG("Normal Reversed = %d\n", treated_tri.normal_was_reversed);
        DEBUG_LOG("V0 = [%.17lf %.17lf %.17lf]\n", treated_tri.points[0].x,
                                treated_tri.points[0].y, treated_tri.points[0].z);
        DEBUG_LOG("V1 = [%.17lf %.17lf %.17lf]\n", treated_tri.points[1].x,
                                treated_tri.points[1].y, treated_tri.points[1].z);
        DEBUG_LOG("V2 = [%.17lf %.17lf %.17lf]\n", treated_tri.points[2].x,
                                treated_tri.points[2].y, treated_tri.points[2].z);
        DEBUG_LOG("R Edge = [%d %d]   LeftEdge = [%d %d]\n",
                                treated_tri.right_edge.edge_treatement_x,
            treated_tri.right_edge.edge_treatement_y, treated_tri.left_edge.edge_treatement_x,
            treated_tri.left_edge.edge_treatement_y);

        DEBUG_LOG("LAST THREE INDICES = %d %d %d\n", triangle_indices[triangle_indice_count - 3],
            triangle_indices[triangle_indice_count - 2], triangle_indices[triangle_indice_count - 1]);

        if (edge_count > 0)
        {
            /* We have at least one edge. Set up the treatement details and push
             * each of these onto the stack. The right is first pushed on the stack
             * and then the left is pushed on the stack. This is different than
             * described in the spec. Unless their stack is not a LIFO stack */
            if (treated_tri.right_edge.edge_status == PRC_EDGE_NOT_TREATED)
            {
                code = prc_compute_triangle_basis(ctx, vertices_out, &treated_tri,
                                                  &treated_details, PRC_EDGE_RIGHT);
                if (code < 0)
                {
                    return code;
                }
                code = push_stack(ctx, &stack, &treated_details);
                if (code < 0)
                {
                    return code;
                }
            }

            if (treated_tri.left_edge.edge_status == PRC_EDGE_NOT_TREATED)
            {
                code = prc_compute_triangle_basis(ctx, vertices_out, &treated_tri,
                                                  &treated_details, PRC_EDGE_LEFT);
                if (code < 0)
                {
                    return code;
                }
                code = push_stack(ctx, &stack, &treated_details);
                if (code < 0)
                {
                    return code;
                }
            }
        }

        DEBUG_LOG("Stack size is %d\n", stack.size);

        stack_was_empty = 0;
        if (prc_stack_empty(ctx, &stack))
        {
            /* We must have a situation where we have one of the following situations
            1) A single reference point and two decoded points.
            2) Two reference points and one decoded point.
            3) Three reference points.

            In any event we need to look at the reference point array and determine
            which of the three we have to deal with */

            /* We may need to revisit this and its interaction with the code below it */
            code = prc_handle_empty_stack_decode(ctx, data, point_array_scaled,
                    vertices_out, &treated_tri, &point_array_count,
                    &reference_array_count, &vertex_treatment_count,
                    &points_is_reference_index);
            if (code < 0)
            {
                return code;
            }
			stack_was_empty = 1;
        }
        else
        {
            /* With that done, lets now compute the next triangle.
               First pop the current one from the stack */
            code = pop_stack(ctx, &stack, &treated_details);
            if (code < 0)
            {
                return code;
            }

            /* If we had no edges pushed onto the stack, then we need to search
               the edge list to make sure that the edge that we just popped
               was not already treated.  This can occur for example if a triangle
               had a left and right neighbor and we got to its right neighbor by
               going left.  Then that original push of the right edge needs to
               be ignored when it is popped from the stack. */
            if (edge_count == 0)
            {
                uint8_t edge_found = 1;

                while (edge_found)
                {
                    prc_check_for_treated_edge(ctx, &edge_list, &treated_details, &edge_found);
                    if (edge_found)
                    {
                        /* We may need to check for an empty stack here */
                        if (prc_stack_empty(ctx, &stack))
                        {
                            stack_was_empty = 1;
                            edge_found = 0; /* Lets get out of this. All edges have been handled */
                            code = prc_handle_empty_stack_decode(ctx, data, point_array_scaled,
                                vertices_out, &treated_tri, &point_array_count,
                                &reference_array_count, &vertex_treatment_count,
                                &points_is_reference_index);
                            if (code < 0)
                            {
                                return code;
                            }
                        }
                        else
                        {
                            /* Edge was found pop the next one */
                            code = pop_stack(ctx, &stack, &treated_details);
                            if (code < 0)
                            {
                                return code;
                            }
                        }
                    }
                }
            }
        }

		if (!stack_was_empty)
		{
            /* points_is_reference_index/reference_array_count are driven by
               independently attacker-controlled counts from the file. Validate
               them and the resulting point_reference_array value before using
               them to index points_is_reference_array/vertices_out, otherwise a
               crafted file causes an out-of-bounds heap read. */
            if (points_is_reference_index < 0 ||
                (uint32_t)points_is_reference_index >= data->reference_array_size)
            {
                prc_error(ctx, PRC_ERROR_PARSE, "points_is_reference_index out of range\n");
                return PRC_ERROR_PARSE;
            }

            /* Only one point is a reference.. */
            if (data->points_is_reference_array[points_is_reference_index] == 1)
            {
                int32_t ref_value;

                if (reference_array_count < 0 ||
                    (uint32_t)reference_array_count >= data->point_reference_array_size)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "reference_array_count out of range\n");
                    return PRC_ERROR_PARSE;
                }
                ref_value = data->point_reference_array[reference_array_count];
                if (ref_value < 0 || (uint32_t)ref_value >= (uint32_t)num_points)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "point_reference_array value out of range\n");
                    return PRC_ERROR_PARSE;
                }

                /* This is a reference point */
                /* Grab the existing point we decoded already. Stick into new_point. */
                prc_vec_copy(vertices_out[ref_value], &new_point, 0);
                new_indice_index = ref_value;

                reference_array_count++;

                /* Order of new triangle is V0, V1, Vnew */
                prc_set_treated_triangle(ctx, &treated_tri, treated_details.V0,
                    treated_details.V1, new_point, treated_details.index0,
                    treated_details.index1, new_indice_index,
                    treated_details.normal_index0, treated_details.normal_index1,
                    0, 0);  /* TODO: Check this  last normal index... */

                DEBUG_LOG("Next Pt (was a reference): %d\n", new_indice_index);
                DEBUG_LOG("    [%.17f %.17f %.17f]\n", new_point.x, new_point.y, new_point.z);
            }
            else
            {
                /* point_array_count/vertex_treatment_count are driven by
                   triangle_face_array_size, a count independent of point_array_size
                   (the actual capacity of point_array/vertices_out). Validate before
                   reading/writing, otherwise a crafted file causes an out-of-bounds
                   heap read (point_array) or write (vertices_out). */
                if (point_array_count >= (uint32_t)num_points ||
                    vertex_treatment_count >= num_points)
                {
                    prc_error(ctx, PRC_ERROR_PARSE, "point_array_count/vertex_treatment_count out of range\n");
                    return PRC_ERROR_PARSE;
                }

                DEBUG_LOG("Origin: [%.17f %.17f %.17f]\n", treated_details.origin.x,
                    treated_details.origin.y, treated_details.origin.z);

                prc_decode_next_point_post_scale(&new_point, treated_details.x_basis, treated_details.y_basis,
                    treated_details.z_basis, treated_details.origin, &data->point_array[3 * point_array_count],
                    data->tolerance);
                DEBUG_LOG("Next Pt: %d\n", vertex_treatment_count);
                DEBUG_LOG("    [%.17f %.17f %.17f]\n", new_point.x, new_point.y, new_point.z);

#if VERTEX_DEBUG
                if (debug_tess)
                {
                    /* Compare the new point to the value in actual_vertices */
                    if (actual_vertices != NULL)
                    {
                        if (fabs(new_point.x - actual_vertices[point_array_count].x) > 0.00000100 ||
                            fabs(new_point.y - actual_vertices[point_array_count].y) > 0.00000100 ||
                            fabs(new_point.z - actual_vertices[point_array_count].z) > 0.00000100)
                        {
                            int zz = 1;
                            DEBUG_LOG("ERROR: Point %d does not match\n", point_array_count);
                            DEBUG_LOG("INDEX was %d\n", k)
                                DEBUG_LOG("    [%1.5f %1.5f %1.5f]\n", new_point.x, new_point.y, new_point.z);
                            DEBUG_LOG("    [%1.5f %1.5f %1.5f]\n", actual_vertices[point_array_count].x,
                                actual_vertices[point_array_count].y, actual_vertices[point_array_count].z);
                        }

                        fprintf(fid, "%d %1.5f %1.5f %1.5f %1.5f %1.5f %1.5f\n", point_array_count,
                            new_point.x, new_point.y, new_point.z, actual_vertices[point_array_count].x,
                            actual_vertices[point_array_count].y, actual_vertices[point_array_count].z);

                        /* Lets use the actual value as our new point */
                        if (k < 0)
                        {
                            new_point.x = actual_vertices[point_array_count].x;
                            new_point.y = actual_vertices[point_array_count].y;
                            new_point.z = actual_vertices[point_array_count].z;
                        }
                    }
                }
#endif

                point_array_count++;
                new_normal_index = vertex_treatment_count;
                new_indice_index = vertex_treatment_count;

                /* Add the new point to the list of vertices */
                vertices_out[vertex_treatment_count] = new_point;
                vertex_treatment_count++;

                /* Order of new triangle is V0, V1, Vnew */
                prc_set_treated_triangle(ctx, &treated_tri, treated_details.V0,
                    treated_details.V1, new_point, treated_details.index0,
                    treated_details.index1, new_indice_index,
                    treated_details.normal_index0, treated_details.normal_index1,
                    new_normal_index, 0);
            }

            /* One point in this case */
            points_is_reference_index++;
        }

        /* With the above decode process, if the normal is reversed we store
           the triangle indices as is. If the normal is not reversed, then we
           swap the middle and final indices. */
        code = prc_handle_normal_calculation(ctx, data, &treated_tri,
                                             vertex_normal_indices, k,
                                             &normal_state, debug_tess);
        if (code < 0)
        {
            return code;
        }

        /* Store the triangle vertex indices and the normal indices */
        prc_store_triangle_indices(ctx, &treated_tri, triangle_indices,
                                        vertex_normal_indices, &triangle_indice_count);

        if (has_faces)
        {
            /* Store the face style for the first triangle */
            prc_store_triangle_style(ctx, data, k, face_styles, face_encountered,
                triangle_style_array, &style_index);
        }

#if VERTEX_DEBUG
        if (debug_tess)
        {
            if (actual_indices != NULL)
            {
                fprintf(fid2, "%d %d %d %d %d %d %d\n", point_array_count,
                    triangle_indices[triangle_indice_count - 3], triangle_indices[triangle_indice_count - 2],
                    triangle_indices[triangle_indice_count - 1], actual_indices[k * 3],
                    actual_indices[k * 3 + 1], actual_indices[k * 3 + 2]);
            }
        }
#endif
    }
#if VERTEX_DEBUG
    if (debug_tess)
    {
        if (fid != NULL)
        {
            fclose(fid);
            // fclose(fid2);
        }
    }
#endif
    number_of_normals = normal_state.normals_vertex_count;

    data->triangle_indices_prc_compressed_3d = triangle_indices;
    data->normal_indices_prc_compressed_3d = vertex_normal_indices;
    data->num_triangle_indices_prc_compressed_3d = num_triangles * 3;
    data->num_normal_indices_prc_compressed_3d = num_triangles * 3;
    data->num_normals_prc_compressed_3d = number_of_normals;
    data->num_vertices_prc_compressed_3d = num_points;

    /* This is the vertex colors if specified */
    if (data->decoded_point_color_array != NULL)
    {
        data->point_colors_prc_compressed_3d = (float *)prc_calloc(ctx, sizeof(float),
            data->decoded_point_color_array_size);
        if (data->point_colors_prc_compressed_3d == NULL)
            return PRC_ERROR_MEMORY;
        memcpy(data->point_colors_prc_compressed_3d, data->decoded_point_color_array,
            data->decoded_point_color_array_size * sizeof(float));
    }

    data->vertices_prc_compressed_3d = (double *)prc_calloc(ctx, sizeof(double), num_points * 3);
    if (data->vertices_prc_compressed_3d == NULL)
        return PRC_ERROR_MEMORY;

    data->normals_prc_compressed_3d = (double *)prc_calloc(ctx, sizeof(double), number_of_normals * 3);
    if (data->normals_prc_compressed_3d == NULL)
        return PRC_ERROR_MEMORY;

    vertex_count = 0;
    DEBUG_LOG("\nVertices\n");
    for (k = 0; k < num_points; k++)
    {
        data->vertices_prc_compressed_3d[vertex_count] = vertices_out[k].x;
        data->vertices_prc_compressed_3d[vertex_count + 1] = vertices_out[k].y;
        data->vertices_prc_compressed_3d[vertex_count + 2] = vertices_out[k].z;
        DEBUG_LOG("%d: [%lf %lf %lf]\n", k, vertices_out[k].x, vertices_out[k].y, vertices_out[k].z);
        vertex_count += 3;
    }

    vertex_count = 0;
    DEBUG_LOG("\nNormals\n");
    for (k = 0; k < number_of_normals; k++)
    {
        data->normals_prc_compressed_3d[vertex_count] = normals_vertex[k].x;
        data->normals_prc_compressed_3d[vertex_count + 1] = normals_vertex[k].y;
        data->normals_prc_compressed_3d[vertex_count + 2] = normals_vertex[k].z;
        DEBUG_LOG("%d: [%lf %lf %lf]\n",k, normals_vertex[k].x, normals_vertex[k].y, normals_vertex[k].z);
        vertex_count += 3;
    }

    DEBUG_LOG("\nTriangle Indices: [ ");
    for (k = 0; k < num_triangles * 3; k++)
    {
        DEBUG_LOG("%d, ", triangle_indices[k]);
    }
    DEBUG_LOG(" ]\n");

    DEBUG_LOG("\nNormal Indices: [ ");
    for (k = 0; k <num_triangles * 3; k++)
    {
        DEBUG_LOG("%d, ", vertex_normal_indices[k]);
    }
    DEBUG_LOG(" ]\n");

	if (face_normal_indices != NULL)
	{
		prc_free(ctx, face_normal_indices);
	}

	if (face_styles != NULL)
	{
		prc_free(ctx, face_styles);
	}

	if (face_encountered != NULL)
	{
		prc_free(ctx, face_encountered);
	}

    /* Just do this assignment. Every triangle either has or does not have
       and assignment of a style. They may or may not all be the same. We will
       deal with this when we get this into a form for the GPU */
    /* These values are biased.  Lets unbias them now */
    data->triangle_styles = triangle_style_array;
    if (triangle_style_array != NULL)
    {
        for (k = 0; k < num_triangles; k++)
        {
            /* A safety check */
            if (triangle_style_array[k] == 0)
            {
                triangle_style_array[k] = 0;
            }
            else
            {
                triangle_style_array[k] = triangle_style_array[k] - 1;
            }
        }
    }

    /* Check if the stack is empty. If not, then clear it */
    while (!prc_stack_empty(ctx, &stack))
    {
        code = pop_stack(ctx, &stack, &treated_details);
        if (code < 0)
        {
            return code;
        }
    }

     /* Free each unique indice1 buffer once by pointer identity.
         Keep pointers intact during this pass so duplicate detection
         can compare against earlier entries. */
    for (k = 0; k < edge_list.capacity; k++)
    {
        int m;
        uint8_t already_freed = 0;
        uint32_t *indices = edge_list.edge[k].indice1;

        if (indices == NULL)
            continue;

        for (m = 0; m < k; m++)
        {
            if (edge_list.edge[m].indice1 == indices)
            {
                already_freed = 1;
                break;
            }
        }

        if (!already_freed)
        {
            prc_free(ctx, indices);
        }
    }

    /* Clear edge slots after cleanup pass completes. */
    for (k = 0; k < edge_list.capacity; k++)
    {
        edge_list.edge[k].indice1 = NULL;
        edge_list.edge[k].capacity = 0;
        edge_list.edge[k].num_second_indices = 0;
    }
    prc_free(ctx, edge_list.edge);

    prc_free(ctx, normals_vertex);
    prc_free(ctx, point_array_scaled);
    prc_free(ctx, vertices_out);
    if (decoded_angles != NULL)
        prc_free(ctx, decoded_angles);
    if (face_normal_decoded != NULL)
        prc_free(ctx, face_normal_decoded);
    if (face_normals != NULL)
        prc_free(ctx, face_normals);
    if (multiple_normals != NULL)
    {
        for (k = 0; k < multiple_normals_size; k++)
        {
            if (multiple_normals[k].vertex_normal_state == PRC_VERTEX_NORM_IS_MULTIPLE)
            {
                prc_free(ctx, multiple_normals[k].normal_indices);
            }
        }
        prc_free(ctx, multiple_normals);
    }

#if VERTEX_DEBUG
    if (debug_tess)
    {
        if (actual_vertices != NULL)
            prc_free(ctx, actual_vertices);
        if (actual_indices != NULL)
            prc_free(ctx, actual_indices);
    }
#endif
    return 0;
}

static void
prc_compressed_tess_get_normal(prc_context *ctx, prc_tess_3d_compressed *data,
           prc_compressed_tess_edge *edge, uint8_t triangle_index, prc_vec3 *normal)
{
    double *normals_array = data->normals_prc_compressed_3d;
    uint32_t *normal_indices = data->normal_indices_prc_compressed_3d;
    uint32_t indices_offset = (triangle_index == 1)
        ? edge->tri_one_indices_offset
        : edge->tri_two_indices_offset;
    uint32_t normal_idx;

    /* normals_prc_compressed_3d holds unique normals addressed through
       normal_indices_prc_compressed_3d[corner_offset].  The indices_offset
       is triangle_index * 3 — a corner base, not a direct normal offset —
       so we must always go through the index array. */
    if (normal_indices != NULL)
    {
        normal_idx = normal_indices[indices_offset];
        normal->x = normals_array[normal_idx * 3];
        normal->y = normals_array[normal_idx * 3 + 1];
        normal->z = normals_array[normal_idx * 3 + 2];
    }
    else
    {
        /* Fallback: direct packed layout (one normal per corner sequentially). */
        normal->x = normals_array[indices_offset];
        normal->y = normals_array[indices_offset + 1];
        normal->z = normals_array[indices_offset + 2];
    }
}

/* Another helper function for the edge list creation */
static void
prc_compressed_tess_set_edge(prc_compressed_tess_edge_list *edge_list,
    prc_compressed_tess_edge *edge, uint32_t *indices,
    prc_compressed_tess_edge_check_case_t edge_case, uint32_t indices_offset)
{
    if (edge == NULL)
    {
        /* Grab the next one in our capacity */
        uint32_t num_edges = edge_list->num_edges;
        edge = &edge_list->edge[num_edges];

        edge->tri_one_full_indices[0] = indices[0];
        edge->tri_one_full_indices[1] = indices[1];
        edge->tri_one_full_indices[2] = indices[2];

        edge->tri_one_indices_offset = indices_offset;

        edge->tri_one_edge_case = edge_case;

        switch (edge_case)
        {
            case PRC_COMPRESSED_TESS_EDGE_01:
                edge->tri_one_edge_indices[0] = indices[0];
                edge->tri_one_edge_indices[1] = indices[1];
                break;

            case PRC_COMPRESSED_TESS_EDGE_02:
                edge->tri_one_edge_indices[0] = indices[0];
                edge->tri_one_edge_indices[1] = indices[2];
                break;

            case PRC_COMPRESSED_TESS_EDGE_12:
                edge->tri_one_edge_indices[0] = indices[1];
                edge->tri_one_edge_indices[1] = indices[2];
                break;
        }
        edge->num_triangles = 1;
        edge_list->num_edges++;
    }
    else
    {
        edge->tri_two_full_indices[0] = indices[0];
        edge->tri_two_full_indices[1] = indices[1];
        edge->tri_two_full_indices[2] = indices[2];

        edge->tri_two_indices_offset = indices_offset;

        edge->tri_two_edge_case = edge_case;

        switch (edge_case)
        {
            case PRC_COMPRESSED_TESS_EDGE_01:
                edge->tri_two_edge_indices[0] = indices[0];
                edge->tri_two_edge_indices[1] = indices[1];
                break;
            case PRC_COMPRESSED_TESS_EDGE_02:
                edge->tri_two_edge_indices[0] = indices[0];
                edge->tri_two_edge_indices[1] = indices[2];
                break;
            case PRC_COMPRESSED_TESS_EDGE_12:
                edge->tri_two_edge_indices[0] = indices[1];
                edge->tri_two_edge_indices[1] = indices[2];
                break;
        }
        edge->num_triangles = 2;
    }
}

static uint32_t
prc_compressed_tess_hash_edge_key(uint32_t index1, uint32_t index2, uint32_t mask)
{
    uint32_t min_index = (index1 < index2) ? index1 : index2;
    uint32_t max_index = (index1 < index2) ? index2 : index1;
    uint32_t hash = (min_index * 73856093u) ^ (max_index * 19349663u);
    return hash & mask;
}

static void
prc_compressed_tess_hash_insert_edge(uint32_t *hash_heads, uint32_t *hash_next,
    uint32_t hash_mask, uint32_t index1, uint32_t index2, uint32_t edge_index)
{
    uint32_t bucket = prc_compressed_tess_hash_edge_key(index1, index2, hash_mask);
    hash_next[edge_index] = hash_heads[bucket];
    hash_heads[bucket] = edge_index;
}

/* Find an unmatched edge candidate in the hashed edge buckets. */
static prc_compressed_tess_edge*
prc_compressed_tess_find_open_edge_hashed(prc_compressed_tess_edge_list *edge_list,
    uint32_t *hash_heads, uint32_t *hash_next, uint32_t hash_mask,
    uint32_t index1, uint32_t index2)
{
    uint32_t bucket = prc_compressed_tess_hash_edge_key(index1, index2, hash_mask);
    uint32_t edge_index = hash_heads[bucket];

    while (edge_index != UINT32_MAX)
    {
        prc_compressed_tess_edge *edge = &edge_list->edge[edge_index];

        if (edge->num_triangles == 1)
        {
            uint32_t a = edge->tri_one_edge_indices[0];
            uint32_t b = edge->tri_one_edge_indices[1];
            if ((a == index1 && b == index2) || (a == index2 && b == index1))
            {
                return edge;
            }
        }

        edge_index = hash_next[edge_index];
    }

    return NULL;
}

/* In this function we build the edge list from the triangle list. The edge list
   consists of a list of edges. The members of each edge will be the list of vertices
   for each of the two triangles that share that edge. We use the edge list to step
   through the tessellation and compare the normals.  Making use of the crease angle
   to determine if we need to adjust the normals. There is no vertex splitting
   in this version since the normals are already assigned to each triangle and the
   crease angle has been ignored. So we simply write over the existing normals */
int
prc_compressed_tess_build_edge_list(prc_context *ctx,  prc_tess_3d_compressed *data,
    prc_compressed_tess_edge_list *edge_list)
{
    uint32_t k;
    uint32_t indices[3];
    prc_compressed_tess_edge *edge;
    uint32_t indices_offset; /* Offset into the indices array for the triangle */
    uint32_t num_triangles = data->triangle_face_array_size;
    uint32_t *hash_heads = NULL;
    uint32_t *hash_next = NULL;
    uint32_t hash_capacity = 1;
    uint32_t hash_mask;

    /* We will definitely need less than this */
    edge_list->capacity = num_triangles * 3;
    edge_list->num_edges = 0;
    edge_list->edge = (prc_compressed_tess_edge *)prc_calloc(ctx,
        edge_list->capacity, sizeof(prc_compressed_tess_edge));
    if (edge_list->edge == NULL)
    {
        return PRC_API_ERROR_MEMORY;
    }

    while (hash_capacity < (num_triangles * 6 + 1))
    {
        hash_capacity <<= 1;
        if (hash_capacity == 0)
        {
            hash_capacity = 1;
            break;
        }
    }
    hash_mask = hash_capacity - 1;

    hash_heads = (uint32_t *)prc_calloc(ctx, hash_capacity, sizeof(uint32_t));
    hash_next = (uint32_t *)prc_calloc(ctx, edge_list->capacity, sizeof(uint32_t));
    if (hash_heads == NULL || hash_next == NULL)
    {
        prc_free(ctx, hash_heads);
        prc_free(ctx, hash_next);
        prc_free(ctx, edge_list->edge);
        edge_list->edge = NULL;
        edge_list->capacity = 0;
        edge_list->num_edges = 0;
        return PRC_API_ERROR_MEMORY;
    }

    for (k = 0; k < hash_capacity; k++)
    {
        hash_heads[k] = UINT32_MAX;
    }

    for (k = 0; k < num_triangles; k++)
    {
        /* Grab the three indices for the triangle */
        indices[0] = data->triangle_indices_prc_compressed_3d[k * 3];
        indices[1] = data->triangle_indices_prc_compressed_3d[k * 3 + 1];
        indices[2] = data->triangle_indices_prc_compressed_3d[k * 3 + 2];

        /* Offset into the indices for that triangle (normals and vertices) */
        indices_offset = k * 3;

        /* Now search through the existing edge list to see if any of these edges
           are already present */
        /* Edge 0-1 */
        edge = prc_compressed_tess_find_open_edge_hashed(edge_list, hash_heads, hash_next,
            hash_mask, indices[0], indices[1]);
        if (edge == NULL)
        {
            uint32_t new_edge_index = edge_list->num_edges;
            if (new_edge_index >= edge_list->capacity)
            {
                prc_free(ctx, hash_heads);
                prc_free(ctx, hash_next);
                prc_free(ctx, edge_list->edge);
                edge_list->edge = NULL;
                edge_list->capacity = 0;
                edge_list->num_edges = 0;
                return PRC_ERROR_INTERNAL;
            }
            prc_compressed_tess_set_edge(edge_list, NULL, indices,
                PRC_COMPRESSED_TESS_EDGE_01, indices_offset);
            prc_compressed_tess_hash_insert_edge(hash_heads, hash_next, hash_mask,
                indices[0], indices[1], new_edge_index);
        }
        else
        {
            prc_compressed_tess_set_edge(edge_list, edge, indices,
                PRC_COMPRESSED_TESS_EDGE_01, indices_offset);
        }

        /* Edge 0-2 */
        edge = prc_compressed_tess_find_open_edge_hashed(edge_list, hash_heads, hash_next,
            hash_mask, indices[0], indices[2]);
        if (edge == NULL)
        {
            uint32_t new_edge_index = edge_list->num_edges;
            if (new_edge_index >= edge_list->capacity)
            {
                prc_free(ctx, hash_heads);
                prc_free(ctx, hash_next);
                prc_free(ctx, edge_list->edge);
                edge_list->edge = NULL;
                edge_list->capacity = 0;
                edge_list->num_edges = 0;
                return PRC_ERROR_INTERNAL;
            }
            prc_compressed_tess_set_edge(edge_list, NULL, indices,
                PRC_COMPRESSED_TESS_EDGE_02, indices_offset);
            prc_compressed_tess_hash_insert_edge(hash_heads, hash_next, hash_mask,
                indices[0], indices[2], new_edge_index);
        }
        else
        {
            prc_compressed_tess_set_edge(edge_list, edge, indices,
                PRC_COMPRESSED_TESS_EDGE_02, indices_offset);
        }

        /* Edge 1-2 */
        edge = prc_compressed_tess_find_open_edge_hashed(edge_list, hash_heads, hash_next,
            hash_mask, indices[1], indices[2]);
        if (edge == NULL)
        {
            uint32_t new_edge_index = edge_list->num_edges;
            if (new_edge_index >= edge_list->capacity)
            {
                prc_free(ctx, hash_heads);
                prc_free(ctx, hash_next);
                prc_free(ctx, edge_list->edge);
                edge_list->edge = NULL;
                edge_list->capacity = 0;
                edge_list->num_edges = 0;
                return PRC_ERROR_INTERNAL;
            }
            prc_compressed_tess_set_edge(edge_list, NULL, indices,
                PRC_COMPRESSED_TESS_EDGE_12, indices_offset);
            prc_compressed_tess_hash_insert_edge(hash_heads, hash_next, hash_mask,
                indices[1], indices[2], new_edge_index);
        }
        else
        {
            prc_compressed_tess_set_edge(edge_list, edge, indices,
                PRC_COMPRESSED_TESS_EDGE_12, indices_offset);
        }
    }

    prc_free(ctx, hash_heads);
    prc_free(ctx, hash_next);

    return 0;
}

static int
prc_compressed_tess_get_indice_offset(prc_context *ctx, uint32_t *indices,
    uint32_t num_indices, uint32_t index)
{
    uint32_t k;
    for (k = 0; k < num_indices; k++)
    {
        if (indices[k] == index)
            return k;
    }
    return -1;
}

static uint8_t
prc_compressed_tess_float_equal(float a, float b, double epsilon)
{
    double diff = (double)a - (double)b;
    if (diff < 0.0)
    {
        diff = -diff;
    }
    return (diff <= epsilon) ? true : false;
}

/* Smoothing tuning knobs:
    - COLOR_EPSILON/UV_EPSILON control seam matching tolerance.
    - CREASE_HYSTERESIS_DEGREES stabilizes threshold behavior near crease_angle. */
#define PRC_COMPRESSED_TESS_COLOR_EPSILON 1.0e-4
#define PRC_COMPRESSED_TESS_UV_EPSILON 1.0e-5
#define PRC_COMPRESSED_TESS_CREASE_HYSTERESIS_DEGREES 0.25
#define PRC_LINE_EDGE_NORMAL_ANGLE_MIN_DEGREES 90.0
#define PRC_LINE_EDGE_CONCAVE_DOT_EPSILON 1000
#define PRC_ENABLE_CONCAVE_EDGE_DETECTION 0
#define PRC_LINE_EDGE_REASON_NOT_LINE 0
#define PRC_LINE_EDGE_REASON_BOUNDARY 1
#define PRC_LINE_EDGE_REASON_ANGLE 2
#define PRC_LINE_EDGE_REASON_CONCAVE 3

/* Set to 1 to enable detailed edge classification counters/logging.
    Default is off to avoid extra work in edge extraction hot path. */
#ifndef PRC_COMPRESSED_TESS_EDGE_DEBUG
#define PRC_COMPRESSED_TESS_EDGE_DEBUG 0
#endif

static uint8_t
prc_compressed_tess_edge_attributes_compatible(prc_tess_3d_compressed *data,
    prc_compressed_tess_edge *edge)
{
    uint32_t tri1 = edge->tri_one_indices_offset / 3;
    uint32_t tri2 = edge->tri_two_indices_offset / 3;
    float *vertex_colors = data->point_colors_prc_compressed_3d;
    uint32_t tri1_face, tri2_face;
    uint8_t tri1_has_texture = true;
    uint8_t tri2_has_texture = true;
    uint8_t i;

    if (vertex_colors == NULL)
    {
        vertex_colors = data->decoded_point_color_array;
    }

    /* Style must match between the two triangles. */
    if (data->triangle_styles != NULL)
    {
        if (data->triangle_styles[tri1] != data->triangle_styles[tri2])
        {
            return false;
        }
    }

    /* If face-level texture flags are present, both triangles must agree. */
    if (!data->no_texture && data->has_faces && data->triangle_face_array != NULL &&
        data->face_has_texture != NULL)
    {
        tri1_face = data->triangle_face_array[tri1];
        tri2_face = data->triangle_face_array[tri2];
        tri1_has_texture = data->face_has_texture[tri1_face];
        tri2_has_texture = data->face_has_texture[tri2_face];
        if (tri1_has_texture != tri2_has_texture)
        {
            return false;
        }
    }

    /* Compare per-vertex attributes on the two shared edge vertices. */
    for (i = 0; i < 2; i++)
    {
        uint32_t v1 = edge->tri_one_edge_indices[i];
        uint32_t v2 = 0;
        uint8_t found = false;
        uint8_t j;

        for (j = 0; j < 2; j++)
        {
            if (edge->tri_two_edge_indices[j] == v1)
            {
                v2 = edge->tri_two_edge_indices[j];
                found = true;
                break;
            }
        }

        if (!found)
        {
            return false;
        }

        if (vertex_colors != NULL)
        {
            /* Color seams are checked with tolerance so tiny decode differences
               do not prevent smoothing across an otherwise smooth edge. */
            if (!prc_compressed_tess_float_equal(vertex_colors[v1 * 4],
                                                 vertex_colors[v2 * 4],
                                                 PRC_COMPRESSED_TESS_COLOR_EPSILON) ||
                !prc_compressed_tess_float_equal(vertex_colors[v1 * 4 + 1],
                                                 vertex_colors[v2 * 4 + 1],
                                                 PRC_COMPRESSED_TESS_COLOR_EPSILON) ||
                !prc_compressed_tess_float_equal(vertex_colors[v1 * 4 + 2],
                                                 vertex_colors[v2 * 4 + 2],
                                                 PRC_COMPRESSED_TESS_COLOR_EPSILON) ||
                !prc_compressed_tess_float_equal(vertex_colors[v1 * 4 + 3],
                                                 vertex_colors[v2 * 4 + 3],
                                                 PRC_COMPRESSED_TESS_COLOR_EPSILON))
            {
                return false;
            }
        }

        if (!data->no_texture && data->uv_coordinates_3d != NULL && tri1_has_texture)
        {
            /* UV seams are also tolerance-based to absorb quantization noise. */
            if (!prc_compressed_tess_float_equal(data->uv_coordinates_3d[v1 * 2],
                                                 data->uv_coordinates_3d[v2 * 2],
                                                 PRC_COMPRESSED_TESS_UV_EPSILON) ||
                !prc_compressed_tess_float_equal(data->uv_coordinates_3d[v1 * 2 + 1],
                                                 data->uv_coordinates_3d[v2 * 2 + 1],
                                                 PRC_COMPRESSED_TESS_UV_EPSILON))
            {
                return false;
            }
        }
    }

    return true;
}

static double
prc_compressed_tess_triangle_area_from_index(prc_tess_3d_compressed *data,
    uint32_t tri_index)
{
    uint32_t base = tri_index * 3;
    uint32_t i0 = data->triangle_indices_prc_compressed_3d[base];
    uint32_t i1 = data->triangle_indices_prc_compressed_3d[base + 1];
    uint32_t i2 = data->triangle_indices_prc_compressed_3d[base + 2];
    prc_vec3 p0, p1, p2;

    p0.x = data->vertices_prc_compressed_3d[i0 * 3];
    p0.y = data->vertices_prc_compressed_3d[i0 * 3 + 1];
    p0.z = data->vertices_prc_compressed_3d[i0 * 3 + 2];

    p1.x = data->vertices_prc_compressed_3d[i1 * 3];
    p1.y = data->vertices_prc_compressed_3d[i1 * 3 + 1];
    p1.z = data->vertices_prc_compressed_3d[i1 * 3 + 2];

    p2.x = data->vertices_prc_compressed_3d[i2 * 3];
    p2.y = data->vertices_prc_compressed_3d[i2 * 3 + 1];
    p2.z = data->vertices_prc_compressed_3d[i2 * 3 + 2];

    return prc_vec_area_of_triangle(p0, p1, p2);
}

int
prc_compressed_tess_apply_crease_angle(prc_context *ctx, prc_tess_3d_compressed *data)
{
    int code;
    uint32_t k;
    uint32_t num_triangles;
    uint32_t num_corners;
    uint32_t num_vertices;
    uint32_t normal_index;
    uint32_t normal_capacity;
    uint32_t *normal_indices_new = NULL;
    prc_vec3 *normals_new = NULL;
    uint8_t *corner_used = NULL;
    uint32_t *vertex_corner_counts = NULL;
    uint32_t *vertex_corner_offsets = NULL;
    uint32_t *vertex_corner_work = NULL;
    uint32_t *vertex_corner_list = NULL;
    uint8_t *edge_neighbor_ok = NULL;
    uint8_t *triangle_edge_count = NULL;
    uint32_t *triangle_edges = NULL;
    double *triangle_area_cache = NULL;
    uint32_t *component_stack = NULL;
    prc_compressed_tess_edge_list edge_list;
    /* Small angular hysteresis avoids unstable toggling near the crease threshold. */
    double crease_hysteresis = PRC_COMPRESSED_TESS_CREASE_HYSTERESIS_DEGREES * PRC_PI / 180.0;

    num_triangles = data->num_triangle_indices_prc_compressed_3d / 3;
    num_corners = num_triangles * 3;
    num_vertices = (uint32_t)data->num_vertices_prc_compressed_3d;

    code = prc_compressed_tess_build_edge_list(ctx, data, &edge_list);
    if (code < 0)
    {
        return code;
    }

    normal_capacity = num_corners;
    normals_new = (prc_vec3 *)prc_calloc(ctx, normal_capacity, sizeof(prc_vec3));
    if (normals_new == NULL)
    {
        prc_free(ctx, edge_list.edge);
        return PRC_ERROR_MEMORY;
    }

    normal_indices_new = (uint32_t *)prc_calloc(ctx, num_corners, sizeof(uint32_t));
    corner_used = (uint8_t *)prc_calloc(ctx, num_corners, sizeof(uint8_t));
    vertex_corner_counts = (uint32_t *)prc_calloc(ctx, num_vertices, sizeof(uint32_t));
    vertex_corner_offsets = (uint32_t *)prc_calloc(ctx, num_vertices + 1, sizeof(uint32_t));
    vertex_corner_work = (uint32_t *)prc_calloc(ctx, num_vertices, sizeof(uint32_t));
    vertex_corner_list = (uint32_t *)prc_calloc(ctx, num_corners, sizeof(uint32_t));
    edge_neighbor_ok = (uint8_t *)prc_calloc(ctx, edge_list.num_edges, sizeof(uint8_t));
    triangle_edge_count = (uint8_t *)prc_calloc(ctx, num_triangles, sizeof(uint8_t));
    triangle_edges = (uint32_t *)prc_calloc(ctx, num_triangles * 3, sizeof(uint32_t));
    triangle_area_cache = (double *)prc_calloc(ctx, num_triangles, sizeof(double));
    component_stack = (uint32_t *)prc_calloc(ctx, num_corners, sizeof(uint32_t));

    if (normal_indices_new == NULL || corner_used == NULL || vertex_corner_counts == NULL ||
        vertex_corner_offsets == NULL || vertex_corner_work == NULL || vertex_corner_list == NULL ||
        edge_neighbor_ok == NULL || triangle_edge_count == NULL || triangle_edges == NULL ||
        triangle_area_cache == NULL || component_stack == NULL)
    {
        prc_free(ctx, edge_list.edge);
        prc_free(ctx, normals_new);
        prc_free(ctx, normal_indices_new);
        prc_free(ctx, corner_used);
        prc_free(ctx, vertex_corner_counts);
        prc_free(ctx, vertex_corner_offsets);
        prc_free(ctx, vertex_corner_work);
        prc_free(ctx, vertex_corner_list);
        prc_free(ctx, edge_neighbor_ok);
        prc_free(ctx, triangle_edge_count);
        prc_free(ctx, triangle_edges);
        prc_free(ctx, triangle_area_cache);
        prc_free(ctx, component_stack);
        return PRC_ERROR_MEMORY;
    }

    for (k = 0; k < num_triangles; k++)
    {
        triangle_area_cache[k] = prc_compressed_tess_triangle_area_from_index(data, k);
    }

    /* Build per-vertex list of triangle corners. */
    for (k = 0; k < num_corners; k++)
    {
        uint32_t v = data->triangle_indices_prc_compressed_3d[k];
        if (v >= num_vertices)
        {
            prc_free(ctx, edge_list.edge);
            prc_free(ctx, normals_new);
            prc_free(ctx, normal_indices_new);
            prc_free(ctx, corner_used);
            prc_free(ctx, vertex_corner_counts);
            prc_free(ctx, vertex_corner_offsets);
            prc_free(ctx, vertex_corner_work);
            prc_free(ctx, vertex_corner_list);
            prc_free(ctx, edge_neighbor_ok);
            prc_free(ctx, triangle_edge_count);
            prc_free(ctx, triangle_edges);
            prc_free(ctx, triangle_area_cache);
            prc_free(ctx, component_stack);
            return PRC_ERROR_INTERNAL;
        }
        vertex_corner_counts[v]++;
    }

    vertex_corner_offsets[0] = 0;
    for (k = 0; k < num_vertices; k++)
    {
        vertex_corner_offsets[k + 1] = vertex_corner_offsets[k] + vertex_corner_counts[k];
        vertex_corner_work[k] = vertex_corner_offsets[k];
    }

    for (k = 0; k < num_corners; k++)
    {
        uint32_t v = data->triangle_indices_prc_compressed_3d[k];
        vertex_corner_list[vertex_corner_work[v]] = k;
        vertex_corner_work[v]++;
    }

    /* Precompute whether each edge can connect its two triangles.
       Also build per-triangle edge adjacency so BFS only checks local edges. */
    for (k = 0; k < (uint32_t)edge_list.num_edges; k++)
    {
        prc_compressed_tess_edge *edge = &edge_list.edge[k];
        uint32_t tri1 = edge->tri_one_indices_offset / 3;

        if (tri1 >= num_triangles || triangle_edge_count[tri1] >= 3)
        {
            prc_free(ctx, edge_list.edge);
            prc_free(ctx, normals_new);
            prc_free(ctx, normal_indices_new);
            prc_free(ctx, corner_used);
            prc_free(ctx, vertex_corner_counts);
            prc_free(ctx, vertex_corner_offsets);
            prc_free(ctx, vertex_corner_work);
            prc_free(ctx, vertex_corner_list);
            prc_free(ctx, edge_neighbor_ok);
            prc_free(ctx, triangle_edge_count);
            prc_free(ctx, triangle_edges);
            prc_free(ctx, component_stack);
            return PRC_ERROR_INTERNAL;
        }
        triangle_edges[tri1 * 3 + triangle_edge_count[tri1]] = k;
        triangle_edge_count[tri1]++;

        edge_neighbor_ok[k] = false;
        if (edge->num_triangles == 2)
        {
            prc_vec3 normal1;
            prc_vec3 normal2;
            double angle;
            uint8_t allow = false;
            uint32_t tri2 = edge->tri_two_indices_offset / 3;

            if (tri2 >= num_triangles || triangle_edge_count[tri2] >= 3)
            {
                prc_free(ctx, edge_list.edge);
                prc_free(ctx, normals_new);
                prc_free(ctx, normal_indices_new);
                prc_free(ctx, corner_used);
                prc_free(ctx, vertex_corner_counts);
                prc_free(ctx, vertex_corner_offsets);
                prc_free(ctx, vertex_corner_work);
                prc_free(ctx, vertex_corner_list);
                prc_free(ctx, edge_neighbor_ok);
                prc_free(ctx, triangle_edge_count);
                prc_free(ctx, triangle_edges);
                prc_free(ctx, triangle_area_cache);
                prc_free(ctx, component_stack);
                return PRC_ERROR_INTERNAL;
            }
            triangle_edges[tri2 * 3 + triangle_edge_count[tri2]] = k;
            triangle_edge_count[tri2]++;

            if (prc_compressed_tess_edge_attributes_compatible(data, edge))
            {
                prc_compressed_tess_get_normal(ctx, data, edge, 1, &normal1);
                prc_compressed_tess_get_normal(ctx, data, edge, 2, &normal2);
                code = prc_vec_angle_between_vectors(normal1, normal2, &angle);
                if (code < 0)
                {
                    prc_free(ctx, edge_list.edge);
                    prc_free(ctx, normals_new);
                    prc_free(ctx, normal_indices_new);
                    prc_free(ctx, corner_used);
                    prc_free(ctx, vertex_corner_counts);
                    prc_free(ctx, vertex_corner_offsets);
                    prc_free(ctx, vertex_corner_work);
                    prc_free(ctx, vertex_corner_list);
                    prc_free(ctx, edge_neighbor_ok);
                    prc_free(ctx, triangle_edge_count);
                    prc_free(ctx, triangle_edges);
                    prc_free(ctx, triangle_area_cache);
                    prc_free(ctx, component_stack);
                    return PRC_ERROR_INTERNAL;
                }
                /* Hysteresis slightly expands smoothing near the threshold so
                   tiny numerical differences do not create visible faceting noise. */
                allow = (angle <= (data->crease_angle + crease_hysteresis)) ? true : false;
            }

            edge_neighbor_ok[k] = allow;
        }
    }

    normal_index = 0;

    /* For each vertex, form connected components in the one-ring using allowed edges. */
    for (k = 0; k < num_vertices; k++)
    {
        uint32_t begin = vertex_corner_offsets[k];
        uint32_t end = vertex_corner_offsets[k + 1];
        uint32_t c;

        for (c = begin; c < end; c++)
        {
            uint32_t seed_corner = vertex_corner_list[c];
            prc_vec3 sum;
            prc_vec3 dominant_normal;
            double dominant_weight = 0.0;
            uint8_t dominant_normal_set = false;
            uint32_t stack_size = 0;
            uint32_t comp_count = 0;

            if (corner_used[seed_corner])
            {
                continue;
            }

            sum.x = 0.0;
            sum.y = 0.0;
            sum.z = 0.0;

            corner_used[seed_corner] = true;
            component_stack[stack_size++] = seed_corner;

            while (stack_size > 0)
            {
                uint32_t corner = component_stack[--stack_size];
                uint32_t tri = corner / 3;
                prc_vec3 tri_n;
                prc_vec3 tri_n_weighted;
                     double tri_area_weight;
                tri_n.x = data->normals_prc_compressed_3d[tri * 3];
                tri_n.y = data->normals_prc_compressed_3d[tri * 3 + 1];
                tri_n.z = data->normals_prc_compressed_3d[tri * 3 + 2];

                     /* Area-weighted averaging better matches reference renderers:
                         larger triangles contribute proportionally more to the smooth normal. */
                     tri_area_weight = triangle_area_cache[tri];
                 if (!(tri_area_weight > 1.0e-20))
                {
                    tri_area_weight = 1.0;
                }
                if (!dominant_normal_set || tri_area_weight > dominant_weight)
                {
                    dominant_normal = tri_n;
                    dominant_weight = tri_area_weight;
                    dominant_normal_set = true;
                }
                tri_n_weighted = tri_n;
                prc_vec_scale(tri_area_weight, &tri_n_weighted);
                prc_vec_add(sum, tri_n_weighted, &sum);
                comp_count++;

                /* Traverse only edges attached to this triangle and push the
                   neighboring corner for the same vertex when allowed. */
                {
                    uint32_t e;
                    uint32_t tri_offset = tri * 3;
                    uint32_t tri_vertex = data->triangle_indices_prc_compressed_3d[corner];
                    uint32_t tri_edge_base = tri * 3;

                    for (e = 0; e < triangle_edge_count[tri]; e++)
                    {
                        uint32_t edge_index = triangle_edges[tri_edge_base + e];
                        prc_compressed_tess_edge *edge = &edge_list.edge[edge_index];
                        uint32_t edge_v0;
                        uint32_t edge_v1;
                        uint32_t other_tri_offset;
                        int other_local;
                        uint32_t other_corner;

                        if (edge->num_triangles != 2)
                        {
                            continue;
                        }

                        edge_v0 = edge->tri_one_edge_indices[0];
                        edge_v1 = edge->tri_one_edge_indices[1];
                        if (!(tri_vertex == edge_v0 || tri_vertex == edge_v1))
                        {
                            continue;
                        }

                        if (!edge_neighbor_ok[edge_index])
                        {
                            continue;
                        }

                        if (edge->tri_one_indices_offset == tri_offset)
                        {
                            other_tri_offset = edge->tri_two_indices_offset;
                            other_local = prc_compressed_tess_get_indice_offset(ctx,
                                edge->tri_two_full_indices, 3, tri_vertex);
                        }
                        else
                        {
                            other_tri_offset = edge->tri_one_indices_offset;
                            other_local = prc_compressed_tess_get_indice_offset(ctx,
                                edge->tri_one_full_indices, 3, tri_vertex);
                        }

                        if (other_local < 0)
                        {
                            continue;
                        }

                        other_corner = other_tri_offset + (uint32_t)other_local;
                        if (!corner_used[other_corner])
                        {
                            corner_used[other_corner] = true;
                            component_stack[stack_size++] = other_corner;
                        }
                    }
                }
            }

            if (comp_count == 0)
            {
                continue;
            }

                /* We normalize the weighted sum directly; dividing by total weight
                    is unnecessary because normalization removes magnitude. */
            code = prc_vec_normalize(&sum);
            if (code < 0)
            {
                if (!dominant_normal_set)
                {
                    prc_free(ctx, edge_list.edge);
                    prc_free(ctx, normals_new);
                    prc_free(ctx, normal_indices_new);
                    prc_free(ctx, corner_used);
                    prc_free(ctx, vertex_corner_counts);
                    prc_free(ctx, vertex_corner_offsets);
                    prc_free(ctx, vertex_corner_work);
                    prc_free(ctx, vertex_corner_list);
                    prc_free(ctx, edge_neighbor_ok);
                    prc_free(ctx, triangle_edge_count);
                    prc_free(ctx, triangle_edges);
                    prc_free(ctx, triangle_area_cache);
                    prc_free(ctx, component_stack);
                    return code;
                }

                sum = dominant_normal;
                code = prc_vec_normalize(&sum);
                if (code < 0)
                {
                    prc_free(ctx, edge_list.edge);
                    prc_free(ctx, normals_new);
                    prc_free(ctx, normal_indices_new);
                    prc_free(ctx, corner_used);
                    prc_free(ctx, vertex_corner_counts);
                    prc_free(ctx, vertex_corner_offsets);
                    prc_free(ctx, vertex_corner_work);
                    prc_free(ctx, vertex_corner_list);
                    prc_free(ctx, edge_neighbor_ok);
                    prc_free(ctx, triangle_edge_count);
                    prc_free(ctx, triangle_edges);
                    prc_free(ctx, triangle_area_cache);
                    prc_free(ctx, component_stack);
                    return code;
                }
            }

            if (normal_index >= normal_capacity)
            {
                prc_free(ctx, edge_list.edge);
                prc_free(ctx, normals_new);
                prc_free(ctx, normal_indices_new);
                prc_free(ctx, corner_used);
                prc_free(ctx, vertex_corner_counts);
                prc_free(ctx, vertex_corner_offsets);
                prc_free(ctx, vertex_corner_work);
                prc_free(ctx, vertex_corner_list);
                prc_free(ctx, edge_neighbor_ok);
                prc_free(ctx, triangle_edge_count);
                prc_free(ctx, triangle_edges);
                prc_free(ctx, triangle_area_cache);
                prc_free(ctx, component_stack);
                return PRC_ERROR_MEMORY;
            }

            normals_new[normal_index] = sum;

            /* Assign this normal to all visited corners in this component. */
            for (c = begin; c < end; c++)
            {
                uint32_t corner = vertex_corner_list[c];
                if (corner_used[corner] && normal_indices_new[corner] == 0)
                {
                    uint32_t tri = corner / 3;
                    uint32_t tri_offset = tri * 3;
                    if (data->triangle_indices_prc_compressed_3d[corner] == k &&
                        (normal_indices_new[corner] == 0))
                    {
                        normal_indices_new[corner] = normal_index + 1;
                    }
                    (void)tri_offset;
                }
            }

            normal_index++;
        }
    }

    /* Convert 1-based temp indices to 0-based and fill any unset corner with triangle normal. */
    for (k = 0; k < num_corners; k++)
    {
        if (normal_indices_new[k] > 0)
        {
            normal_indices_new[k] -= 1;
        }
        else
        {
            uint32_t tri = k / 3;
            prc_vec3 tri_n;
            tri_n.x = data->normals_prc_compressed_3d[tri * 3];
            tri_n.y = data->normals_prc_compressed_3d[tri * 3 + 1];
            tri_n.z = data->normals_prc_compressed_3d[tri * 3 + 2];
            if (normal_index >= normal_capacity)
            {
                prc_free(ctx, edge_list.edge);
                prc_free(ctx, normals_new);
                prc_free(ctx, normal_indices_new);
                prc_free(ctx, corner_used);
                prc_free(ctx, vertex_corner_counts);
                prc_free(ctx, vertex_corner_offsets);
                prc_free(ctx, vertex_corner_work);
                prc_free(ctx, vertex_corner_list);
                prc_free(ctx, edge_neighbor_ok);
                prc_free(ctx, triangle_edge_count);
                prc_free(ctx, triangle_edges);
                prc_free(ctx, component_stack);
                return PRC_ERROR_MEMORY;
            }
            normals_new[normal_index] = tri_n;
            normal_indices_new[k] = normal_index;
            normal_index++;
        }
    }

    prc_free(ctx, data->normals_prc_compressed_3d);
    prc_free(ctx, data->normal_indices_prc_compressed_3d);

    data->num_normals_prc_compressed_3d = (int)normal_index;
    data->normals_prc_compressed_3d = (double *)prc_calloc(ctx, sizeof(double), normal_index * 3);
    if (data->normals_prc_compressed_3d == NULL)
    {
        prc_free(ctx, edge_list.edge);
        prc_free(ctx, normals_new);
        prc_free(ctx, normal_indices_new);
        prc_free(ctx, corner_used);
        prc_free(ctx, vertex_corner_counts);
        prc_free(ctx, vertex_corner_offsets);
        prc_free(ctx, vertex_corner_work);
        prc_free(ctx, vertex_corner_list);
        prc_free(ctx, edge_neighbor_ok);
        prc_free(ctx, triangle_edge_count);
        prc_free(ctx, triangle_edges);
        prc_free(ctx, component_stack);
        return PRC_ERROR_MEMORY;
    }

    for (k = 0; k < normal_index; k++)
    {
        data->normals_prc_compressed_3d[k * 3] = normals_new[k].x;
        data->normals_prc_compressed_3d[k * 3 + 1] = normals_new[k].y;
        data->normals_prc_compressed_3d[k * 3 + 2] = normals_new[k].z;
    }

    data->normal_indices_prc_compressed_3d = normal_indices_new;

    prc_free(ctx, edge_list.edge);
    prc_free(ctx, normals_new);
    prc_free(ctx, corner_used);
    prc_free(ctx, vertex_corner_counts);
    prc_free(ctx, vertex_corner_offsets);
    prc_free(ctx, vertex_corner_work);
    prc_free(ctx, vertex_corner_list);
    prc_free(ctx, edge_neighbor_ok);
    prc_free(ctx, triangle_edge_count);
    prc_free(ctx, triangle_edges);
    prc_free(ctx, triangle_area_cache);
    prc_free(ctx, component_stack);

    return 0;
}

static void
prc_compressed_tess_get_triangle_centroid_from_offset(prc_tess_3d_compressed *data,
    uint32_t tri_indices_offset, prc_vec3 *centroid)
{
    uint32_t i0 = data->triangle_indices_prc_compressed_3d[tri_indices_offset];
    uint32_t i1 = data->triangle_indices_prc_compressed_3d[tri_indices_offset + 1];
    uint32_t i2 = data->triangle_indices_prc_compressed_3d[tri_indices_offset + 2];
    prc_vec3 p0, p1, p2;

    p0.x = data->vertices_prc_compressed_3d[i0 * 3];
    p0.y = data->vertices_prc_compressed_3d[i0 * 3 + 1];
    p0.z = data->vertices_prc_compressed_3d[i0 * 3 + 2];

    p1.x = data->vertices_prc_compressed_3d[i1 * 3];
    p1.y = data->vertices_prc_compressed_3d[i1 * 3 + 1];
    p1.z = data->vertices_prc_compressed_3d[i1 * 3 + 2];

    p2.x = data->vertices_prc_compressed_3d[i2 * 3];
    p2.y = data->vertices_prc_compressed_3d[i2 * 3 + 1];
    p2.z = data->vertices_prc_compressed_3d[i2 * 3 + 2];

    centroid->x = (p0.x + p1.x + p2.x) / 3.0;
    centroid->y = (p0.y + p1.y + p2.y) / 3.0;
    centroid->z = (p0.z + p1.z + p2.z) / 3.0;
}

static int
prc_compressed_tess_edge_is_line(prc_context *ctx, prc_tess_3d_compressed *data,
    prc_compressed_tess_edge *edge, uint8_t *is_line, uint8_t *reason)
{
    int code;
    prc_vec3 n1, n2;
    double angle_radians;
    double angle_degrees;

    *is_line = false;
    if (reason != NULL)
    {
        *reason = PRC_LINE_EDGE_REASON_NOT_LINE;
    }

    /* Always keep boundary edges as visible silhouette/cut edges. */
    if (edge->num_triangles == 1)
    {
        *is_line = true;
        if (reason != NULL)
        {
            *reason = PRC_LINE_EDGE_REASON_BOUNDARY;
        }
        return 0;
    }

    if (edge->num_triangles != 2)
    {
        return 0;
    }

    prc_compressed_tess_get_normal(ctx, data, edge, 1, &n1);
    prc_compressed_tess_get_normal(ctx, data, edge, 2, &n2);
    code = prc_vec_angle_between_vectors(n1, n2, &angle_radians);
    if (code < 0)
    {
        return PRC_ERROR_INTERNAL;
    }

    angle_degrees = angle_radians * 180.0 / PRC_PI;

    /* Sharp normal differences are edge lines (e.g. 90-degree crease). */
    if (angle_degrees >= PRC_LINE_EDGE_NORMAL_ANGLE_MIN_DEGREES)
    {
        *is_line = true;
        if (reason != NULL)
        {
            *reason = PRC_LINE_EDGE_REASON_ANGLE;
        }
        return 0;
    }

    /* Optional concavity test: disabled by default for better Adobe parity. */
#if PRC_ENABLE_CONCAVE_EDGE_DETECTION
    {
        prc_vec3 c1, c2, d12;
        double dot1, dot2;

        prc_compressed_tess_get_triangle_centroid_from_offset(data,
            edge->tri_one_indices_offset, &c1);
        prc_compressed_tess_get_triangle_centroid_from_offset(data,
            edge->tri_two_indices_offset, &c2);

        prc_vec_sub(c2, c1, &d12);
        dot1 = prc_vec_dot_product(n1, d12);
        dot2 = prc_vec_dot_product(n2, d12);

        if (dot1 > PRC_LINE_EDGE_CONCAVE_DOT_EPSILON &&
            dot2 < -PRC_LINE_EDGE_CONCAVE_DOT_EPSILON)
        {
            *is_line = true;
            if (reason != NULL)
            {
                *reason = PRC_LINE_EDGE_REASON_CONCAVE;
            }
        }
    }
#endif

    return 0;
}

int prc_compute_edges_compressed_tess(prc_context *ctx, prc_tess_3d_compressed *data)
{
    int code;
    uint32_t k;
    uint32_t edge_count = 0;
    uint32_t num_vertices;
    uint32_t num_edges_to_store;
    prc_compressed_tess_edge_list edge_list;
    uint8_t *edge_keep_flags = NULL;
    uint32_t debug_total_edges = 0;
    uint32_t debug_kept_edges = 0;
    uint32_t debug_angle_edges = 0;
    uint32_t debug_boundary_edges = 0;
    uint32_t debug_concave_edges = 0;

    if (data == NULL || data->triangle_indices_prc_compressed_3d == NULL ||
        data->vertices_prc_compressed_3d == NULL || data->normals_prc_compressed_3d == NULL)
    {
        return PRC_ERROR_INTERNAL;
    }

    /* If we are recomputing, clear previous allocations first. */
    if (data->edge_vertices != NULL)
    {
        prc_free(ctx, data->edge_vertices);
        data->edge_vertices = NULL;
    }
    if (data->edge_indices != NULL)
    {
        prc_free(ctx, data->edge_indices);
        data->edge_indices = NULL;
    }
    data->number_of_edges = 0;

    num_vertices = (uint32_t)data->num_vertices_prc_compressed_3d;

    code = prc_compressed_tess_build_edge_list(ctx, data, &edge_list);
    if (code < 0)
    {
        return code;
    }

    if (edge_list.num_edges > 0)
    {
        edge_keep_flags = (uint8_t *)prc_calloc(ctx, (uint32_t)edge_list.num_edges,
            sizeof(uint8_t));
        if (edge_keep_flags == NULL)
        {
            prc_free(ctx, edge_list.edge);
            return PRC_ERROR_MEMORY;
        }
    }

    /* Pass 1: count line edges selected by sharpness and concavity tests. */
    for (k = 0; k < (uint32_t)edge_list.num_edges; k++)
    {
        prc_compressed_tess_edge *edge = &edge_list.edge[k];
        uint8_t keep_edge;
        uint8_t reason;
        debug_total_edges++;
        code = prc_compressed_tess_edge_is_line(ctx, data, edge, &keep_edge, &reason);
        if (code < 0)
        {
            prc_free(ctx, edge_list.edge);
            prc_free(ctx, edge_keep_flags);
            return code;
        }

        edge_keep_flags[k] = keep_edge;

        if (keep_edge)
        {
            edge_count++;
            debug_kept_edges++;
            if (reason == PRC_LINE_EDGE_REASON_BOUNDARY)
            {
                debug_boundary_edges++;
            }
            else if (reason == PRC_LINE_EDGE_REASON_ANGLE)
            {
                debug_angle_edges++;
            }
            else if (reason == PRC_LINE_EDGE_REASON_CONCAVE)
            {
                debug_concave_edges++;
            }
        }
    }

    /* Vertices are stored as positions. Indices reference these positions. */
    if (num_vertices > 0)
    {
        data->edge_vertices = (double *)prc_calloc(ctx, num_vertices * 3, sizeof(double));
        if (data->edge_vertices == NULL)
        {
            prc_free(ctx, edge_list.edge);
            prc_free(ctx, edge_keep_flags);
            return PRC_ERROR_MEMORY;
        }
        memcpy(data->edge_vertices, data->vertices_prc_compressed_3d,
            num_vertices * 3 * sizeof(double));
    }

    num_edges_to_store = edge_count;
    if (num_edges_to_store > 0)
    {
        uint32_t edge_index_out = 0;

        data->edge_indices = (uint32_t *)prc_calloc(ctx, num_edges_to_store * 2, sizeof(uint32_t));
        if (data->edge_indices == NULL)
        {
            prc_free(ctx, edge_list.edge);
            prc_free(ctx, edge_keep_flags);
            prc_free(ctx, data->edge_vertices);
            data->edge_vertices = NULL;
            return PRC_ERROR_MEMORY;
        }

        /* Pass 2: write selected edge index pairs. */
        for (k = 0; k < (uint32_t)edge_list.num_edges; k++)
        {
            prc_compressed_tess_edge *edge = &edge_list.edge[k];
            if (edge_keep_flags[k])
            {
                uint32_t v0 = edge->tri_one_edge_indices[0];
                uint32_t v1 = edge->tri_one_edge_indices[1];

                if (v0 >= num_vertices || v1 >= num_vertices)
                {
                    prc_free(ctx, edge_list.edge);
                    prc_free(ctx, edge_keep_flags);
                    prc_free(ctx, data->edge_vertices);
                    prc_free(ctx, data->edge_indices);
                    data->edge_vertices = NULL;
                    data->edge_indices = NULL;
                    data->number_of_edges = 0;
                    return PRC_ERROR_INTERNAL;
                }

                data->edge_indices[edge_index_out * 2] = v0;
                data->edge_indices[edge_index_out * 2 + 1] = v1;
                edge_index_out++;
            }
        }
    }

    if (debug_boundary_edges == edge_count)
    {
        /* All the triangles are disjoint. Don't draw any edges */
        data->number_of_edges = 0;
        if (data->edge_indices != NULL)
        {
            prc_free(ctx, data->edge_indices);
            data->edge_indices = NULL;
        }
        if (data->edge_vertices != NULL)
        {
            prc_free(ctx, data->edge_vertices);
            data->edge_vertices = NULL;
        }
    }
    else
    {
        data->number_of_edges = num_edges_to_store;
    }

#if PRC_COMPRESSED_TESS_EDGE_DEBUG
    DEBUG_LOG("Compressed tess edges: total=%u kept=%u boundary=%u angle=%u concave=%u\n",
        debug_total_edges, debug_kept_edges, debug_boundary_edges,
        debug_angle_edges, debug_concave_edges);
#endif

    prc_free(ctx, edge_list.edge);
    prc_free(ctx, edge_keep_flags);

    return 0;
}
