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
#include <math.h>
#include <float.h>  /* Not sure about this one... Needed for FLT_EPSILON */
#include <prc_context.h>
#include "prc_vector_util.h"
#include "prc_parse_common.h"

union ieee754_float
{
    float f;
    /* This is the IEEE 754 float-precision format. */
    struct
    {
#if defined(PRC_BIG_ENDIAN)
        unsigned int negative:1;
        unsigned int exponent:8;
        unsigned int mantissa:23;
#elif defined(PRC_LITTLE_ENDIAN)
        unsigned int mantissa:23;
        unsigned int exponent:8;
        unsigned int negative:1;
#else
# error "Big/Little endian to be defined"
#endif
    } ieee;
};

/* Compute the cross product of two vectors */
void
prc_vec_cross(prc_vec3 vec1, prc_vec3 vec2, prc_vec3 *output)
{
    output->z = (vec1.x * vec2.y) - (vec1.y * vec2.x);
    output->y = (vec1.z * vec2.x) - (vec1.x * vec2.z);
    output->x = (vec1.y * vec2.z) - (vec1.z * vec2.y);
}

/* Return the length of the vector using the
   ieee754_float definition from the ISO spec */
double
prc_vec_length(prc_vec3 vec)
{
    union ieee754_float fval;
    double dsquared;
    double x = vec.x;
    double y = vec.y;
    double z = vec.z;
    double dx_i, dx_0;
    unsigned int count = 0;

    fval.f = (float) (x * x + y * y + z * z);
    dsquared = (double) fval.f;
    if (fval.ieee.exponent > 127)
        fval.ieee.exponent = (fval.ieee.exponent - 127) /2 + 127;

    dx_0 = (double) fval.f;
    while (count != 100)
    {
        dx_i = (dx_0 + dsquared / dx_0) / 2;
        if ((double) dx_i == (double) dx_0)
            break;
        dx_0 = dx_i;
        count++;
    };

    if (count == 100)
        return (double) -1.0;

    return (double) dx_i;
}

/* Return the norm of the vector with sqrt */
double
prc_vec_length_sqrt(prc_vec3 vec)
{
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

double
prc_vec_dist_between_two_points(prc_vec3 pt1, prc_vec3 pt2)
{
    prc_vec3 diff;
    prc_vec_sub(pt1, pt2, &diff);
    return prc_vec_length(diff);
}

void
prc_vec_compute_find_normal_from_sides(prc_vec3 vec0, prc_vec3 vec1, prc_vec3 vec2, prc_vec3 *normal)
{
    prc_vec3 diff1, diff2;

    prc_vec_sub(vec1, vec0, &diff1);
    prc_vec_sub(vec2, vec0, &diff2);

    /* If the v0 is on the right and v0 v1 is the base then
       a positive normal will mean right is right and left is left */
    prc_vec_cross(diff2, diff1, normal);
    prc_vec_normalize(normal);
}

/* Normalize a vector. If its length is less than FLT_EPSILON, let it be known */
int
prc_vec_normalize(prc_vec3 *vec)
{
    double len = prc_vec_length(*vec);

    if (len >= FLT_EPSILON)
    {
        vec->x = vec->x / len;
        vec->y = vec->y / len;
        vec->z = vec->z / len;
        return 0;
    }
    else
        return PRC_VEC_ERROR;
}

/* Compute the angle between two vectors. A special version for dealing with theta1
   theta2 and theta3 which require much care in precision */
int
prc_vec_angle_between_vectors_normal(prc_vec3 vec1, prc_vec3 vec2, double *angle)
{
    double dot_product;
    double len1 = prc_vec_length(vec1);
    double len2 = prc_vec_length(vec2);
    double arg;
    double acos_val;
    int code;

    code = prc_vec_normalize(&vec1);
    if (code < 0)
        return code;

    code = prc_vec_normalize(&vec2);
    if (code < 0)
        return code;

    arg = prc_vec_dot_product(vec1, vec2);

    /* If we are this close, the angle will be PCR_HALF_PI */
    if (arg > 1.0 - 1.0e-12)
    {
        *angle = PRC_HALF_PI;
    }
    else if (arg < -1.0 + 1.0e-12)
    {
        *angle = PRC_HALF_PI;
    }
    else
    {
        acos_val = acos(arg);
        *angle = fabs(fabs(acos_val) - PRC_HALF_PI);
    }
    return 0;
}


/* Compute the angle between two vectors */
int
prc_vec_angle_between_vectors(prc_vec3 vec1, prc_vec3 vec2, double *angle)
{
    double dot_product;
    double len1 = prc_vec_length(vec1);
    double len2 = prc_vec_length(vec2);
    double arg;
    int code;

    code = prc_vec_normalize(&vec1);
    if (code < 0)
        return code;

    code = prc_vec_normalize(&vec2);
    if (code < 0)
        return code;

    arg = prc_vec_dot_product(vec1, vec2);

    if (arg > 1.0 - 1.0e-12)
    {
        *angle = 0.0;
    }
    else if (arg < -1.0 + 1.0e-12)
    {
        *angle = PRC_PI;
    }
    else
    {
        *angle = acos(arg);
    }

    return 0;
}

int
prc_vec_vectors_are_equal(prc_vec3 vec1, prc_vec3 vec2, double epsilon)
{
    if (fabs(vec1.x - vec2.x) > epsilon)
        return 0;
    if (fabs(vec1.y - vec2.y) > epsilon)
        return 0;
    if (fabs(vec1.z - vec2.z) > epsilon)
        return 0;
    return 1;
}

/* Subtract two vectors */
void
prc_vec_sub(prc_vec3 vec1, prc_vec3 vec2, prc_vec3 *output)
{
    output->x = vec1.x - vec2.x;
    output->y = vec1.y - vec2.y;
    output->z = vec1.z - vec2.z;
}

/* Add two vectors */
void
prc_vec_add(prc_vec3 vec1, prc_vec3 vec2, prc_vec3 *output)
{
    output->x = vec1.x + vec2.x;
    output->y = vec1.y + vec2.y;
    output->z = vec1.z + vec2.z;
}

/* Average two vectors */
void
prc_vec_avg(prc_vec3 vec1, prc_vec3 vec2, prc_vec3 *output)
{
    output->x = (vec1.x + vec2.x) / 2.0;
    output->y = (vec1.y + vec2.y) / 2.0;
    output->z = (vec1.z + vec2.z) / 2.0;
}

/* Average three vectors */
void
prc_vec_avg3(prc_vec3 vec1, prc_vec3 vec2, prc_vec3 vec3, prc_vec3 *output)
{
    output->x = (vec1.x + vec2.x + vec3.x) / 3.0;
    output->y = (vec1.y + vec2.y + vec3.y) / 3.0;
    output->z = (vec1.z + vec2.z + vec3.z) / 3.0;
}

/* Average three vectors */
void
prc_vec_avg3_test(prc_vec3 vec1, prc_vec3 vec2, prc_vec3 vec3, prc_vec3 *output)
{
    prc_vec3 temp1;
    prc_vec_avg(vec1, vec2, &temp1);
    prc_vec_avg(temp1, vec3, output);
}


void
prc_vec_swap(prc_vec3 *vec1, prc_vec3 *vec2)
{
    prc_vec3 temp;
    prc_vec_copy(*vec1, &temp, 0);
    prc_vec_copy(*vec2, vec1, 0);
    prc_vec_copy(temp, vec2, 0);
}

/* Set vector to known values */
void
prc_vec_set(prc_vec3 *vec, double x, double y, double z)
{
    vec->x = x;
    vec->y = y;
    vec->z = z;
}

void
prc_vec_negate(prc_vec3 *vec)
{
    vec->x = -vec->x;
    vec->y = -vec->y;
    vec->z = -vec->z;
}

void
prc_vec_copy(prc_vec3 vec_in, prc_vec3 *vec_out, uint8_t negate)
{
    if (negate)
    {
        vec_out->x = -vec_in.x;
        vec_out->y = -vec_in.y;
        vec_out->z = -vec_in.z;
    }
    else
    {
        vec_out->x = vec_in.x;
        vec_out->y = vec_in.y;
        vec_out->z = vec_in.z;
    }
}

double
prc_vec_dot_product(prc_vec3 vec1, prc_vec3 vec2)
{
    double result;

    result = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    return result;
}

/* Compute the area of a triangle given three points using Heron's formula */
double
prc_vec_area_of_triangle(prc_vec3 pt1, prc_vec3 pt2, prc_vec3 pt3)
{
    prc_vec3 vec1, vec2, cross;

    prc_vec_sub(pt2, pt1, &vec1);
    prc_vec_sub(pt3, pt1, &vec2);
    prc_vec_cross(vec1, vec2, &cross);

    /* Cross-product area is more stable than Heron's formula for skinny
       triangles and avoids NaNs from tiny negative radicands. */
    return 0.5 * prc_vec_length(cross);
}

void
prc_vec_scale(double scale, prc_vec3 *vec)
{
    vec->x = scale * vec->x;
    vec->y = scale * vec->y;
    vec->z = scale * vec->z;
}

/* TODO add special case in computing basis */
int
prc_vec_compute_normal_vector(prc_vec3 v0, prc_vec3 v1, prc_vec3 v2, double theta, double phi, prc_vec3 *normal)
{
    prc_vec3 v1_norm, v2_norm, v3_norm;
    int code;
    double theta1, theta2, theta3;
    prc_vec3 x_norm, y_norm, z_norm, z_norm2;

    prc_vec_sub(v1, v0, &v1_norm);
    code = prc_vec_normalize(&v1_norm);
    if (code < 0)
    {
        return code;
    }

    prc_vec_sub(v2, v0, &v2_norm);
    code = prc_vec_normalize(&v2_norm);
    if (code < 0)
    {
        return code;
    }

    prc_vec_sub(v2, v1, &v3_norm);
    code = prc_vec_normalize(&v3_norm);
    if (code < 0)
    {
        return code;
    }

    theta1 = fabs(acos(prc_vec_dot_product(v1_norm, v2_norm)) - PRC_HALF_PI);
    theta2 = fabs(acos(-prc_vec_dot_product(v3_norm, v1_norm)) - PRC_HALF_PI);
    theta3 = fabs(acos(prc_vec_dot_product(v2_norm, v3_norm)) - PRC_HALF_PI);

    if ((theta1 < theta2) && (theta1 < theta3))
    {
        prc_vec_copy(v3_norm, &x_norm, 0);
        prc_vec_cross(v1_norm, v2_norm, &z_norm);
    }
    else
    {
        if (theta1 < theta3)
        {
            prc_vec_copy(v3_norm, &x_norm, 0);
            prc_vec_cross(v3_norm, v1_norm, &z_norm);
            prc_vec_negate(&z_norm);
        }
        else
        {
            prc_vec_copy(v2_norm, &x_norm, 1);
            prc_vec_cross(v2_norm, v3_norm, &z_norm);
        }
    }

    prc_vec_copy(z_norm, &z_norm2, 0);
    prc_vec_normalize(&z_norm2);
    prc_vec_cross(z_norm2, x_norm, &y_norm);

    prc_vec_scale(cos(phi) * cos(theta), &x_norm);
    prc_vec_scale(sin(theta) * cos(phi), &y_norm);
    prc_vec_scale(sin(phi), &z_norm);

    prc_vec_add(x_norm, y_norm, normal);
    prc_vec_add(*normal, z_norm, normal);

    return 0;
}

/* Take the encoded point and using the basis and origin from the parent to compute the next point */
void
prc_vec_apply_basis_origin(prc_basis basis, prc_vec3 origin, prc_vec3 input_point, prc_vec3 *output_point)
{
    prc_vec3 vecx, vecy, vecz, vec_sum;

    prc_vec_copy(basis.X, &vecx, 0);
    prc_vec_copy(basis.Y, &vecy, 0);
    prc_vec_copy(basis.Z, &vecz, 0);

    prc_vec_scale(input_point.x, &vecx);
    prc_vec_scale(input_point.y, &vecy);
    prc_vec_scale(input_point.z, &vecz);

    prc_vec_add(vecx, vecy, &vec_sum);
    prc_vec_add(vecz, vec_sum, &vec_sum);

    prc_vec_sub(origin, vec_sum, output_point);
}

/* As outlined in Mesh Points and Triangles compute a basis given the three triangle points.
   This new basis is used to compute the next triangle vertices from the compressed data.
   In certain cases (if the vector norms are small) the basis is computed in a different way
   (using prc_vec_make_orth_basis). If we can't compute the basis for some reason,
   we return an error */
int
prc_vec_compute_basis_origin(prc_vec3 v0, prc_vec3 v1, prc_vec3 v3, prc_basis *basis, prc_vec3 *origin)
{
    prc_vec3 ztemp;

    /* Compute the new origin */
    prc_vec_avg(v1, v0, origin);

    /* Now compute the basis. NOTE X already normalized */
    prc_vec_sub(v1, v0, &basis->X);
    if (prc_vec_normalize(&basis->X) < 0)
        return PRC_VEC_ERROR;

    prc_vec_sub(v3, *origin, &ztemp);
    prc_vec_cross(ztemp, basis->X, &basis->Z);
    if (prc_vec_normalize(&basis->Z) < 0)
        return prc_vec_make_orth_basis(basis);

    prc_vec_cross(basis->Z, basis->X, &basis->Y);
    if (prc_vec_normalize(&basis->Y) < 0)
        return prc_vec_make_orth_basis(basis);

    return 0;
}

/* The degenerate-frame fallback for the normal case (7.8.9.4). There is only
   ONE MakeOrthoRep in the spec: 7.8.9.3 (apex frame) and the normal-angle
   text on p.90 both say the null-axis case is "computed by the function
   MakeOrthoRep(), using the unit axis X as input", and that function is
   defined once, in 10.3 "Basis pseudo code". So this is deliberately a thin
   wrapper over prc_vec_make_orth_basis rather than a second implementation.

   FIXED 2026-08-28: this used to be an independent copy whose degenerate
   fallback picked trial axis (0,0,1) where 10.3 picks (1,0,0), under a
   comment claiming it was "exactly what the spec shows" -- it was not. Both
   choices yield a valid orthonormal basis, but different ones: for X
   parallel to (0,1,0) the two differ by a 90-degree rotation about X, which
   changes the decoded normal in exactly the case the clause is there to
   cover. The bug was reachable -- prc_compute_vertex_normal and
   prc_derive_normal (src/prc_decode_compressed_tess.c) both route their
   null-axis case here.

   Worth recording why it survived: 7.8.9.3 and 7.8.9.4 name MakeOrthoRep
   without cross-referencing 10.3, which sits ~180 pages away under an
   annex-like heading, so the function reads as named-but-undefined and
   invites improvisation. An independent implementer reported losing months
   to the same gap and hand-rolling a different orthonormal basis; a defect
   entry asking for the cross-reference is being filed on that basis. */
int
prc_vec_make_orth_basis_normals(prc_basis *basis)
{
    return prc_vec_make_orth_basis(basis);
}

/* This is the MakeOrthoRep method from the spec.
   Note that in that bit of code the * operator is defined
   to be the cross product for the PrcPt objects
   The X basis vector is assumed to already be set but it may or may not
   already be normalize. Set a input flag to indicate if this code should
   normalize it (we want to avoid double normalization). */
int
prc_vec_make_orth_basis(prc_basis *basis)
{

    if (prc_vec_normalize(&basis->X) < 0)
        return PRC_VEC_ERROR;

    prc_vec_set(&basis->Y, 0, 1, 0);
    prc_vec_cross(basis->X, basis->Y, &basis->Z);
    if (prc_vec_normalize(&basis->Z) < 0)
    {
        prc_vec_set(&basis->Y, 1, 0, 0);
        prc_vec_cross(basis->X, basis->Y, &basis->Z);
        if (prc_vec_normalize(&basis->Z) < 0)
            return PRC_VEC_ERROR;
    }

    prc_vec_cross(basis->Z, basis->X, &basis->Y);
    if (prc_vec_normalize(&basis->Y) < 0)
        return PRC_VEC_ERROR;

    return 0;
}

/* Set triangle for left/right tree */
void
prc_vec_set_triangle(prc_vec3 v0, prc_vec3 v1, prc_vec3 v2, prc_vec3 normal, uint8_t edge, prc_compress_triangle *triangle, double norm_test, uint8_t null_children)
{
    if (null_children)
    {
        triangle->left = NULL;
        triangle->right = NULL;
    }
    prc_vec_copy(v0, &triangle->V0, 0);
    prc_vec_copy(v1, &triangle->V1, 0);
    prc_vec_copy(v2, &triangle->V2, 0);
    prc_vec_copy(normal, &triangle->normal, 0);

    triangle->neighbors = edge;

    /* norm test should either be 2 or 0 */
    if (norm_test < 1)
    {
        triangle->v1v2_is_left = 0;
    }
    else
    {
        triangle->v1v2_is_left = 1;
    }

    triangle->is_initialized = true;
}
