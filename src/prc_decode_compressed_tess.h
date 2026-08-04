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

#ifndef PRC_DECODE_H
#define PRC_DECODE_H

#include "prc_data.h"

#define PRC_INITIAL_MULTI_NORMAL_CAPACITY 5
#define PRC_INITIAL_TREATED_EDGE_LIST_CAPACITY 5
#define PRC_ARG_ACOS_MAX (1.0 - 1.0e-12)
#define PRC_ARG_ACOS_MIN (-1.0 + 1.0e-12)
#define PRC_NORMAL_EPSILON 1.0e-4
#define PRC_NORMAL_AVG_EPSILON 0.5

typedef struct treated_triangle_s treated_triangle;
typedef struct prc_treated_details_s prc_treated_details;
typedef struct prc_triangle_stack_s prc_triangle_stack;
typedef struct comp_edge_s comp_edge;
typedef struct edge_list_comp_s edge_list_comp;
typedef struct treated_edge_list_s treated_edge_list;
typedef struct edge_entry_s edge_entry;
typedef struct prc_normal_state_s prc_normal_state;
typedef struct decoded_normal_info_s decoded_normal_info;
typedef struct treated_edge_s treated_edge;

typedef enum {
    PRC_EDGE_NOT_TREATED = 0,
    PRC_EDGE_TREATED = 1
} prc_edge_treated_case_t;

typedef enum {
    PRC_EDGE_UNDEFINED = 0,
    PRC_EDGE_LEFT = 1,
    PRC_EDGE_RIGHT = 2
} prc_edge_case_t;

typedef enum {
    PRC_VERTEX_NORM_NOT_ENCOUNTERED = 0,
    PRC_VERTEX_NORM_IS_MULTIPLE = 1,
    PRC_VERTEX_NORM_IS_NOT_MULTIPLE = 2
} prc_vertex_norm_case_t;

struct prc_normal_state_s
{
    prc_vec3 normal;
    prc_vec3 averaged_normal;
    uint8_t must_calculate_normals;
    uint8_t face_is_planar;
    uint8_t *face_normal_decoded;
    prc_vec3 *face_normals;
    uint32_t *face_normal_indices;
    prc_vec3 *normals_vertex;  /* These are the normals stored for each vertex */
    uint32_t normals_vertex_count; /* Counter for the above */
    uint8_t *normal_bin_data;
    uint32_t normal_bin_data_index;
    double *decoded_angles;
    uint32_t angle_index;
    uint8_t *normal_is_reversed;
    decoded_normal_info *multiple_normals;
};

struct edge_entry_s
{
    uint32_t indice0;
    uint32_t indice1;
    prc_edge_treated_case_t edge_status;
};

/* treated_edge_list_s structure allows us to do a faster search for edges that
   have already been treated as we make our way through the stack. The issue is
   that when we pop the edge off the stack, we have to make sure that it was not
   already handled. If it was, we pop the next one.  If a treated triangle does
   not have a left or a right neighbor that edge with no neighbor is set as having
   been treated and added to this treated edge list. The location where it is added
   is based upon the smallest index of the two. That smallest index gets us to
   another list of second indices that share that first indice.  In this way
   we can test more quickly */
struct treated_edge_s
{
    uint32_t num_second_indices;
	uint32_t capacity;
    uint32_t indice0;
	uint32_t *indice1;
};

struct treated_edge_list_s
{
	uint32_t capacity;
	treated_edge *edge;
};

struct edge_list_comp_s
{
    uint32_t capacity;
    uint32_t num_edges;
    edge_entry *edge;
};

struct comp_edge_s
{
    uint32_t edge_treatement_x;
    uint32_t edge_treatement_y;
    uint32_t edge_treatement_z; /* Used for computation of basis */
    prc_edge_treated_case_t edge_status;
};

/* This is used to manage the normal_binary_data reference to normals when
   a vertex has multiple normals */
struct decoded_normal_info_s
{
    prc_vertex_norm_case_t vertex_normal_state;
    uint32_t num_already_stored_normals_on_vertex;
    uint32_t normal_indice_capacity;
    uint32_t *normal_indices;
    uint32_t non_multiple_normal_index;
};

struct treated_triangle_s
{
     /* triangle_base_ordered_indices[3] should define the triangle winding
        following the normal with the first two as the base. Handy for left/right
        edge determination. These are often different than the treated_index which
        indentify the vertices as given by the point[3] which relate to the vertices
        array where we store them (given by treated_index[3]). triangle_base_ordered_indices
        are always numbers 0, 1, 2 with the first two giving the position of the base
        along the normal winding. So for example, we may have 2, 0, 1 as the values
        which tells us that the triangle winding is V2, V0, V1 where V2, V0 is the base,
        V0 V1 is the right edge, V2 V2 is the left edge */
    prc_vec3 points[3];
    int treated_index[3];
    int normal_indices[3];  /* Not used for face case */
    int uses_reference;
    uint8_t normal_was_reversed;
    uint8_t face_was_reversed;  /* In vertex mode uses 3x averaging to determine */
    uint32_t triangle_index;
    comp_edge left_edge;
    comp_edge right_edge;
};

struct prc_treated_details_s
{
    prc_vec3 x_basis;
    prc_vec3 y_basis;
    prc_vec3 z_basis;
    prc_vec3 origin;
    prc_vec3 V0;
    prc_vec3 V1;
    int index0;
    int index1;
    int normal_index0;  /* Not used for face case */
    int normal_index1;  /* Not used for face case */
    prc_edge_case_t edge_case;
    prc_treated_details *next;
};

struct prc_triangle_stack_s
{
    prc_treated_details *details;
    uint32_t size;
};

typedef enum {
    PRC_COMPRESSED_DECODE_SHARED_SIDE_UNDEFINED = 0,
    PRC_COMPRESSED_DECODE_SHARED_SIDE_01 = 1,
    PRC_COMPRESSED_DECODE_SHARED_SIDE_12 = 2,
    PRC_COMPRESSED_DECODE_SHARED_SIDE_20 = 3
} prc_shared_side_case_t;


typedef struct prc_edge_stack_comp_s prc_edge_stack_comp;

/* In support of apply crease angle */
typedef enum {
    PRC_COMPRESSED_TESS_EDGE_01 = 0,
    PRC_COMPRESSED_TESS_EDGE_02,
    PRC_COMPRESSED_TESS_EDGE_12
} prc_compressed_tess_edge_check_case_t;

/* Edge list supporting stuctures */
typedef struct prc_compressed_tess_edge_s
{
    uint32_t tri_one_full_indices[3];
    uint32_t tri_two_full_indices[3];
    uint32_t tri_one_edge_indices[2];
    uint32_t tri_two_edge_indices[2];
    prc_compressed_tess_edge_check_case_t tri_one_edge_case;
    prc_compressed_tess_edge_check_case_t tri_two_edge_case;
    uint32_t tri_one_indices_offset;  /* Offset into the normal and the vertex indices */
    uint32_t tri_two_indices_offset;  /* Offset into the normal and the vertex indices */
    uint32_t num_triangles;
} prc_compressed_tess_edge;

typedef struct prc_compressed_tess_edge_list_s
{
    size_t num_edges;
    size_t capacity;
    prc_compressed_tess_edge *edge;
} prc_compressed_tess_edge_list;

int prc_decode_compressed_tess(prc_context *ctx, prc_tess_3d_compressed *data, uint8_t debug_tess);
int prc_compressed_tess_apply_crease_angle(prc_context *ctx, prc_tess_3d_compressed *data);
int prc_compute_edges_compressed_tess(prc_context *ctx, prc_tess_3d_compressed *data);

typedef struct norms_and_edge_indices_s norms_and_edge_indices;
struct norms_and_edge_indices_s
{
    prc_vec3 normal;
    uint32_t edge_index;
    norms_and_edge_indices *next;
};

typedef struct norm_details_s norm_details;
struct norm_details_s
{
    uint32_t num_norms_to_avg; /* Number of normals to average */
    norms_and_edge_indices *norm_to_avg; /* List of normals to average */
    prc_vec3 averaged_normal;
    norm_details *next;
};

typedef struct triangle_to_edge_entry_s triangle_to_edge_entry;
struct triangle_to_edge_entry_s
{
    uint8_t num_edges;
    uint32_t edge_indexes[3];
    prc_vec3 normal;
    uint32_t vertex_indices[3];
    uint8_t was_processed;
    uint32_t new_normal_indices[3];
};

typedef struct triangle_to_edge_entry2_s triangle_to_edge_entry2;
struct triangle_to_edge_entry2_s
{
    uint8_t num_edges;
    uint32_t edge_indexes[3];
    prc_vec3 normal;
    uint32_t vertex_indices[3];
    uint8_t was_processed[3]; /* Associated with each edge */
    uint32_t new_normal_indices[3];
};

/* A super structure so that we can modularize things more easily */
typedef struct prc_ss_crease_compressed_s
{
    uint32_t num_vertices;
    uint32_t norms_and_edge_indices_heap_capacity;
    uint32_t norms_and_edge_indices_heap_index;
    norms_and_edge_indices *norms_and_edge_indices_heap;
    uint32_t norm_details_heap_capacity;
    uint32_t norm_details_heap_index;
    norm_details *norm_details_heap;
    prc_vec3 *current_normal;
    uint32_t edge_index; /* Index into the edge list */
    uint32_t vertex_index; /* Index into the vertex list */
    uint8_t triangle_index; /* either 1 or 2 if we are doing single triangle settings -- or cases where we are greater than the crease angle */
    norm_details **vertex_list_of_norm_details;
    uint32_t num_triangles;

} prc_ss_crease_compressed;


#endif
