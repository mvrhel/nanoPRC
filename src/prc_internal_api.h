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

#ifndef PRC_API_INTERNAL_H
#define PRC_API_INTERNAL_H

#include <stdint.h>
#include "prc_fwd.h"
#include "prc_api.h"

typedef struct prc_internal_api_vertex_s
{
    float position[3];
} prc_internal_api_vertex;

typedef struct prc_internal_api_normal_s
{
    float normal[3];
} prc_internal_api_normal;

/* Need to see if the number of texture coords is ever greater than 1. Right now we
   will throw and error if it is */
#define MAX_NUM_TEXTURE_COORDS 1
typedef struct prc_internal_api_texture_indices_s
{
    float texture[MAX_NUM_TEXTURE_COORDS];
} prc_internal_api_texture_indices;

/* This is to allow for a faster look-up of the api vertex based
   upon the index into the normal and position of the prc data.
   Essentially an array of the same size as the prc positions
   vertices but at each position we have a linked list that
   we can go through to check for a normal vector and get the
   index into the api vertex data that has that position and normal vector AND
   texture vertices if theu are being used */
typedef struct prc_internal_api_position_normal_pair_s prc_internal_api_position_normal_pair;

/* Define a state for the normal vector. We have three possiblities. Either it
   is already set, it is not set but we CAN set it, or it is not set and must be
   calculated. Also keep a state for the texture, color, and style of the
   vertex. */
typedef enum {
    PRC_INTERNAL_API_NORM_MUST_BE_COMPUTED = -1,
    PRC_INTERNAL_API_NORM_NOT_SET,
    PRC_INTERNAL_API_NORM_SET
} prc_internal_api_normal_state_t;

typedef enum {
    PRC_INTERNAL_API_NO_TEXTURE = -1,
    PRC_INTERNAL_API_TEXTURE_NOT_SET,
    PRC_INTERNAL_API_TEXTURE_SET
} prc_internal_api_texture_state_t;

/* Note that the prc_position_index is also used to index */
typedef enum {
    PRC_INTERNAL_API_NO_COLOR = -1,
    PRC_INTERNAL_API_COLOR_NOT_SET,
    PRC_INTERNAL_API_COLOR_SET
} prc_internal_api_color_state_t;

typedef enum {
    PRC_INTERNAL_API_NO_STYLE = -1,
    PRC_INTERNAL_API_STYLE_NOT_SET_FROM_TESS,
    PRC_INTERNAL_API_STYLE_SET_FROM_TESS,
    PRC_INTERNAL_API_STYLE_NOT_SET_FROM_REF_DATA,
    PRC_INTERNAL_API_STYLE_SET_FROM_REF_DATA
} prc_internal_api_style_state_t;

typedef enum {
    PRC_INTERNAL_API_OBJECT_TYPE_PRODUCT,
    PRC_INTERNAL_API_OBJECT_TYPE_PART,
    PRC_INTERNAL_API_OBJECT_TYPE_RI
} prc_internal_api_object_type;

/* Special note on prc_vertex_color_index which is a member variable below. So if we
   are dealing with vertex colors then every vertex has a color. We of course have
   a list of indices into the vertices. The count into the indices array is the count
   that is used to get the color in the decode_array (which is where we store the
   vertex colors).  In searching for matching vertices we check for a match of the color
   not the indice though. */
struct prc_internal_api_position_normal_pair_s
{
    uint32_t prc_position_index;
    int32_t prc_normal_index;
    uint32_t prc_vertex_color_index; /* This is the count into the indices array */
    float vertex_color[4]; /* This is related to prc_vertex_color_index */
    uint32_t prc_texture_index[2];
    uint32_t prc_num_texture_indices; /* Usually 1. Would like to see something other than that */
    size_t api_vertex_index;
    int32_t style_index;
    int32_t face_style_file_index;
    prc_internal_api_normal_state_t normal_set;
    prc_internal_api_texture_state_t texture_set;
    prc_internal_api_color_state_t color_set;
    prc_internal_api_style_state_t style_set;
    prc_internal_api_position_normal_pair *next;
};

/* Forward declaration only -- the full struct (prc_internal_graph_style_s)
   is defined further down this file; only a pointer to it is needed here. */
typedef struct prc_internal_graph_style_s prc_internal_graph_style;

typedef struct prc_internal_api_position_normal_lookup_s
{
    size_t number_values;
    prc_internal_api_position_normal_pair *position_normal_pair;
    /* PRC_TYPE_TESS_3D_Compressed only: per-face-group style table
       (face_number entries), built once per api_tess INSTANCE by
       prc_api_get_tessellation_vertices' compressed branch and reused
       across every face_index call for that same instance. Lives here
       (not on the shared prc_tess struct) for the same reason the
       position_normal_pair cache above does -- see this file's own
       comment at the compressed branch's cache lookup site
       ("An earlier version of this optimization cached on the shared
       prc_tess struct instead... disc-brake-rendering-regression").
       Before this cache existed, the style table (O(face_number) work)
       was rebuilt from scratch on every single face_index call, which
       for the standard one-call-per-face walk every demo/binding uses
       (see demos/quick_start) cost O(face_number^2) time and -- because
       nothing freed a face's own copy until the whole walk finished --
       O(face_number^2) peak memory too. */
    prc_internal_graph_style *face_style_cache;
    uint32_t face_style_cache_count;
    uint8_t face_style_has_more_than_one_ref_style;
    uint8_t face_style_cache_valid;
    /* Distinguishes "this cache struct has been allocated" from "the
       position/normal portion of it has been built" -- both this cache
       and the face style cache above now share one struct/one
       api_tess->reserved slot, so `position_normal_pair != NULL` alone
       is no longer a safe signal that this part specifically was already
       built (position_normal_pair could also be NULL, e.g. calloc(0,...),
       if this part was built with zero vertices). */
    uint8_t position_normal_cache_valid;
} prc_internal_api_position_normal_lookup;

/* This structure is used to store the tessellation data for a given face.
   It is used to store the number of triangles, fans, and strips that are
   used to represent the face. It also stores the offsets into the vertex
   indices for the fans and strips. This is for a specific type. For example
   those entities that have a single norm, or a texture, or a multiple norm */
typedef struct prc_internal_api_tess_entities_s
{
    size_t num_triangles;
    size_t num_fans;
    size_t num_strips;
    size_t *fan_offsets;
    size_t *strip_offsets;
} prc_internal_api_tess_entities;

typedef struct prc_internal_texture_s prc_internal_texture;
struct prc_internal_texture_s
{
    /* PRC_TYPE_GRAPH_TextureApplication */
    int32_t material_generic_index;
    int32_t texture_definition_index;
    int32_t next_texture_index;
    int32_t uv_coordinates_index;
    int32_t picture_index;
    int32_t wrapping_u; /* Repeat in u */
    int32_t wrapping_v; /* Repeat in v */
    uint32_t picture_width;
    uint32_t picture_height;
    uint32_t num_elements_per_pixel;
    unsigned char *picture_data;
    uint8_t has_texture_transform;
    double texture_transform[9]; /* A 3x3 */
    prc_internal_texture *next_texture; /* We can have a pipeline of textures */
};

/* typedef already forward-declared above, near prc_internal_api_position_
   normal_lookup, which needs a pointer to this type before it's fully
   defined. */
struct prc_internal_graph_style_s
{
    double line_width;
    uint8_t is_vpicture;
    uint32_t biased_pattern_index;
    uint8_t is_material;
    uint32_t biased_color_index;
    uint8_t is_transparency;
    uint8_t transparency;
    uint32_t face_style_index;
    uint32_t face_style_file_index;
    float tint[4];

    prc_internal_texture texture;

    /* PRC_TYPE_GRAPH_Material */
    int32_t ambient_index;
    int32_t diffuse_index;
    int32_t emissive_index;
    int32_t specular_index;
    double shininess;
    double ambient_alpha;
    double diffuse_alpha;
    double emissive_alpha;
    double specular_alpha;
    uint32_t material_type;
    float ambient_color[3];
    float diffuse_color[3];
    float emissive_color[3];
    float specular_color[3];
};

/* In PRC a face can contain n triangles, m fans and p strips
   This structure gives a list of vertex indices for the num_triangles
   which are the first entries num_triangles*3 in the vertex_indice_list.
   If there are fans, num_fans > 0 and pairs of values in fan_offsets will
   give the offset into vertex_indices and how many points are used in vertex_indices.
   Same logic is used for strips.  Also note that while each face has one style,
   the compressed tessellation is forced by the API to have a single face but
   in reality can have multiple faces. Those are packed into the the
   number_of_styles, and *style information and used during the creation of the
   triangle vertices that we hand back */
typedef struct prc_internal_api_face_s
{
    size_t capacity;
    size_t num_indices;
    size_t num_texture_coords;
    uint32_t *vertex_indices;
    prc_internal_api_tess_entities single_norm;
    prc_internal_api_tess_entities multi_norm;
    prc_internal_api_tess_entities texture_single_norm;
    prc_internal_api_tess_entities texture_multi_norm;
    size_t number_of_styles;
    prc_internal_graph_style *style;
    /* 1 if `style` is this face's own allocation (the PRC_TYPE_TESS_3D
       branch of prc_api_get_tessellation_vertices, one entry) and must be
       freed (including its texture chain) when this face is released; 0
       if it's a borrowed pointer into a per-api_tess-instance cache
       shared by every face of a compressed tessellation (see
       prc_internal_api_position_normal_lookup's face_style_cache) --
       freed once, with the cache, not per-face. */
    uint8_t owns_style;
} prc_internal_api_face;

/* We have a similar set up for the 3D wire */
typedef struct prc_internal_api_wire_s
{
    size_t capacity;
    size_t num_indices;
    uint32_t *vertex_indices;
    prc_internal_graph_style style;
    prc_api_material material;
    prc_api_graphic_type_t type;
} prc_internal_api_wire;

typedef enum {
    PRC_INTERNAL_API_MULTINORM = 0,
    PRC_INTERNAL_API_SINGLENORM,
    PRC_INTERNAL_API_SINGLENORM_TEXTURE,
    PRC_INTERNAL_API_MULTINORM_TEXTURE,
    PRC_INTERNAL_API_MAX
} prc_internal_api_entity_t;

typedef struct prc_api_child_reserve_s
{
    uint32_t num_products;
    uint32_t num_parts;
    uint32_t num_markups;
    prc_api_product *products;
    prc_api_part *parts;
    prc_api_markup *markups;
    uint32_t markup_index;
    uint32_t product_index;
    uint32_t part_index;
    prc_api_object_style *style_tree_root;

    /* Preallocated pool of prc_api_object_style nodes.
       Caller should initialize the pool after computing the totals
       (num_products + num_parts + num_markups) by calling
       prc_internal_api_style_pool_init(...). */
    prc_api_object_style *style_pool;
    uint32_t style_pool_capacity; /* total entries allocated in style_pool */
    uint32_t style_pool_index;    /* next free slot index (0 .. style_pool_capacity-1) */
} prc_api_child_reserve;

/* This is a rework of the part style. We will capture all the product
   entity reference information here which will have information about
   the styles for each of the faces potentially. We will capture a tree
   of styles starting from the base product going through the product prototypes
   all the way to the part and the representation item (RI). RI have
   tessellation data associated with them. When we process the RI, we
   will want to be able to traverse back up the tree to get the style
   information for the face that we are processing. Products can have multiple
   entity references which specify styles for different RIs. They can
   also specify styles for specific faces of those RIs. We want to build
   a tree of object styles that we can traverse forward AND backward. We will
   build it in the forward direction as we go from the base product through
   the product prototypes to a part and finally a RI (which is actually a part).*/

/* Define the structure so we can self reference in the tree forward and backwards
   We only store the style index and the file index and the array of prc_misc_entity_reference
   items that may have been defined for that product. */

/* This will be much cleaner and enable us to decode things later when we
   do the drawing of the tessellation */

struct prc_api_object_style_s
{
    prc_internal_api_object_type object_type;
    uint32_t brep_type;
    uint32_t index_of_topological_index;  /* Used only with Brep RI */
    uint32_t index_of_body;               /* Used only with Brep RI */
    uint32_t file_index;  /* File index of product/part/RI */
    prc_base_with_graphics *base_with_graphics;
    uint32_t num_entity_references;
    prc_misc_entity_reference *entity_references;
    prc_api_object_style *parent;
    prc_api_object_style **children;
    uint32_t num_children;
};

/* Internal accessors for prc_api_tess::reserved and prc_api_face::reserved.
   Both fields are void* in the public API but always hold one of two
   internal types at runtime, never both:
     - prc_internal_api_face * for a face-bearing tessellation/face
     - prc_internal_api_wire * for a wire/line/markup tessellation, or for
       a face slot repurposed to carry line-primitive (wire) data
   These replace the bare (prc_internal_api_face *)/(prc_internal_api_wire *)
   casts -- and equivalent uncast void*-to-typed-pointer assignments -- that
   were scattered across prc_api.c and prc_tri_primitives_api.c, so the
   release logic in prc_api_release_data can be audited by reading which
   helper is called rather than by re-deriving the cast's intent each time.
   The caller is still responsible for knowing which interpretation applies
   at a given call site (usually via the tessellation/face type); these are
   plain accessors, not a tagged union. prc_api_tess::reserved2 is a
   separate, unrelated field (currently holding prc_internal_graph_style*,
   always NULL in the current codebase) and is out of scope here. */
static PRC_INLINE prc_internal_api_face *
prc_tess_internal_face(const prc_api_tess *t)
{
    return (prc_internal_api_face *)t->reserved;
}

static PRC_INLINE prc_internal_api_wire *
prc_tess_internal_wire(const prc_api_tess *t)
{
    return (prc_internal_api_wire *)t->reserved;
}

static PRC_INLINE prc_internal_api_face *
prc_face_internal_face(const prc_api_face *f)
{
    return (prc_internal_api_face *)f->reserved;
}

static PRC_INLINE prc_internal_api_wire *
prc_face_internal_wire(const prc_api_face *f)
{
    return (prc_internal_api_wire *)f->reserved;
}

/* prc_api_tess::reserved, interpreted as the compressed-tessellation
   per-vertex dedup cache (prc_api_get_tessellation_vertices). Scoped to
   ONE api_tess instance -- i.e. one caller-owned prc_api_tess, built on
   its first face_index call and reused by that SAME instance's later
   faces -- never shared across different api_tess instances that happen
   to reference the same underlying PRC tess_index, since those can
   legitimately carry different per-instance styles (see the comment on
   prc_api_tess itself: "we can have different realizations of the same
   tessellation with different styles"). Mutually exclusive with the
   wire/markup branches' own use of this same field (api_tess->type
   determines which interpretation applies). */
static PRC_INLINE prc_internal_api_position_normal_lookup *
prc_tess_internal_position_normal_lookup(const prc_api_tess *t)
{
    return (prc_internal_api_position_normal_lookup *)t->reserved;
}

#endif
