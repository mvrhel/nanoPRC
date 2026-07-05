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

#ifndef PRC_API_H
#define PRC_API_H

#include "../include/prc_context.h"

/**
 * @file prc_api.h
 * @brief Public C API for loading PRC/PDF content and extracting tessellation data.
 */

/** @defgroup public_api Public API
 *  @brief Public entry points for context management, loading, model tree creation,
 *  and tessellation/primitive extraction.
 *  @{
 */

#define PRC_API_ERROR_MEMORY -1
#define PRC_API_ERROR_PARAMETER -2
#define PRC_API_ERROR_PARSER -3
#define PRC_API_ERROR_UNSUPPORTED -4

/* Returned by prc_api_release_context when PRC_DEBUG_MEMORY is enabled
    and at least one allocation remains unfreed. */
#define PRC_API_MEMORY_LEAK_DETECTED 100

#define prc_api_data void *

#ifdef _WIN32
#define PRC_EXPORT __declspec(dllexport)
#elif !_WIN32 && PRC_BUILD_SHARED
#define PRC_EXPORT __attribute__((visibility("default")))
#else
#define PRC_EXPORT
#endif

typedef enum
{
    PRC_API_TESS_UNKNOWN = 0,
    PRC_API_TESS_3D,
    PRC_API_TESS_3D_Compressed,
    PRC_API_TESS_3D_Wire,
    PRC_API_TESS_3D_Wire_Extra,
    PRC_API_TESS_MarkUp
} prc_api_test_type_t;

typedef enum
{
    PRC_API_INTEGER_ATTRIBUTE = 1,
    PRC_API_DOUBLE_ATTRIBUTE,
    PRC_API_VALUE_SECS_INTEGER_ATTRIBUTE,
    PRC_API_STRING_ATTRIBUTE,
    PRC_API_VALUE_TIME_ATTRIBUTE
} prc_api_attribute_type_t;

typedef enum
{
    PRC_API_NODE_UNKNOWN = 0,
    PRC_API_NODE_PRODUCT,
    PRC_API_NODE_PART,
    PRC_API_NODE_PRODUCT_WITH_PART,
    PRC_API_NODE_MARKUP
} prc_api_node_type_t;

typedef enum
{
    PRC_API_TRIANGLES = 0,
    PRC_API_FAN,
    PRC_API_STRIP,
    PRC_API_LINE,
    PRC_API_LINE_STRIP,
    PRC_API_LINE_LOOP
} prc_api_graphic_type_t;

typedef enum
{
    PRC_API_TEXT_NO_MODE = -1,
    PRC_API_TEXT_FACE_VIEW_MODE = 0,
    PRC_API_TEXT_FRAME_DRAW_MODE = 1,
    PRC_API_TEXT_FIXED_SIZE_MODE = 2
} prc_api_text_block_mode_t;

typedef struct prc_api_material_s
{
    float emissive[3];
    float diffuse[3];
    float specular[3];
    float ambient[3];
    float shininess;
    float emissive_alpha;
    float diffuse_alpha;
    float specular_alpha;
    float ambient_alpha;
} prc_api_material;

typedef struct prc_api_vertex_s
{
    float position[3];
    float normal[3];
    float color[4];
    float uv[2];
    uint8_t normal_set; /* 0 = not set, 1 = set */
    uint8_t uv_set;     /* 0 = not set, 1 = set */

    /* These are values that will only be used in the compressed tessellation
       case as triangles can have individual styles. */
    uint8_t tri_has_material;
    int32_t style_index; /* Biased index for when we are finding and splitting vertices */
    int32_t style_file_index; /* File index for when we are finding and splitting vertices */
    float diffuse[3]; /* This will be used for the diffuse color if there is a style */
    float tint[3];    /* This will be used for the tint color if there is a style */
    float specular[3]; /* This will be used for the specular color if there is a style */
    float emissive[3]; /* This will be used for the emissive color if there is a style */
    float shininess; /* This will be used for the shininess if there is a style */
    float alpha;
} prc_api_vertex;

typedef struct prc_api_texture_s
{
    uint32_t height;
    uint32_t width;
    uint32_t num_channels;
    uint8_t has_transform;
    double transform[9];
    unsigned char *data;
} prc_api_texture;

typedef struct prc_api_tess_vertex_buffer_s
{
    size_t num_vertices;
    size_t capacity;
    prc_api_vertex *vertices;
} prc_api_tess_vertex_buffer;

/* The default style.  reserved has the details for the actual face */
typedef struct prc_api_face_s
{
    size_t num_graphic_primitives;
    uint8_t is_material;
    uint8_t has_transparency;
    uint8_t is_texture;
    prc_api_texture texture;
    prc_api_material material;
    uint8_t vertices_have_style;
    uint8_t face_has_single_style;
    uint32_t single_style_index; /* Only valid if face_has_single_style */
    prc_api_tess_vertex_buffer face_vertices; /* Only valid in the PRC_API_TESS_3D case */
    void *reserved;
    uint8_t disable_face;
} prc_api_face;

typedef struct prc_api_transform_s
{
    double matrix[16];
    uint8_t is_identity;
} prc_api_transform;

typedef struct prc_api_text_primitive_s
{
    char *text;
    double text_height;
    double text_width;
    double origin[3];
    unsigned char *font;
    double color[4];
    prc_api_text_block_mode_t mode;
} prc_api_text_primitive;

/* A tessellation can have a material associated with it. But if the tessellation
   has faces, the material associated with the face, will contain the material
   that should be used with that face. Wires (e.g. lines) do not have faces but
   their color can be defined by a material. Also, it turns out that we can
   have different realizations of the same tessellation with different styles */
typedef struct prc_api_tess_s
{
    prc_api_test_type_t type;
    uint8_t has_transparency;
    size_t num_faces;
    size_t num_line_primitives;
    size_t num_text_primitives;
    prc_api_text_primitive *text_primitives;
    prc_api_face *tess_faces;
    prc_api_tess_vertex_buffer tess_vertices;
    double bounding_box_min[3];
    double bounding_box_max[3];
    const char *name;
    int tess_index;
    int part_index;
    int product_index;
    int mark_up_index;
    uint8_t is_material;
    prc_api_material tess_material;
    void *reserved;
    void *reserved2;
    void *style_leaf;
} prc_api_tess;

typedef struct prc_api_inherited_style_s
{
    uint8_t has_inheritance;
    uint32_t inherited_file_index;
    uint32_t inherited_value;
} prc_api_inherited_style;

typedef struct prc_api_entity_ref_s
{
    uint32_t biased_layer_index;
    uint32_t biased_index_of_line_style;
    uint8_t behavior_bit_field1;
    uint8_t behavior_bit_field2;
    uint32_t file_index;
} prc_api_entity_ref;

typedef struct prc_api_attribute_entry_s prc_api_attribute_entry;
struct prc_api_attribute_entry_s
{
    char *entry_title; /* Have not seen this set to anything other than NULL. This would be if the base_attribute had multiple entries */
    prc_api_attribute_type_t type;

    union
    {
        int value_integer;
        double value_double;
        uint32_t value_secs_integer;
        char *value_string;
        uint64_t value_time;
    };
};

typedef struct prc_api_attribute_base_s prc_api_attribute_base;
struct prc_api_attribute_base_s
{
    char *attribute_base_title;
    size_t num_attributes; /* This is almost always 1. */
    prc_api_attribute_entry *attributes;
};

typedef struct prc_api_attributes_s prc_api_attributes;
struct prc_api_attributes_s
{
    uint32_t num_base_attributes;
    prc_api_attribute_base *base_attributes;
};

/* A rep item can have a rep item. For example, an RI SET type has rep items.
   We will just treat all of these as a type of part as it simplifies the code
   */
typedef struct prc_api_part_s prc_api_part;
typedef struct prc_api_object_style_s prc_api_object_style;
struct prc_api_part_s
{
    char *name;
    uint8_t name_same_as_product;
    prc_api_tess *tess;
    prc_api_tess *tess_line;
    uint32_t biased_tess_index;
    uint32_t tess_file_index;
    uint32_t biased_style_index;
    uint32_t biased_layer_index;
    uint8_t behavior_bit_field1;
    uint8_t behavior_bit_field2;
    uint32_t biased_local_coordinate_index;
    size_t num_rep_items;
    prc_api_part *rep_items;
    uint8_t has_inherited_style;
    prc_api_inherited_style color;
    prc_api_inherited_style transparency;
    uint32_t part_detail_index; /* Index into tess_style_file_part *part_details in prc_data_s */
    uint8_t has_entity_ref;
    prc_api_entity_ref entity_ref;
    prc_api_object_style *RI_item_style_node; /* This is a pointer to the RI style tree leaf. We can traverse this backwards to get everything */
    prc_api_attributes attributes;
};

typedef struct prc_api_markup_s
{
    char *name;
    prc_api_tess *tess;
    uint32_t biased_tess_index;
    uint32_t biased_style_index;
    uint32_t file_index;
} prc_api_markup;

typedef struct prc_api_product_s prc_api_product;
struct prc_api_product_s
{
    uint8_t is_model;
    prc_api_node_type_t type;
    size_t num_children;
    prc_api_product *children;
    uint8_t location_set;
    prc_api_transform location;
    char *name;
    prc_api_part *part;
    uint32_t num_markups;
    prc_api_markup *markup;
    int32_t file_index;
    prc_api_attributes attributes;
    void *reserved;
};

typedef struct prc_api_graphic_primitive_s
{
    prc_api_graphic_type_t type;
    size_t num_indices;
    uint32_t *indices;
} prc_api_graphic_primitive;

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Create a new API context.
 *
 * @param hooks Optional allocation/logging hooks; pass NULL for defaults.
 * @return A valid context pointer on success, or NULL on failure.
 */
PRC_EXPORT prc_context *prc_api_new_context(const prc_hooks *hooks);

/**
 * @brief Release a context and all resources owned by it.
 *
 * @param ctx Context returned by prc_api_new_context.
 * @return 0 on success. May return PRC_API_MEMORY_LEAK_DETECTED when leak
 * detection is enabled and unreleased allocations remain.
 */
PRC_EXPORT int prc_api_release_context(prc_context *ctx);

/**
 * @brief Print and clear the current context error stack.
 *
 * @param ctx Active API context.
 */
PRC_EXPORT void prc_api_print_error_stack(prc_context *ctx);

/**
 * @brief Open input contents (PRC or supported container) and prepare API data.
 *
 * @param ctx Active API context.
 * @param infile Input file path.
 * @return Opaque API data handle on success, NULL on failure.
 */
PRC_EXPORT prc_api_data prc_api_open_contents(prc_context *ctx, const char *infile);

/**
 * @brief Release API data and optional tessellation/model-tree allocations.
 *
 * @param ctx Active API context.
 * @param data Data handle returned by prc_api_open_contents.
 * @param tess Optional tessellation array previously allocated by caller.
 * @param num_tess Number of entries in tess.
 * @param line_tess Optional line tessellation array previously allocated by caller.
 * @param num_line_tess Number of entries in line_tess.
 * @param product_tree Optional model tree previously allocated by caller.
 */
PRC_EXPORT void prc_api_release_data(prc_context *ctx, prc_api_data data, prc_api_tess *tess, uint32_t num_tess, prc_api_tess *line_tess, uint32_t num_line_tess, prc_api_product *product_tree);

/**
 * @brief Print an API product tree for diagnostics.
 *
 * @param ctx Active API context.
 * @param product Product subtree root to print.
 * @param level Initial indentation level.
 */
PRC_EXPORT void prc_api_print_tree(prc_context *ctx, prc_api_product *product, int level);

/**
 * @brief Return number of stored camera views.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @return Number of views in the data set.
 */
PRC_EXPORT uint32_t prc_api_get_number_of_view(prc_context *ctx, prc_api_data data);

/**
 * @brief Fetch view metadata and transform for one view index.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param view_index Zero-based view index.
 * @param name Output view name.
 * @param matrix Output 4x4 transform matrix pointer.
 * @param camera_z Output camera distance along z.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_get_view(prc_context *ctx, prc_api_data data, uint32_t view_index, char **name, double **matrix, double *camera_z);

/**
 * @brief Set a transform structure to identity.
 *
 * @param ctx Active API context.
 * @param transform Transform object to initialize.
 */
PRC_EXPORT void prc_api_set_transform_identity(prc_context *ctx, prc_api_transform *transform);

/**
 * @brief Concatenate/apply a new transform into an existing transform.
 *
 * @param ctx Active API context.
 * @param concate_transform In/out accumulated transform.
 * @param new_transform Transform to apply.
 */
PRC_EXPORT void prc_api_update_transform(prc_context *ctx, prc_api_transform *concate_transform, prc_api_transform *new_transform);

/**
 * @brief Get tessellation pointer for a part representation item.
 *
 * @param ctx Active API context.
 * @param part Part object.
 * @param rep_item_index Representation-item index.
 * @return Tessellation pointer, or NULL when unavailable.
 */
PRC_EXPORT prc_api_tess *prc_api_get_ri_tessellation(prc_context *ctx, prc_api_part *part, uint32_t rep_item_index);

/**
 * @brief Get optional extra wire tessellation pointer for a representation item.
 *
 * @param ctx Active API context.
 * @param part Part object.
 * @param rep_item_index Representation-item index.
 * @return Extra wire tessellation pointer, or NULL when unavailable.
 */
PRC_EXPORT prc_api_tess *prc_api_get_ri_line_tessellation(prc_context *ctx, prc_api_part *part, uint32_t rep_item_index);

/**
 * @brief Get a model-level tessellation associated with a product node.
 *
 * @param ctx Active API context.
 * @param parent Product node.
 * @return Tessellation pointer, or NULL when unavailable.
 */
PRC_EXPORT prc_api_tess* prc_api_get_model_tessellation(prc_context* ctx, prc_api_product *parent);

/**
 * @brief Get markup tessellation by index.
 *
 * @param ctx Active API context.
 * @param product Product node containing markups.
 * @param markup_index Markup index.
 * @return Markup tessellation pointer, or NULL when unavailable.
 */
PRC_EXPORT prc_api_tess* prc_api_get_markup_tessellation(prc_context *ctx, prc_api_product *product, uint32_t markup_index);

/**
 * @brief Check whether a model item corresponds to a part.
 *
 * @param ctx Active API context.
 * @param parent Model node.
 * @return Non-zero if node represents a part.
 */
PRC_EXPORT uint8_t prc_api_model_item_is_part(prc_context *ctx, prc_api_product *parent);

/**
 * @brief Return number of markups attached to a model item.
 *
 * @param ctx Active API context.
 * @param api_product Product/model node.
 * @return Number of markups attached to the node.
 */
PRC_EXPORT uint32_t prc_api_model_item_number_of_markups(prc_context *ctx, prc_api_product *api_product);

/**
 * @brief Initialize one tessellation entry and optional extra-line companion.
 *
 * @param ctx Active API context.
 * @param data_in Data handle from prc_api_open_contents.
 * @param model_tree Model tree created by prc_api_create_model_tree.
 * @param tess_index Tessellation index in prepared model data.
 * @param api_tess Output tessellation object.
 * @param api_tess_line Optional output for extra line tessellation.
 * @param has_line Output flag indicating whether extra line tessellation exists.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_initialize_tessellation(prc_context* ctx, prc_api_data data_in, prc_api_product *model_tree, uint32_t tess_index, prc_api_tess *api_tess, prc_api_tess *api_tess_line, uint8_t *has_line);

/**
 * @brief Get counts of tessellations and extra line tessellations.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param modeltree Model tree root.
 * @param num_tess Output count of tessellations.
 * @param num_line_tess Output count of line tessellations.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_get_number_tessellations(prc_context *ctx, prc_api_data data, prc_api_product *modeltree, uint32_t *num_tess, uint32_t *num_line_tess);

/**
 * @brief Return number of faces for a tessellation.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param tess_index Tessellation index.
 * @return Number of faces for the tessellation.
 */
PRC_EXPORT uint32_t prc_api_get_number_faces(prc_context *ctx, prc_api_data data, uint32_t tess_index);

/**
 * @brief Build face wire primitive data for one tessellation face.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param api_tree Model tree root.
 * @param tess_index Tessellation index.
 * @param face_index Face index in tessellation.
 * @param face_out Output face metadata.
 * @param tess_line Output line tessellation data.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_get_line_tessellation_vertices(prc_context *ctx, prc_api_data data, prc_api_product *api_tree, uint32_t tess_index, uint32_t face_index, prc_api_face *face_out, prc_api_tess *tess_line);

/**
 * @brief Build tessellation vertices/primitives for one tessellation face.
 *
 * @param ctx Active API context.
 * @param data_in Data handle.
 * @param api_tree Model tree root.
 * @param tess_index Tessellation index.
 * @param face_index Face index in tessellation.
 * @param face_out Output face metadata.
 * @param api_tess Output tessellation data.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_get_tessellation_vertices(prc_context *ctx, prc_api_data data_in, prc_api_product *api_tree, uint32_t tess_index, uint32_t face_index, prc_api_face *face_out, prc_api_tess *api_tess);

/**
 * @brief Get one graphics primitive descriptor by index.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param tess Tessellation object.
 * @param face_index Face index.
 * @param graphics_primitive_index Primitive index for the face.
 * @param graphics_primitive Output primitive descriptor.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_get_graphics_primitive(prc_context *ctx, prc_api_data data, const prc_api_tess *tess, uint32_t face_index, size_t graphics_primitive_index, prc_api_graphic_primitive *graphics_primitive);

/**
 * @brief Return number of graphics primitives for a tessellation object.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param tess Tessellation object.
 * @return Number of graphics primitives available.
 */
PRC_EXPORT size_t prc_api_get_num_graphics_primitives(prc_context *ctx, prc_api_data data, const prc_api_tess *tess);

/**
 * @brief Return whether a face uses material properties.
 *
 * @param ctx Active API context.
 * @param api_tess Tessellation object.
 * @param face_index Face index.
 * @return Non-zero when material data is present.
 */
PRC_EXPORT int prc_api_face_is_material(prc_context *ctx, const prc_api_tess *api_tess, uint32_t face_index);

/**
 * @brief Return whether a face should be skipped for rendering.
 *
 * @param ctx Active API context.
 * @param api_tess Tessellation object.
 * @param face_index Face index.
 * @return Non-zero when face should be skipped.
 */
PRC_EXPORT uint8_t prc_api_skip_face(prc_context *ctx, const prc_api_tess *api_tess, uint32_t face_index);

/**
 * @brief Get material values for a tessellation face.
 *
 * @param ctx Active API context.
 * @param api_tess Tessellation object.
 * @param material Output material payload.
 * @param face_index Face index.
 */
PRC_EXPORT void prc_api_get_face_material(prc_context *ctx, const prc_api_tess *api_tess, prc_api_material *material, uint32_t face_index);

/**
 * @brief Get one text primitive from a tessellation.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param tess Tessellation object.
 * @param text_index Text primitive index.
 * @param text_primitive Output text primitive.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_get_text_primitive(prc_context *ctx, prc_api_data data, const prc_api_tess *tess, uint32_t text_index, prc_api_text_primitive *text_primitive);

/**
 * @brief Return number of materials represented by a tessellation.
 *
 * @param ctx Active API context.
 * @param data_in Data handle.
 * @param tess Tessellation object.
 * @return Number of materials represented.
 */
PRC_EXPORT uint32_t prc_api_number_of_materials(prc_context *ctx, prc_api_data data_in, const prc_api_tess *tess);

/**
 * @brief Return whether per-vertex material/style payload is present on a face.
 *
 * @param ctx Active API context.
 * @param api_tess Tessellation object.
 * @param face_index Face index.
 * @return Non-zero when vertices carry style/material data.
 */
PRC_EXPORT uint8_t prc_api_vertices_have_material(prc_context *ctx, const prc_api_tess *api_tess, uint32_t face_index);

/**
 * @brief Get expanded face vertices for a tessellation face.
 *
 * @param ctx Active API context.
 * @param tess Tessellation object.
 * @param face_index Face index.
 * @param vertex_count Output number of expanded vertices.
 * @param vertices Output allocated vertex array.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_get_face_vertices(prc_context *ctx, const prc_api_tess *tess, uint32_t face_index, uint32_t *vertex_count, prc_api_vertex **vertices);

/**
 * @brief Precompute counts needed to allocate model tree arrays.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param num_parts Output part count.
 * @param num_products Output product count.
 * @param num_markups Output markup count.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_prep_model_tree(prc_context *ctx, prc_api_data data, uint32_t *num_parts, uint32_t *num_products, uint32_t *num_markups);

/**
 * @brief Build the API model tree from parsed data.
 *
 * @param ctx Active API context.
 * @param data Data handle.
 * @param parent Output root pointer for the created model tree.
 * @param num_parts Preallocated part capacity from prc_api_prep_model_tree.
 * @param num_products Preallocated product capacity from prc_api_prep_model_tree.
 * @param num_markups Preallocated markup capacity from prc_api_prep_model_tree.
 * @return 0 on success, negative PRC_API_ERROR_* code on failure.
 */
PRC_EXPORT int prc_api_create_model_tree(prc_context *ctx, prc_api_data data, prc_api_product **parent, uint32_t num_parts, uint32_t num_products, uint32_t num_markups);
#ifdef __cplusplus
}
#endif

/** @} */

#endif
