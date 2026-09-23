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

#ifndef PRC_WRITE_TREE_H
#define PRC_WRITE_TREE_H

#include "prc_write_global.h"

#include "prc_write_common.h"
#include "prc_data.h"
#include "prc_bit.h"

/* Encoder for PRC_TYPE_ASM_FileStructureTree (Table 47): the product/part
   assembly tree, the exact inverse of prc_parse_file_tree (which dispatches
   to prc_parse_parts / prc_parse_product_occurrence in prc_parse_tree.c).

   prc_write_ri_kind, prc_write_rep_item, and prc_write_tree_node are
   aliases of the public prc_api_write_ri_kind_t / prc_api_write_rep_item /
   prc_api_write_node (include/prc_api.h) -- see those types' doc comments
   for field semantics. Aliased rather than redefined so the internal
   encoder and the public API can never drift apart. A representation item
   wraps one tessellation-section entry: PRC_WRITE_RI_SURFACE writes
   PRC_TYPE_RI_PolyBrepModel (used for tessellated surface geometry -- NOT
   exact B-Rep, which is out of scope), PRC_WRITE_RI_WIRE writes
   PRC_TYPE_RI_PolyWire (line/polyline geometry). */
typedef prc_api_write_ri_kind_t prc_write_ri_kind;
#define PRC_WRITE_RI_SURFACE PRC_API_WRITE_RI_SURFACE
#define PRC_WRITE_RI_WIRE PRC_API_WRITE_RI_WIRE

typedef prc_api_write_rep_item prc_write_rep_item;

/* The tree is walked iteratively (explicit heap work stack): depth is
   caller-controlled input, so no native recursion is used. */
typedef prc_api_write_node prc_write_tree_node;

/* Encodes the whole tree rooted at `root` into the current file structure's
   Table 47 section content (tag + parts[] + products[] + the trailing
   PRC_TYPE_ASM_FileStructure internal-data record). Nodes are flattened in
   post-order (children before parents) since PRC_TYPE_ASM_ProductOccurrence
   child references are plain 0-based indices into the shared products[]
   array and the root -- consumed by prc_api_prep_model_tree as simply the
   LAST entry of that array -- must therefore be written last. Product
   unique_id values (ContentPRCRefBase.unique_id) are assigned sequentially
   in that same post-order, starting at 1.

   *root_biased_index_out (if non-NULL) receives the root's BIASED (1-based)
   index into this file structure's products[] array -- which, since the
   root is always written last, equals the total product count. This is
   the value prc_write_model.c's ASM_ModelFile root reference needs for its
   ProductOccurrenceReference.root_index field: confirmed against a real
   PRC stream that root_index follows the same biased-index convention
   (0 = none) as every other cross-reference in this format, not a plain
   0-based array index as its parser-side field name might suggest.

   default_biased_style_index is a biased (1-based) index into the
   globals style table (prc_write_global.h), applied to every part,
   product occurrence, and representation item's GraphicsContent -- or 0
   for none. Pass 0 only if you have some other reason to believe the
   reader you're targeting tolerates a completely styleless file; every
   real-world PRC producer checked during this write facility's
   development always attaches at least a default material somewhere. */
/* Which style each representation item ends up with.

   The globals section, styles and all, is serialised before the tree is
   written, so an item's own material cannot be registered while the tree is
   being walked -- it has to be resolved first. Keying that by the item's
   address rather than by its position in a traversal means the two passes
   cannot drift apart: whatever order either one visits in, an item finds its
   own answer.

   Lookup is linear. Trees here are small, and an item without a material never
   reaches the map at all. */
typedef struct
{
    const prc_write_rep_item **items;
    uint32_t *styles;
    uint32_t count;
    uint32_t cap;
} prc_write_style_map;

/* Walks the tree, registering a colour/material/style for every item with
   has_material set, and records the biased style index for each. Items without
   one are absent from the map and fall back to the shared default. */
int prc_write_collect_item_styles(prc_context *ctx, prc_write_global_tables *tables,
    const prc_write_tree_node *root, prc_write_style_map *map);

/* Fills out_styles[num_entries] with the biased style index each tessellation
   entry's faces should name, or 0 for "leave it to the owner".

   A tessellation referenced by more than one representation item gets 0: one
   face record cannot be right for two items with different materials. So does
   one whose single item has no material of its own. Call after
   prc_write_collect_item_styles, which supplies the map. */
int prc_write_resolve_face_styles(prc_context *ctx, const prc_write_tree_node *root,
    const prc_write_style_map *map, uint32_t num_entries, uint32_t *out_styles);

void prc_write_style_map_release(prc_context *ctx, prc_write_style_map *map);

int prc_write_tree_to_stream(prc_context *ctx, prc_bit_write_state *s,
    const prc_write_tree_node *root, uint32_t *root_biased_index_out,
    uint32_t default_biased_style_index, const prc_write_style_map *style_map);

#endif
