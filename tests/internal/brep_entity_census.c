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

/* INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
   registered with CTest, no exit-code contract to keep stable.

   WHAT: Reports what a PRC file's exact geometry actually contains -- the
   six unzipped section sizes, each TopoContext's header fields, and a census
   of the PRC_TYPE_TOPO_* graph (bodies, connex, shells, faces, loops,
   coedges, edges, vertices) with per-type histograms of the surfaces carried
   by faces and the curves carried by edges and coedges.

   usage: brep_entity_census <file.prc|file.pdf> [more files...]

   HOW: Opens each file with prc_api_open_contents, then reads the parsed
   prc_data structures directly rather than going through the public API.
   That is deliberate: prc_api.h exposes exact geometry only as tessellated
   output (prc_api_get_exact_geometry_tessellation_vertices), which cannot
   distinguish a cylinder from a NURBS patch and so is the wrong view for
   this question. The walk descends bodies -> connex -> shells -> faces ->
   loops -> coedges -> edges -> vertices from each TopoContext in
   FileStructureExactGeometry (Table 50).

   WHY IT EXISTS: Every other diagnostic here is tessellation-centric --
   scan_prc counts parts and tessellations, face_planarity_census measures
   triangles, json_export dumps the model tree. None says anything about the
   exact-geometry side hanging off PRC_TYPE_ASM_FileStructureGeometry
   (Table 49), which leaves two recurring jobs without a tool:

     - Characterising a file for an outside bug report. "Exact geometry
       only, no tessellation" and "which surface and curve types" are the
       first two questions another reader's maintainer asks, and answering
       them used to mean a throwaway program each time.

     - Triage of the exact-geometry crash cluster. A fault that only fires
       on files carrying, say, SURF_Blend02 or CRV_Intersection is a
       different investigation from one that fires on plain NURBS, and the
       entity census separates those hypotheses cheaply.

   The section-size block answers a third question on its own: an empty
   tessellation section beside a large geometry section is the signature of
   an exact-only file, and the reverse is a tessellation-only one.

   LIMITATIONS: The walk follows the parsed pointer graph, so an entity that
   is referenced rather than stored inline (PtrTopology.is_stored == 0,
   Table 243) is counted once at its definition and not again at each
   reference -- these are counts of distinct entities, not of incidences.

   Compressed bodies (BrepDataCompress and SingleWireBodyCompress, Tables
   197/198) are counted but NOT descended into, because nanoPRC does not
   decode them; a file built entirely from compressed bodies therefore
   reports its body count and then zero for everything below it. That is the
   correct reading of the output, not a parse failure.

   Anything the parser skipped is invisible here by construction: this
   reports what nanoPRC understood, not what the bytes contain. */

#include <stdio.h>
#include <string.h>

#include "prc_api.h"
#include "prc_context.h"
#include "prc_data.h"

/* Histogram slots per type family. Both families are far smaller than this
   (Table 245 defines 16 curve types, Table 246 eighteen surfaces); the slack
   means an out-of-range value from a malformed or future file lands in a
   slot rather than off the end of the array. */
#define TYPE_SLOTS 64

typedef struct census_s
{
    long body_brep;
    long body_brep_compressed;
    long body_wire;
    long body_wire_compressed;
    long topo_other;

    long connex;
    long shell;
    long shell_closed;
    long face;
    long face_trimmed;
    long face_tolerance;
    long loop;
    long coedge;
    long edge;
    long vertex_unique;
    long vertex_multiple;

    long surf[TYPE_SLOTS];
    long crv[TYPE_SLOTS];
} census;

/* Table 246 -- surface types. */
static const char *
surf_name(uint32_t type)
{
    switch (type)
    {
    case PRC_TYPE_SURF_Blend01:     return "Blend01";
    case PRC_TYPE_SURF_Blend02:     return "Blend02";
    case PRC_TYPE_SURF_Blend03:     return "Blend03";
    case PRC_TYPE_SURF_NURBS:       return "NURBS";
    case PRC_TYPE_SURF_Cone:        return "Cone";
    case PRC_TYPE_SURF_Cylinder:    return "Cylinder";
    case PRC_TYPE_SURF_Cylindrical: return "Cylindrical";
    case PRC_TYPE_SURF_Offset:      return "Offset";
    case PRC_TYPE_SURF_Pipe:        return "Pipe";
    case PRC_TYPE_SURF_Plane:       return "Plane";
    case PRC_TYPE_SURF_Ruled:       return "Ruled";
    case PRC_TYPE_SURF_Sphere:      return "Sphere";
    case PRC_TYPE_SURF_Revolution:  return "Revolution";
    case PRC_TYPE_SURF_Extrusion:   return "Extrusion";
    case PRC_TYPE_SURF_FromCurves:  return "FromCurves";
    case PRC_TYPE_SURF_Torus:       return "Torus";
    case PRC_TYPE_SURF_Transform:   return "Transform";
    case PRC_TYPE_SURF_Blend04:     return "Blend04";
    default:                        return "(absent/unknown)";
    }
}

/* Table 245 -- curve types. */
static const char *
crv_name(uint32_t type)
{
    switch (type)
    {
    case PRC_TYPE_CRV_Blend02Boundary: return "Blend02Boundary";
    case PRC_TYPE_CRV_NURBS:           return "NURBS";
    case PRC_TYPE_CRV_Circle:          return "Circle";
    case PRC_TYPE_CRV_Composite:       return "Composite";
    case PRC_TYPE_CRV_OnSurf:          return "OnSurf";
    case PRC_TYPE_CRV_Ellipse:         return "Ellipse";
    case PRC_TYPE_CRV_Equation:        return "Equation";
    case PRC_TYPE_CRV_Helix01:         return "Helix01";
    case PRC_TYPE_CRV_Hyperbola:       return "Hyperbola";
    case PRC_TYPE_CRV_Intersection:    return "Intersection";
    case PRC_TYPE_CRV_Line:            return "Line";
    case PRC_TYPE_CRV_Offset:          return "Offset";
    case PRC_TYPE_CRV_Parabola:        return "Parabola";
    case PRC_TYPE_CRV_PolyLine:        return "PolyLine";
    case PRC_TYPE_CRV_Transform:       return "Transform";
    default:                           return "(absent/unknown)";
    }
}

/* Both histograms are indexed by a type's offset from its family base, so
   slot 0 -- the family base itself, which is abstract and never appears on a
   real entity -- doubles as the catch-all for an absent or out-of-range
   type. It prints as "(absent/unknown)". A face whose surface is absent and
   an edge carrying no curve both land there, which is what we want: they are
   worth seeing, not worth a column each. */
static void
tally_surf(census *c, uint32_t type)
{
    uint32_t slot = (type > PRC_TYPE_SURF && type < PRC_TYPE_SURF + TYPE_SLOTS)
                        ? (type - PRC_TYPE_SURF) : 0;
    c->surf[slot]++;
}

static void
tally_crv(census *c, uint32_t type)
{
    uint32_t slot = (type > PRC_TYPE_CRV && type < PRC_TYPE_CRV + TYPE_SLOTS)
                        ? (type - PRC_TYPE_CRV) : 0;
    c->crv[slot]++;
}

static void
walk_topo(census *c, prc_topo *topo);

static void
walk_ptr_topo(census *c, prc_ptr_topology *ptr)
{
    if (ptr == NULL || ptr->topo == NULL)
        return;
    walk_topo(c, ptr->topo);
}

/* Counts one entity and recurses into whatever it owns. Note that the
   BrepData arm also PRINTS, mid-walk: a body's stored bounding box is the
   cheapest read on a file's units and scale, which is the first thing anyone
   comparing two readers' renders wants, and printing it where it is found
   keeps it next to the context header it belongs to. */
static void
walk_topo(census *c, prc_topo *topo)
{
    prc_unsigned_int i;

    if (topo == NULL)
        return;

    switch (topo->tag)
    {
    case PRC_TYPE_TOPO_BrepData:
    {
        prc_topo_brep_data *body = topo->topo_brep_data;
        c->body_brep++;
        if (body == NULL)
            break;
        printf("      BrepData bounding box: (%g, %g, %g) .. (%g, %g, %g)\n",
               body->bounding_box.minimum_corner.x,
               body->bounding_box.minimum_corner.y,
               body->bounding_box.minimum_corner.z,
               body->bounding_box.maximum_corner.x,
               body->bounding_box.maximum_corner.y,
               body->bounding_box.maximum_corner.z);
        for (i = 0; i < body->number_of_connex; i++)
            walk_ptr_topo(c, &body->connex[i]);
        break;
    }

    /* Compressed bodies are counted only -- nanoPRC does not decode them, so
       there is no topology below this point to walk. See the header. */
    case PRC_TYPE_TOPO_BrepDataCompress:
        c->body_brep_compressed++;
        break;
    case PRC_TYPE_TOPO_SingleWireBodyCompress:
        c->body_wire_compressed++;
        break;

    case PRC_TYPE_TOPO_SingleWireBody:
        c->body_wire++;
        break;

    case PRC_TYPE_TOPO_Connex:
    {
        prc_topo_connex *connex = topo->topo_connex;
        c->connex++;
        if (connex == NULL)
            break;
        for (i = 0; i < connex->number_of_shells; i++)
            walk_ptr_topo(c, &connex->shells[i]);
        break;
    }

    case PRC_TYPE_TOPO_Shell:
    {
        prc_topo_shell *shell = topo->topo_shell;
        c->shell++;
        if (shell == NULL)
            break;
        if (shell->is_closed)
            c->shell_closed++;
        for (i = 0; i < shell->number_of_faces; i++)
            walk_ptr_topo(c, &shell->faces[i].face);
        break;
    }

    case PRC_TYPE_TOPO_Face:
    {
        prc_topo_face *face = topo->topo_face;
        c->face++;
        if (face == NULL)
            break;
        if (face->is_trimmed)
            c->face_trimmed++;
        if (face->has_tolerance)
            c->face_tolerance++;
        tally_surf(c, face->surface_geometry.surface.surface_type);
        for (i = 0; i < face->number_of_loops; i++)
            walk_ptr_topo(c, &face->loops[i]);
        break;
    }

    case PRC_TYPE_TOPO_Loop:
    {
        prc_topo_loop *loop = topo->topo_loop;
        c->loop++;
        if (loop == NULL)
            break;
        for (i = 0; i < loop->number_of_coedges; i++)
            walk_ptr_topo(c, &loop->coedge[i].next_coedge);
        break;
    }

    case PRC_TYPE_TOPO_CoEdge:
    {
        prc_topo_coedge *coedge = topo->topo_coedge;
        c->coedge++;
        if (coedge == NULL)
            break;
        /* A coedge's UV curve is optional, unlike an edge's 3D curve, so an
           absent one is not counted at all rather than counted as unknown. */
        if (coedge->ptr_curves.curve_type != 0)
            tally_crv(c, coedge->ptr_curves.curve_type);
        walk_ptr_topo(c, &coedge->ptr_topology);
        break;
    }

    case PRC_TYPE_TOPO_Edge:
    {
        prc_topo_edge *edge = topo->topo_edge;
        c->edge++;
        if (edge == NULL)
            break;
        tally_crv(c, edge->wire_edge.ptr_curve.curve_type);
        walk_ptr_topo(c, &edge->start_vertex);
        walk_ptr_topo(c, &edge->end_vertex);
        break;
    }

    case PRC_TYPE_TOPO_UniqueVertex:
        c->vertex_unique++;
        break;
    case PRC_TYPE_TOPO_MultipleVertex:
        c->vertex_multiple++;
        break;

    default:
        c->topo_other++;
        break;
    }
}

static void
report_file_structure(census *c, const prc_filestructure *fs, uint32_t index)
{
    prc_unsigned_int i;
    uint32_t j, k;

    printf("\n--- file structure %u: unzipped section sizes (bytes) ---\n", index);
    printf("  schema+globals : %lu\n", (unsigned long)fs->schema_globals_size);
    printf("  tree           : %lu\n", (unsigned long)fs->tree_size);
    printf("  tessellation   : %lu\n", (unsigned long)fs->tessellation_size);
    printf("  geometry       : %lu\n", (unsigned long)fs->geometry_size);
    printf("  extra geometry : %lu\n", (unsigned long)fs->extra_geometry_size);
    printf("  model          : %lu\n", (unsigned long)fs->model_size);

    if (fs->geometry == NULL)
    {
        printf("\n  no geometry section\n");
    }
    else
    {
        printf("\n  topo contexts  : %lu\n",
               (unsigned long)fs->geometry->exact_geometry.topo_context_count);
        for (i = 0; i < fs->geometry->exact_geometry.topo_context_count; i++)
        {
            prc_topo_context *tc = &fs->geometry->exact_geometry.topo_contexts[i];
            printf("    context[%lu]: behavior=0x%02x granularity=%g tolerance=%g "
                   "face_thickness=%s scale=%s bodies=%lu\n",
                   (unsigned long)i, tc->behavior, tc->grandularity, tc->tolerance,
                   tc->has_face_thickness ? "yes" : "no",
                   tc->has_scale ? "yes" : "no",
                   (unsigned long)tc->number_of_bodies);
            for (j = 0; j < tc->number_of_bodies; j++)
                walk_topo(c, &tc->bodies[j]);
        }
    }

    if (fs->extra_geometry == NULL)
        return;

    /* ExtraGeometry (Table 52) restates each body's type as a serial number
       -- PRC_TYPE_TOPO_BrepData is 154, BrepDataCompress 156 -- so it is a
       useful independent check on what the topology walk found. */
    printf("\n  extra geometry entries : %lu\n",
           (unsigned long)fs->extra_geometry->extra_geom_count);
    for (j = 0; j < fs->extra_geometry->extra_geom_count; j++)
    {
        prc_extra_geometry *extra = &fs->extra_geometry->extra_geom[j];
        printf("    entry[%lu]: bodies=%lu treat types=%lu\n", (unsigned long)j,
               (unsigned long)extra->summary.number_of_bodies,
               (unsigned long)extra->context_graphics.number_of_treat_type);
        for (k = 0; k < extra->summary.number_of_bodies; k++)
            printf("      body[%lu]: serial type=%lu tolerance=%g\n", (unsigned long)k,
                   (unsigned long)extra->summary.bodies[k].body_serial_type,
                   extra->summary.bodies[k].tolerance);
    }
}

static void
report_census(const census *c)
{
    int i;

    printf("\n--- topology census ---\n");
    printf("  bodies, BrepData             : %ld\n", c->body_brep);
    printf("  bodies, BrepDataCompress     : %ld\n", c->body_brep_compressed);
    printf("  bodies, SingleWireBody       : %ld\n", c->body_wire);
    printf("  bodies, SingleWireCompress   : %ld\n", c->body_wire_compressed);
    printf("  connex                       : %ld\n", c->connex);
    printf("  shells                       : %ld (closed %ld)\n",
           c->shell, c->shell_closed);
    printf("  faces                        : %ld (trimmed %ld, with tolerance %ld)\n",
           c->face, c->face_trimmed, c->face_tolerance);
    printf("  loops                        : %ld\n", c->loop);
    printf("  coedges                      : %ld\n", c->coedge);
    printf("  edges                        : %ld\n", c->edge);
    printf("  unique vertices              : %ld\n", c->vertex_unique);
    printf("  multiple vertices            : %ld\n", c->vertex_multiple);
    printf("  other topology tags          : %ld\n", c->topo_other);

    printf("\n--- surface types on faces ---\n");
    if (c->face == 0)
        printf("  (none)\n");
    for (i = 0; i < TYPE_SLOTS; i++)
        if (c->surf[i] != 0)
            printf("  %-16s : %ld\n",
                   surf_name((uint32_t)(PRC_TYPE_SURF + i)), c->surf[i]);

    printf("\n--- curve types on edges and coedges ---\n");
    if (c->edge == 0 && c->coedge == 0)
        printf("  (none)\n");
    for (i = 0; i < TYPE_SLOTS; i++)
        if (c->crv[i] != 0)
            printf("  %-16s : %ld\n",
                   crv_name((uint32_t)(PRC_TYPE_CRV + i)), c->crv[i]);
}

/* Returns 0 if the file was censused, 1 if it could not be opened. A file
   that opens but carries no exact geometry is a success with empty counts,
   not a failure -- that is a real and common answer to the question asked. */
static int
run_file(const char *path)
{
    prc_context *ctx;
    prc_data *data;
    census c;
    uint32_t i;

    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        printf("=== %s ===\n  context creation failed\n", path);
        return 1;
    }

    data = (prc_data *)prc_api_open_contents(ctx, path);
    if (data == NULL)
    {
        printf("=== %s ===\n  open failed\n", path);
        prc_api_print_error_stack(ctx);
        prc_api_release_context(ctx);
        return 1;
    }

    memset(&c, 0, sizeof(c));

    printf("=== %s ===\n", path);
    printf("  file structures : %u\n", data->file_structure_count);
    printf("  PDF views       : %u\n", data->view_count);
    printf("  unique parts    : %u\n", data->unique_part_count);
    printf("  unique markups  : %u\n", data->unique_markup_count);

    for (i = 0; i < data->file_structure_count; i++)
        report_file_structure(&c, &data->file_struct[i], i);

    report_census(&c);
    printf("\n");

    /* No tessellations were built, so every tessellation argument is empty.
       In particular this never populates data->exact_geom_tess, which is why
       the tool is unaffected by the known teardown fault on files that do. */
    prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL, 0, NULL);
    prc_api_release_context(ctx);
    return 0;
}

int main(int argc, char **argv)
{
    int failures = 0;
    int i;

    if (argc < 2)
    {
        printf("usage: %s <file.prc|file.pdf> [more files...]\n", argv[0]);
        return 2;
    }

    for (i = 1; i < argc; i++)
        failures += run_file(argv[i]);

    return failures == 0 ? 0 : 1;
}
