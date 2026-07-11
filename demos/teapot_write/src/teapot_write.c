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

/* ====================================================================
 * nanoPRC write-facility example: generate a complete .prc file from
 * scratch (no input PDF/PRC file is read).
 *
 * This is the companion to demos/quick_start (which shows how to READ a
 * .prc file); this demo shows how to WRITE one, using only the public API
 * declared in include/prc_api.h -- nothing here reaches into the library's
 * internal src/ headers, so everything below is something your own code
 * can do too.
 *
 * Usage:
 *     nano_prc_teapot_write [output.prc] [output.pdf]
 * (defaults to "utah_teapot.prc"/"utah_teapot.pdf" in the current directory
 * if no paths are given). Writes both a plain .prc file and the same
 * geometry embedded in a minimal 3D-annotated PDF (see Step 6 below); open
 * either in any PRC-capable viewer (nanoPRC's own nano_prc_viewer demo,
 * Adobe Reader/Acrobat, or a third-party PRC reader) to see the classic
 * Utah teapot -- the sample model created by Martin Newell at the
 * University of Utah in 1975, a de facto standard test object in computer
 * graphics ever since. See https://en.wikipedia.org/wiki/Utah_teapot for
 * its history; the control-point data used below (teapot_patches) is the
 * classic 32-patch Bezier formulation of that model.
 *
 * THE THREE-LEVEL SHAPE OF THE WRITE API
 * ---------------------------------------------------------------------
 * Every .prc file nanoPRC writes is built from three levels, assembled
 * bottom-up when you fill in the structs but read top-down once you see
 * the finished file:
 *
 *   1. prc_api_write_tessellation -- one raw piece of geometry: a
 *      position array, a triangle index array grouped into "faces" (each
 *      face gets its own normal), and optionally per-vertex normals if
 *      you already have them. You supply an ARRAY of these; each one is
 *      later referred to by a 1-based ("biased") index.
 *
 *   2. prc_api_write_rep_item -- a "representation item": a pointer from
 *      the product/part tree to ONE entry of that tessellation array,
 *      plus a `kind` (SURFACE for triangulated shells, WIRE for
 *      line/polyline geometry -- this demo only uses SURFACE).
 *
 *   3. prc_api_write_node -- one node of the product/part assembly tree.
 *      A node with representation items attached becomes a "part" with
 *      its own bounding box; every node (with or without a part) becomes
 *      a "product occurrence" that can nest children for sub-assemblies.
 *      The simplest possible scene -- and the one this demo builds -- is
 *      a single root node with one representation item and no children.
 *
 * Once you have those three pieces, prc_api_write_prc_file() does
 * everything else: it deflate-compresses each section, assembles the
 * ISO 14739 section framing and offset tables, and writes the complete
 * file in one call.
 *
 * WHAT'S NOT HERE YET
 * ---------------------------------------------------------------------
 * The write facility does not yet expose colors/materials/styles/pictures
 * through the public API (every representation item gets the reader's
 * default appearance), and it writes exactly one file structure (no
 * multi-file cross-references). See the "Write facility" section of
 * include/prc_api.h for the current, authoritative list of what is and
 * isn't implemented.
 * ==================================================================== */

#include <prc_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ---------------------------------------------------------------------
 * Sample geometry: the Utah teapot, as 32 bicubic Bezier patches (a NURBS
 * patch with unit weights and a Bezier-clamped knot vector is exactly a
 * plain Bezier patch, so no weight/knot machinery is needed to evaluate
 * this classic dataset). This part of the file has nothing to do with
 * nanoPRC's API -- it is just a way to generate some interesting
 * triangles to write out. Skip to teapot_build_mesh() and main() below if
 * you just want to see the API calls.
 * ------------------------------------------------------------------- */

#define TEAPOT_NUM_PATCHES 32

static const double teapot_patches[TEAPOT_NUM_PATCHES][16][3] =
{
    { /* Patch 0 */
        {1.4,0,2.4},{1.4,-0.784,2.4},{0.784,-1.4,2.4},{0,-1.4,2.4},
        {1.3375,0,2.53125},{1.3375,-0.749,2.53125},{0.749,-1.3375,2.53125},{0,-1.3375,2.53125},
        {1.4375,0,2.53125},{1.4375,-0.805,2.53125},{0.805,-1.4375,2.53125},{0,-1.4375,2.53125},
        {1.5,0,2.4},{1.5,-0.84,2.4},{0.84,-1.5,2.4},{0,-1.5,2.4}
    },
    { /* Patch 1 */
        {0,-1.4,2.4},{-0.784,-1.4,2.4},{-1.4,-0.784,2.4},{-1.4,0,2.4},
        {0,-1.3375,2.53125},{-0.749,-1.3375,2.53125},{-1.3375,-0.749,2.53125},{-1.3375,0,2.53125},
        {0,-1.4375,2.53125},{-0.805,-1.4375,2.53125},{-1.4375,-0.805,2.53125},{-1.4375,0,2.53125},
        {0,-1.5,2.4},{-0.84,-1.5,2.4},{-1.5,-0.84,2.4},{-1.5,0,2.4}
    },
    { /* Patch 2 */
        {-1.4,0,2.4},{-1.4,0.784,2.4},{-0.784,1.4,2.4},{0,1.4,2.4},
        {-1.3375,0,2.53125},{-1.3375,0.749,2.53125},{-0.749,1.3375,2.53125},{0,1.3375,2.53125},
        {-1.4375,0,2.53125},{-1.4375,0.805,2.53125},{-0.805,1.4375,2.53125},{0,1.4375,2.53125},
        {-1.5,0,2.4},{-1.5,0.84,2.4},{-0.84,1.5,2.4},{0,1.5,2.4}
    },
    { /* Patch 3 */
        {0,1.4,2.4},{0.784,1.4,2.4},{1.4,0.784,2.4},{1.4,0,2.4},
        {0,1.3375,2.53125},{0.749,1.3375,2.53125},{1.3375,0.749,2.53125},{1.3375,0,2.53125},
        {0,1.4375,2.53125},{0.805,1.4375,2.53125},{1.4375,0.805,2.53125},{1.4375,0,2.53125},
        {0,1.5,2.4},{0.84,1.5,2.4},{1.5,0.84,2.4},{1.5,0,2.4}
    },
    { /* Patch 4 */
        {1.5,0,2.4},{1.5,-0.84,2.4},{0.84,-1.5,2.4},{0,-1.5,2.4},
        {1.75,0,1.875},{1.75,-0.98,1.875},{0.98,-1.75,1.875},{0,-1.75,1.875},
        {2,0,1.35},{2,-1.12,1.35},{1.12,-2,1.35},{0,-2,1.35},
        {2,0,0.9},{2,-1.12,0.9},{1.12,-2,0.9},{0,-2,0.9}
    },
    { /* Patch 5 */
        {0,-1.5,2.4},{-0.84,-1.5,2.4},{-1.5,-0.84,2.4},{-1.5,0,2.4},
        {0,-1.75,1.875},{-0.98,-1.75,1.875},{-1.75,-0.98,1.875},{-1.75,0,1.875},
        {0,-2,1.35},{-1.12,-2,1.35},{-2,-1.12,1.35},{-2,0,1.35},
        {0,-2,0.9},{-1.12,-2,0.9},{-2,-1.12,0.9},{-2,0,0.9}
    },
    { /* Patch 6 */
        {-1.5,0,2.4},{-1.5,0.84,2.4},{-0.84,1.5,2.4},{0,1.5,2.4},
        {-1.75,0,1.875},{-1.75,0.98,1.875},{-0.98,1.75,1.875},{0,1.75,1.875},
        {-2,0,1.35},{-2,1.12,1.35},{-1.12,2,1.35},{0,2,1.35},
        {-2,0,0.9},{-2,1.12,0.9},{-1.12,2,0.9},{0,2,0.9}
    },
    { /* Patch 7 */
        {0,1.5,2.4},{0.84,1.5,2.4},{1.5,0.84,2.4},{1.5,0,2.4},
        {0,1.75,1.875},{0.98,1.75,1.875},{1.75,0.98,1.875},{1.75,0,1.875},
        {0,2,1.35},{1.12,2,1.35},{2,1.12,1.35},{2,0,1.35},
        {0,2,0.9},{1.12,2,0.9},{2,1.12,0.9},{2,0,0.9}
    },
    { /* Patch 8 */
        {2,0,0.9},{2,-1.12,0.9},{1.12,-2,0.9},{0,-2,0.9},
        {2,0,0.45},{2,-1.12,0.45},{1.12,-2,0.45},{0,-2,0.45},
        {1.5,0,0.225},{1.5,-0.84,0.225},{0.84,-1.5,0.225},{0,-1.5,0.225},
        {1.5,0,0.15},{1.5,-0.84,0.15},{0.84,-1.5,0.15},{0,-1.5,0.15}
    },
    { /* Patch 9 */
        {0,-2,0.9},{-1.12,-2,0.9},{-2,-1.12,0.9},{-2,0,0.9},
        {0,-2,0.45},{-1.12,-2,0.45},{-2,-1.12,0.45},{-2,0,0.45},
        {0,-1.5,0.225},{-0.84,-1.5,0.225},{-1.5,-0.84,0.225},{-1.5,0,0.225},
        {0,-1.5,0.15},{-0.84,-1.5,0.15},{-1.5,-0.84,0.15},{-1.5,0,0.15}
    },
    { /* Patch 10 */
        {-2,0,0.9},{-2,1.12,0.9},{-1.12,2,0.9},{0,2,0.9},
        {-2,0,0.45},{-2,1.12,0.45},{-1.12,2,0.45},{0,2,0.45},
        {-1.5,0,0.225},{-1.5,0.84,0.225},{-0.84,1.5,0.225},{0,1.5,0.225},
        {-1.5,0,0.15},{-1.5,0.84,0.15},{-0.84,1.5,0.15},{0,1.5,0.15}
    },
    { /* Patch 11 */
        {0,2,0.9},{1.12,2,0.9},{2,1.12,0.9},{2,0,0.9},
        {0,2,0.45},{1.12,2,0.45},{2,1.12,0.45},{2,0,0.45},
        {0,1.5,0.225},{0.84,1.5,0.225},{1.5,0.84,0.225},{1.5,0,0.225},
        {0,1.5,0.15},{0.84,1.5,0.15},{1.5,0.84,0.15},{1.5,0,0.15}
    },
    { /* Patch 12 */
        {-1.6,0,2.025},{-1.6,-0.3,2.025},{-1.5,-0.3,2.25},{-1.5,0,2.25},
        {-2.3,0,2.025},{-2.3,-0.3,2.025},{-2.5,-0.3,2.25},{-2.5,0,2.25},
        {-2.7,0,2.025},{-2.7,-0.3,2.025},{-3,-0.3,2.25},{-3,0,2.25},
        {-2.7,0,1.8},{-2.7,-0.3,1.8},{-3,-0.3,1.8},{-3,0,1.8}
    },
    { /* Patch 13 */
        {-1.5,0,2.25},{-1.5,0.3,2.25},{-1.6,0.3,2.025},{-1.6,0,2.025},
        {-2.5,0,2.25},{-2.5,0.3,2.25},{-2.3,0.3,2.025},{-2.3,0,2.025},
        {-3,0,2.25},{-3,0.3,2.25},{-2.7,0.3,2.025},{-2.7,0,2.025},
        {-3,0,1.8},{-3,0.3,1.8},{-2.7,0.3,1.8},{-2.7,0,1.8}
    },
    { /* Patch 14 */
        {-2.7,0,1.8},{-2.7,-0.3,1.8},{-3,-0.3,1.8},{-3,0,1.8},
        {-2.7,0,1.575},{-2.7,-0.3,1.575},{-3,-0.3,1.35},{-3,0,1.35},
        {-2.5,0,1.125},{-2.5,-0.3,1.125},{-2.65,-0.3,0.9375},{-2.65,0,0.9375},
        {-2,0,0.9},{-2,-0.3,0.9},{-1.9,-0.3,0.6},{-1.9,0,0.6}
    },
    { /* Patch 15 */
        {-3,0,1.8},{-3,0.3,1.8},{-2.7,0.3,1.8},{-2.7,0,1.8},
        {-3,0,1.35},{-3,0.3,1.35},{-2.7,0.3,1.575},{-2.7,0,1.575},
        {-2.65,0,0.9375},{-2.65,0.3,0.9375},{-2.5,0.3,1.125},{-2.5,0,1.125},
        {-1.9,0,0.6},{-1.9,0.3,0.6},{-2,0.3,0.9},{-2,0,0.9}
    },
    { /* Patch 16 */
        {1.7,0,1.425},{1.7,-0.66,1.425},{1.7,-0.66,0.6},{1.7,0,0.6},
        {2.6,0,1.425},{2.6,-0.66,1.425},{3.1,-0.66,0.825},{3.1,0,0.825},
        {2.3,0,2.1},{2.3,-0.25,2.1},{2.4,-0.25,2.025},{2.4,0,2.025},
        {2.7,0,2.4},{2.7,-0.25,2.4},{3.3,-0.25,2.4},{3.3,0,2.4}
    },
    { /* Patch 17 */
        {1.7,0,0.6},{1.7,0.66,0.6},{1.7,0.66,1.425},{1.7,0,1.425},
        {3.1,0,0.825},{3.1,0.66,0.825},{2.6,0.66,1.425},{2.6,0,1.425},
        {2.4,0,2.025},{2.4,0.25,2.025},{2.3,0.25,2.1},{2.3,0,2.1},
        {3.3,0,2.4},{3.3,0.25,2.4},{2.7,0.25,2.4},{2.7,0,2.4}
    },
    { /* Patch 18 */
        {2.7,0,2.4},{2.7,-0.25,2.4},{3.3,-0.25,2.4},{3.3,0,2.4},
        {2.8,0,2.475},{2.8,-0.25,2.475},{3.525,-0.25,2.49375},{3.525,0,2.49375},
        {2.9,0,2.475},{2.9,-0.15,2.475},{3.45,-0.15,2.5125},{3.45,0,2.5125},
        {2.8,0,2.4},{2.8,-0.15,2.4},{3.2,-0.15,2.4},{3.2,0,2.4}
    },
    { /* Patch 19 */
        {3.3,0,2.4},{3.3,0.25,2.4},{2.7,0.25,2.4},{2.7,0,2.4},
        {3.525,0,2.49375},{3.525,0.25,2.49375},{2.8,0.25,2.475},{2.8,0,2.475},
        {3.45,0,2.5125},{3.45,0.15,2.5125},{2.9,0.15,2.475},{2.9,0,2.475},
        {3.2,0,2.4},{3.2,0.15,2.4},{2.8,0.15,2.4},{2.8,0,2.4}
    },
    { /* Patch 20 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {0.8,0,3.15},{0.8,-0.45,3.15},{0.45,-0.8,3.15},{0,-0.8,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {0.2,0,2.7},{0.2,-0.112,2.7},{0.112,-0.2,2.7},{0,-0.2,2.7}
    },
    { /* Patch 21 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {0,-0.8,3.15},{-0.45,-0.8,3.15},{-0.8,-0.45,3.15},{-0.8,0,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {0,-0.2,2.7},{-0.112,-0.2,2.7},{-0.2,-0.112,2.7},{-0.2,0,2.7}
    },
    { /* Patch 22 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {-0.8,0,3.15},{-0.8,0.45,3.15},{-0.45,0.8,3.15},{0,0.8,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {-0.2,0,2.7},{-0.2,0.112,2.7},{-0.112,0.2,2.7},{0,0.2,2.7}
    },
    { /* Patch 23 */
        {0,0,3.15},{0,0,3.15},{0,0,3.15},{0,0,3.15},
        {0,0.8,3.15},{0.45,0.8,3.15},{0.8,0.45,3.15},{0.8,0,3.15},
        {0,0,2.85},{0,0,2.85},{0,0,2.85},{0,0,2.85},
        {0,0.2,2.7},{0.112,0.2,2.7},{0.2,0.112,2.7},{0.2,0,2.7}
    },
    { /* Patch 24 */
        {0.2,0,2.7},{0.2,-0.112,2.7},{0.112,-0.2,2.7},{0,-0.2,2.7},
        {0.4,0,2.55},{0.4,-0.224,2.55},{0.224,-0.4,2.55},{0,-0.4,2.55},
        {1.3,0,2.55},{1.3,-0.728,2.55},{0.728,-1.3,2.55},{0,-1.3,2.55},
        {1.3,0,2.4},{1.3,-0.728,2.4},{0.728,-1.3,2.4},{0,-1.3,2.4}
    },
    { /* Patch 25 */
        {0,-0.2,2.7},{-0.112,-0.2,2.7},{-0.2,-0.112,2.7},{-0.2,0,2.7},
        {0,-0.4,2.55},{-0.224,-0.4,2.55},{-0.4,-0.224,2.55},{-0.4,0,2.55},
        {0,-1.3,2.55},{-0.728,-1.3,2.55},{-1.3,-0.728,2.55},{-1.3,0,2.55},
        {0,-1.3,2.4},{-0.728,-1.3,2.4},{-1.3,-0.728,2.4},{-1.3,0,2.4}
    },
    { /* Patch 26 */
        {-0.2,0,2.7},{-0.2,0.112,2.7},{-0.112,0.2,2.7},{0,0.2,2.7},
        {-0.4,0,2.55},{-0.4,0.224,2.55},{-0.224,0.4,2.55},{0,0.4,2.55},
        {-1.3,0,2.55},{-1.3,0.728,2.55},{-0.728,1.3,2.55},{0,1.3,2.55},
        {-1.3,0,2.4},{-1.3,0.728,2.4},{-0.728,1.3,2.4},{0,1.3,2.4}
    },
    { /* Patch 27 */
        {0,0.2,2.7},{0.112,0.2,2.7},{0.2,0.112,2.7},{0.2,0,2.7},
        {0,0.4,2.55},{0.224,0.4,2.55},{0.4,0.224,2.55},{0.4,0,2.55},
        {0,1.3,2.55},{0.728,1.3,2.55},{1.3,0.728,2.55},{1.3,0,2.55},
        {0,1.3,2.4},{0.728,1.3,2.4},{1.3,0.728,2.4},{1.3,0,2.4}
    },
    { /* Patch 28 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {1.425,0,0},{1.425,0.798,0},{0.798,1.425,0},{0,1.425,0},
        {1.5,0,0.075},{1.5,0.84,0.075},{0.84,1.5,0.075},{0,1.5,0.075},
        {1.5,0,0.15},{1.5,0.84,0.15},{0.84,1.5,0.15},{0,1.5,0.15}
    },
    { /* Patch 29 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {0,1.425,0},{-0.798,1.425,0},{-1.425,0.798,0},{-1.425,0,0},
        {0,1.5,0.075},{-0.84,1.5,0.075},{-1.5,0.84,0.075},{-1.5,0,0.075},
        {0,1.5,0.15},{-0.84,1.5,0.15},{-1.5,0.84,0.15},{-1.5,0,0.15}
    },
    { /* Patch 30 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {-1.425,0,0},{-1.425,-0.798,0},{-0.798,-1.425,0},{0,-1.425,0},
        {-1.5,0,0.075},{-1.5,-0.84,0.075},{-0.84,-1.5,0.075},{0,-1.5,0.075},
        {-1.5,0,0.15},{-1.5,-0.84,0.15},{-0.84,-1.5,0.15},{0,-1.5,0.15}
    },
    { /* Patch 31 */
        {0,0,0},{0,0,0},{0,0,0},{0,0,0},
        {0,-1.425,0},{0.798,-1.425,0},{1.425,-0.798,0},{1.425,0,0},
        {0,-1.5,0.075},{0.84,-1.5,0.075},{1.5,-0.84,0.075},{1.5,0,0.075},
        {0,-1.5,0.15},{0.84,-1.5,0.15},{1.5,-0.84,0.15},{1.5,0,0.15}
    }
};

/* Cubic Bernstein basis polynomials at parameter t. */
static void
teapot_bernstein(double t, double b[4])
{
    double mt = 1.0 - t;

    b[0] = mt * mt * mt;
    b[1] = 3.0 * t * mt * mt;
    b[2] = 3.0 * t * t * mt;
    b[3] = t * t * t;
}

/* Evaluates one 4x4 bicubic Bezier patch at parameter (u, v). */
static void
teapot_eval(const double cp[16][3], double u, double v, double out[3])
{
    double bu[4], bv[4];
    int i, j;

    teapot_bernstein(u, bu);
    teapot_bernstein(v, bv);
    out[0] = out[1] = out[2] = 0.0;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            double weight = bu[i] * bv[j];
            const double *p = cp[i * 4 + j];

            out[0] += weight * p[0];
            out[1] += weight * p[1];
            out[2] += weight * p[2];
        }
    }
}

/* Derivative of the cubic Bernstein basis at parameter t (see
   teapot_bernstein). */
static void
teapot_bernstein_deriv(double t, double db[4])
{
    double mt = 1.0 - t;

    db[0] = -3.0 * mt * mt;
    db[1] = 3.0 * mt * mt - 6.0 * t * mt;
    db[2] = 6.0 * t * mt - 3.0 * t * t;
    db[3] = 3.0 * t * t;
}

/* Evaluates the patch's partial derivatives dS/du and dS/dv at (u, v) --
   the surface tangent vectors, whose cross product gives the true
   analytic surface normal (see teapot_eval_normal). Differentiating the
   basis functions instead of finite-differencing the surface gives an
   exact tangent with no step-size tuning. */
static void
teapot_eval_partials(const double cp[16][3], double u, double v, double dS_du[3], double dS_dv[3])
{
    double bu[4], bv[4], dbu[4], dbv[4];
    int i, j;

    teapot_bernstein(u, bu);
    teapot_bernstein(v, bv);
    teapot_bernstein_deriv(u, dbu);
    teapot_bernstein_deriv(v, dbv);
    dS_du[0] = dS_du[1] = dS_du[2] = 0.0;
    dS_dv[0] = dS_dv[1] = dS_dv[2] = 0.0;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            const double *p = cp[i * 4 + j];
            double wu = dbu[i] * bv[j];
            double wv = bu[i] * dbv[j];

            dS_du[0] += wu * p[0]; dS_du[1] += wu * p[1]; dS_du[2] += wu * p[2];
            dS_dv[0] += wv * p[0]; dS_dv[1] += wv * p[1]; dS_dv[2] += wv * p[2];
        }
    }
}

/* Analytic per-vertex surface normal: normalize(dS/du x dS/dv), matching
   the winding of teapot_build_mesh's triangulation (increasing i = +u,
   increasing j = +v), so this needs no separate orientation flip.

   8 of the 32 patches collapse one whole control-point row to a single
   point (see teapot_build_mesh's comment); along that row dS/dv is
   identically zero (moving in v doesn't move the position at all when
   the whole row is one point), making the cross product zero too. When
   that happens, nudge (u, v) a small step toward the patch interior and
   re-evaluate -- the position we already sampled stays exactly on the
   boundary, only the normal is estimated from just inside it, which is
   visually indistinguishable at this tessellation density. */
static void
teapot_eval_normal(const double cp[16][3], double u, double v, double n[3])
{
    double du[3], dv[3];
    double cx = 0.0, cy = 0.0, cz = 0.0, len = 0.0;
    double uu = u, vv = v;
    int attempt;

    for (attempt = 0; attempt < 6; attempt++)
    {
        teapot_eval_partials(cp, uu, vv, du, dv);
        cx = du[1] * dv[2] - du[2] * dv[1];
        cy = du[2] * dv[0] - du[0] * dv[2];
        cz = du[0] * dv[1] - du[1] * dv[0];
        len = sqrt(cx * cx + cy * cy + cz * cz);
        if (len > 1e-9)
            break;
        uu = u + (u < 0.5 ? 1.0 : -1.0) * 0.01 * (attempt + 1);
        vv = v + (v < 0.5 ? 1.0 : -1.0) * 0.01 * (attempt + 1);
    }

    if (len > 1e-9)
    {
        n[0] = cx / len; n[1] = cy / len; n[2] = cz / len;
    }
    else
    {
        /* Never observed in practice for this dataset, but fall back to
           something finite and unit-length rather than propagate a
           zero/NaN normal. */
        n[0] = 0.0; n[1] = 0.0; n[2] = 1.0;
    }
}

/* Each patch is sampled on an independent (TEAPOT_SAMPLES x TEAPOT_SAMPLES)
   grid covering the FULL u,v in [0,1] range, including the true patch
   boundary. This teapot control-point data is specifically constructed so
   adjacent patches share their boundary row/column of control points, so
   sampling right to the edge makes neighboring patches meet exactly
   (Bezier patches are fully determined by their control points, so two
   patches sharing a boundary control-point row evaluate identical points
   all along it) -- no explicit cross-patch vertex welding needed for the
   surface to look seamless, just matching positions.

   8 of the 32 patches (the lid tip and the base) additionally collapse
   one whole control-point row to a single point, so sampling AT that row
   (u or v = 0 or 1, whichever collapses) makes every vertex in it
   coincide and the triangles touching it degenerate (zero area). That's
   harmless here: prc_write_tess_3d (the uncompressed encoder this demo
   uses) does no deduplication or quantization, so a degenerate triangle
   just renders as an invisible sliver, not an error. (The original
   geometry generator this was adapted from, tests/unit/test_compress_
   tess.c, sampled inset from the boundary instead -- it exercises the
   *compressed* encoder's quantized vertex welding, where a degenerate
   triangle becomes a welded-away point that has to be excluded from
   both sides of a centroid cross-check by hand. That concern doesn't
   apply here, so this demo samples the full range instead and gets a
   seamless surface.) */
#define TEAPOT_GRID_N 8
#define TEAPOT_SAMPLES (TEAPOT_GRID_N + 1)
#define TEAPOT_VERTS_PER_PATCH (TEAPOT_SAMPLES * TEAPOT_SAMPLES)
#define TEAPOT_TRIS_PER_PATCH (2 * TEAPOT_GRID_N * TEAPOT_GRID_N)
#define TEAPOT_QUADS_PER_PATCH (TEAPOT_GRID_N * TEAPOT_GRID_N)
#define TEAPOT_TOTAL_VERTS (TEAPOT_NUM_PATCHES * TEAPOT_VERTS_PER_PATCH)
#define TEAPOT_TOTAL_TRIS (TEAPOT_NUM_PATCHES * TEAPOT_TRIS_PER_PATCH)
#define TEAPOT_TOTAL_FACES (TEAPOT_NUM_PATCHES * TEAPOT_QUADS_PER_PATCH)

/* Samples every patch and triangulates it into `positions`/`tris`, filling
   `normals` with the analytic per-vertex surface normal at each sampled
   point (see teapot_eval_normal) so the same vertex index can be reused
   for both position and normal in main()'s prc_api_write_tessellation --
   every triangle touching a given grid point shares its exact normal, so
   shading varies smoothly across the whole patch instead of faceting at
   triangle/quad boundaries. `face_tri_counts` still gets 2 for every grid
   quad: prc_api_write_tessellation needs a face grouping regardless of
   whether normals are supplied or computed, it just no longer determines
   shading now that real per-vertex normals are provided. */
static void
teapot_build_mesh(double *positions, double *normals, uint32_t *tris, uint32_t *face_tri_counts,
    double bbox_min[3], double bbox_max[3])
{
    uint32_t p, i, j;

    bbox_min[0] = bbox_min[1] = bbox_min[2] = 1e300;
    bbox_max[0] = bbox_max[1] = bbox_max[2] = -1e300;

    for (p = 0; p < TEAPOT_NUM_PATCHES; p++)
    {
        uint32_t vbase = p * TEAPOT_VERTS_PER_PATCH;
        uint32_t tbase = p * TEAPOT_TRIS_PER_PATCH;
        uint32_t fbase = p * TEAPOT_QUADS_PER_PATCH;
        uint32_t t = 0, f = 0;

        for (i = 0; i < TEAPOT_SAMPLES; i++)
        {
            double u = (double)i / (double)TEAPOT_GRID_N;

            for (j = 0; j < TEAPOT_SAMPLES; j++)
            {
                double v = (double)j / (double)TEAPOT_GRID_N;
                double pt[3], nrm[3];
                uint32_t vidx = vbase + i * TEAPOT_SAMPLES + j;
                int c;

                teapot_eval(teapot_patches[p], u, v, pt);
                teapot_eval_normal(teapot_patches[p], u, v, nrm);
                positions[vidx * 3 + 0] = pt[0];
                positions[vidx * 3 + 1] = pt[1];
                positions[vidx * 3 + 2] = pt[2];
                normals[vidx * 3 + 0] = nrm[0];
                normals[vidx * 3 + 1] = nrm[1];
                normals[vidx * 3 + 2] = nrm[2];
                for (c = 0; c < 3; c++)
                {
                    if (pt[c] < bbox_min[c]) bbox_min[c] = pt[c];
                    if (pt[c] > bbox_max[c]) bbox_max[c] = pt[c];
                }
            }
        }

        for (i = 0; i < TEAPOT_GRID_N; i++)
        {
            for (j = 0; j < TEAPOT_GRID_N; j++)
            {
                uint32_t v00 = vbase + i * TEAPOT_SAMPLES + j;
                uint32_t v01 = vbase + i * TEAPOT_SAMPLES + (j + 1);
                uint32_t v10 = vbase + (i + 1) * TEAPOT_SAMPLES + j;
                uint32_t v11 = vbase + (i + 1) * TEAPOT_SAMPLES + (j + 1);
                uint32_t tidx = tbase + t;

                tris[tidx * 3 + 0] = v00;
                tris[tidx * 3 + 1] = v10;
                tris[tidx * 3 + 2] = v11;
                t++;
                tidx = tbase + t;
                tris[tidx * 3 + 0] = v00;
                tris[tidx * 3 + 1] = v11;
                tris[tidx * 3 + 2] = v01;
                t++;

                face_tri_counts[fbase + f] = 2;
                f++;
            }
        }
    }
}

/* ---------------------------------------------------------------------
 * The actual nanoPRC write-API example starts here.
 * ------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
    const char *output_path = (argc > 1) ? argv[1] : "utah_teapot.prc";
    const char *pdf_path = (argc > 2) ? argv[2] : "utah_teapot.pdf";
    prc_context *ctx;
    double *positions;
    double *normals;
    uint32_t *tris;
    uint32_t *face_tri_counts;
    double bbox_min[3], bbox_max[3];
    int code;

    /* Step 1: build the raw geometry we want to write. nanoPRC never
       generates geometry for you -- positions/indices/face groups are
       always the caller's data, in the caller's own units. */
    positions = (double *)malloc(sizeof(double) * TEAPOT_TOTAL_VERTS * 3);
    normals = (double *)malloc(sizeof(double) * TEAPOT_TOTAL_VERTS * 3);
    tris = (uint32_t *)malloc(sizeof(uint32_t) * TEAPOT_TOTAL_TRIS * 3);
    face_tri_counts = (uint32_t *)malloc(sizeof(uint32_t) * TEAPOT_TOTAL_FACES);
    if (positions == NULL || normals == NULL || tris == NULL || face_tri_counts == NULL)
    {
        fprintf(stderr, "allocation failed\n");
        return 1;
    }
    teapot_build_mesh(positions, normals, tris, face_tri_counts, bbox_min, bbox_max);

    /* Step 2: describe that geometry as one prc_api_write_tessellation
       entry. `normals` holds one analytic surface normal per sampled
       vertex (see teapot_eval_normal), aligned 1:1 with `positions` --
       every grid point has exactly one position AND one normal, so
       norm_indices can reuse `tris` verbatim (the same vertex index
       selects both). That per-vertex normal is what makes shading vary
       smoothly across a whole patch instead of faceting at triangle/quad
       boundaries; passing normals == NULL instead asks the encoder to
       compute normals itself instead of using a supplied one, which is a
       reasonable default when you don't have real per-vertex normals to
       hand, but looks faceted.

       kind = COMPRESSED (PRC_TYPE_TESS_3D_Compressed). Earlier sessions'
       attempts at COMPRESSED all came back null/empty geometry (or, once
       has_faces was fixed, an outright "invalid vector subscript"
       exception) from an independent reader used for ground-truth
       verification. Root-caused via real, independently-produced reference
       files (not just spec reading): (1) this encoder's C1 path wrote a
       genuinely empty line_attribute_array, while every real producer
       writes at least one entry; (2) once fixed to write exactly one
       entry, that only matched reference files with a single face --
       multi-face content needs one entry PER FACE, which a real,
       multi-face reference file confirmed; (3) has_faces was earlier
       (wrongly) believed to require TRUE whenever triangle_face_array has
       real data -- a real multi-face reference file has has_faces FALSE
       with genuinely multi-valued triangle_face_array, so it was reverted
       to FALSE (this write facility never emits exact B-Rep geometry, and
       has_faces means "built from real topological faces", not "has
       triangle-to-style groupings"). With all three fixed, this teapot
       (2592 verts, 4096 tris, 2048 faces) now reads back via that
       independent reader as real, non-null geometry: 4032 of 4096
       triangles, which is CORRECT, not a shortfall -- teapot_patches[]'s
       patches 20-23 (the lid's top pole) and 28-31 (the base's center
       pole) each have all four control points of one grid row set to the
       exact same point ({0,0,3.15} and {0,0,0} respectively, the classic
       Utah teapot dataset's pole/apex construction), so every sample along
       that row of each of those 8 patches coincides regardless of the
       other parameter. Each of the resulting 8 quads per pole patch (64
       quads total across all 8 pole patches) therefore has one zero-area
       (degenerate) triangle and one real one; this write facility's own
       preprocessing correctly welds/drops the degenerate ones, so
       4096 - 64 == 4032 real triangles is exactly right. Confirmed by
       inspecting the independent reader's own per-face triangle-index
       counts: precisely those 64 faces (8 groups of 8, matching the 8
       pole patches' first grid row) have 3 indices (1 triangle) instead
       of 6 (2 triangles); every other face has the full 6. */
    prc_api_write_tessellation tess;
    memset(&tess, 0, sizeof(tess));
    tess.kind = PRC_API_WRITE_TESS_KIND_COMPRESSED;
    tess.positions = positions;
    tess.num_positions = TEAPOT_TOTAL_VERTS;
    tess.normals = normals;
    tess.num_normals = TEAPOT_TOTAL_VERTS;
    tess.tri_indices = tris;
    tess.norm_indices = tris;
    tess.num_triangles = TEAPOT_TOTAL_TRIS;
    tess.face_tri_counts = face_tri_counts;
    tess.num_faces = TEAPOT_TOTAL_FACES;

    /* Step 3: describe one representation item referencing that
       tessellation. biased_tessellation_index is 1-based: since our
       tess_entries array (passed to prc_api_write_prc_file below) has
       exactly one entry, that entry's biased index is 1. */
    prc_api_write_rep_item rep_item;
    memset(&rep_item, 0, sizeof(rep_item));
    rep_item.kind = PRC_API_WRITE_RI_SURFACE;
    rep_item.biased_tessellation_index = 1;
    rep_item.is_closed = 0; /* the per-patch sampling isn't seam-welded -- see teapot_build_mesh's comment */

    /* Step 4: describe the product/part tree. Real-world PRC files (e.g.
       PDF 3D-annotated files produced by CAD tools) consistently model
       even a single-part scene as TWO levels: a pure "assembly" root node
       (no representation items of its own) with ONE child node that
       actually owns the part -- confirmed by comparing this demo's
       earlier flat, single-node output (root holding the part directly)
       against a real extracted PRC stream, field by field. Some readers
       apparently expect that assembly/part split and silently produce no
       output for a part attached directly to the root, even though
       nothing in the format's own parsing rules requires it. So: a leaf
       node for the actual geometry, nested one level under a bare
       assembly root. */
    prc_api_write_node part_node;
    memset(&part_node, 0, sizeof(part_node));
    part_node.rep_items = &rep_item;
    part_node.num_rep_items = 1;
    /* A padded (not exactly-tight, not unbounded) bbox: real-world PRC
       producers' PartDefinition boxes are rarely the mesh's exact tight
       bounds, but a huge +/-1e20 placeholder is needlessly close to
       double-precision's own limits for no known benefit -- pad the real
       computed box by 50% per axis instead. */
    {
        int a;
        for (a = 0; a < 3; a++)
        {
            double mid = 0.5 * (bbox_min[a] + bbox_max[a]);
            double half = 0.5 * (bbox_max[a] - bbox_min[a]);
            if (half < 1e-6) half = 1e-6;
            part_node.bbox_min[a] = mid - half * 1.5;
            part_node.bbox_max[a] = mid + half * 1.5;
        }
    }
    /* `name` labels this node's PRODUCT OCCURRENCE (the placement/instance
       shown in a reader's model tree); `part_name` labels its PART
       DEFINITION (the underlying geometry/body entry) -- distinct fields
       because the format itself keeps them as distinct entities, and a
       reader's tree UI typically shows both. Leaving either NULL falls
       back to a generic label like "node". */
    part_node.name = "teapot_body";
    part_node.part_name = "patch_faces";
    /* part_node.has_transform stays 0: it occupies its parent's coordinate
       system as-is. Set has_transform = 1 and fill in `transform`
       (column-major 4x4) to place a child elsewhere within its parent. */

    prc_api_write_node *part_node_ptr = &part_node;
    prc_api_write_node root;
    memset(&root, 0, sizeof(root));
    root.children = &part_node_ptr;
    root.num_children = 1;
    root.name = "teapot";
    /* root.rep_items/num_rep_items stay 0/NULL: this node is a pure
       assembly container, it owns no part of its own -- so root.part_name
       is left NULL too, there is no part here to name. */

    /* Step 5: everything above was just describing data in memory -- no
       nanoPRC calls yet. Now open a context (the library's per-session
       handle: custom allocators, the error stack, and, in debug builds,
       leak tracking all live on it) and write the file. */
    ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        fprintf(stderr, "prc_api_new_context failed\n");
        free(positions); free(normals); free(tris); free(face_tri_counts);
        return 1;
    }

    printf("Encoding %u vertices, %u triangles, %u faces...\n",
        (unsigned)TEAPOT_TOTAL_VERTS, (unsigned)TEAPOT_TOTAL_TRIS, (unsigned)TEAPOT_TOTAL_FACES);

    /* model_name labels the top-level "model" entry itself -- the root of
       the whole model tree, one level above root.name ("teapot").

       prc_api_write_prc_buffer does the same encoding prc_api_write_prc_file
       does, but returns the complete PRC byte stream in memory instead of
       writing it straight to disk -- so the exact same encoded bytes can be
       (a) written out as a plain .prc file ourselves, and (b) embedded in a
       PDF via prc_api_pdf_embed_prc below, without encoding the geometry
       twice. */
    {
        uint8_t *prc_buf = NULL;
        size_t prc_buf_size = 0;
        FILE *out;

        code = prc_api_write_prc_buffer(ctx, "nanoPRC", &root, &tess, 1, &prc_buf, &prc_buf_size);

        /* The geometry arrays were only needed for the duration of the call
           above (the encoder copies whatever it needs into the encoded
           byte stream) -- safe to free them now regardless of success or
           failure. */
        free(positions);
        free(normals);
        free(tris);
        free(face_tri_counts);

        if (code != 0)
        {
            fprintf(stderr, "prc_api_write_prc_buffer failed (code %d)\n", code);
            prc_api_print_error_stack(ctx);
            prc_api_release_context(ctx);
            return 1;
        }

        out = fopen(output_path, "wb");
        if (out == NULL || fwrite(prc_buf, 1, prc_buf_size, out) != prc_buf_size)
        {
            fprintf(stderr, "failed to write %s\n", output_path);
            if (out != NULL) fclose(out);
            prc_api_write_prc_buffer_free(ctx, prc_buf);
            prc_api_release_context(ctx);
            return 1;
        }
        fclose(out);
        printf("Wrote %s (%u bytes)\n", output_path, (unsigned)prc_buf_size);

        /* Step 6: embed the same encoded bytes in a minimal, single-page
           3D-annotated PDF -- the standard (ISO 32000) mechanism Adobe
           Reader/Acrobat and examples/ *.pdf in this repository use, not a
           nanoPRC-specific format. One named view is enough to demonstrate
           prc_pdf_view_spec; its eye/target/up are derived from the mesh's
           own bounding box (computed by teapot_build_mesh above) so the
           teapot is actually framed in shot rather than pointed at an
           arbitrary fixed camera that might miss it entirely for
           differently-scaled geometry. */
        {
            prc_pdf_view_spec view;
            prc_pdf_write_options pdf_opts;
            double center[3], extent[3], diag_len;
            int a;

            for (a = 0; a < 3; a++) center[a] = 0.5 * (bbox_min[a] + bbox_max[a]);
            for (a = 0; a < 3; a++) extent[a] = bbox_max[a] - bbox_min[a];
            diag_len = sqrt(extent[0] * extent[0] + extent[1] * extent[1] + extent[2] * extent[2]);
            if (diag_len < 1e-6) diag_len = 1.0;

            memset(&view, 0, sizeof(view));
            view.name = "Default";
            /* A 3/4-ish view direction. Distance multiplier history (all
               2026-07-10, same session): started at ~1.78x the bounding-
               box diagonal (magnitude of (1.0,-1.3,0.7)), found "too
               tight, teapot off-screen" -- but that was BEFORE prc_pdf_
               compute_c2w_co's camera-matrix bug (transposed rows,
               inverted forward vector) was fixed, so "off-screen" was very
               likely the camera pointing at the wrong place entirely, not
               a real distance problem; widened to ~5.3x, which just made a
               mis-pointed camera's frame wide enough to stumble onto the
               object anyway. Once the matrix bug was fixed (camera
               provably centered on the model, verified by solving eye +
               CO*forward against the mesh's own real bbox center), that
               ~5.3x distance made the teapot fill only ~10% of the view
               (confirmed in Acrobat) -- tried dividing by 7 (~0.76x) to
               reach the requested ~70%, which overshot into clipping.
               Landed back on the ORIGINAL ~1.78x value: it sits neatly
               between the two data points (0.76x too close, 5.3x too far)
               and was never actually wrong on distance -- only on
               direction, since the matrix bug was still active on that
               very first attempt too. up = +Z matches this dataset's own
               up axis (the spout tip sits at the highest Z value in
               teapot_patches). */
            view.eye[0] = center[0] + diag_len * 1.0;
            view.eye[1] = center[1] - diag_len * 1.3;
            view.eye[2] = center[2] + diag_len * 0.7;
            view.target[0] = center[0];
            view.target[1] = center[1];
            view.target[2] = center[2];
            view.up[0] = 0.0; view.up[1] = 0.0; view.up[2] = 1.0;
            view.is_default = 1;

            memset(&pdf_opts, 0, sizeof(pdf_opts));
            pdf_opts.views = &view;
            pdf_opts.num_views = 1;

            code = prc_api_pdf_embed_prc(ctx, pdf_path, prc_buf, prc_buf_size, &pdf_opts);
            if (code != 0)
            {
                fprintf(stderr, "prc_api_pdf_embed_prc failed (code %d)\n", code);
                prc_api_print_error_stack(ctx);
                prc_api_write_prc_buffer_free(ctx, prc_buf);
                prc_api_release_context(ctx);
                return 1;
            }
            printf("Wrote %s\n", pdf_path);
        }

        prc_api_write_prc_buffer_free(ctx, prc_buf);
    }

    /* Step 7 (optional): read both files straight back -- the .prc file
       directly, and the .pdf file through the same prc_api_open_contents
       entry point (it transparently detects and extracts the embedded PRC
       stream from a PDF 3D annotation) -- as a sanity check that both are
       actually well-formed and agree with each other. This is not required
       to use the write API; it's here purely so this demo verifies its own
       output. */
    {
        const char *paths[2];
        const char *labels[2];
        int p;
        uint32_t verified_tess[2] = { 0, 0 };
        uint32_t verified_faces[2] = { 0, 0 };

        paths[0] = output_path;  labels[0] = "prc";
        paths[1] = pdf_path;     labels[1] = "pdf";

        for (p = 0; p < 2; p++)
        {
            prc_api_data data;
            prc_api_product *model_tree = NULL;
            uint32_t num_parts, num_products, num_markups;
            uint32_t total_tess = 0, total_line_tess = 0;

            data = prc_api_open_contents(ctx, paths[p]);
            if (data == NULL)
            {
                fprintf(stderr, "verification: prc_api_open_contents failed for %s\n", paths[p]);
                prc_api_print_error_stack(ctx);
                prc_api_release_context(ctx);
                return 1;
            }

            code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
            if (code == 0)
                code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts, num_products, num_markups);
            if (code == 0)
                code = prc_api_get_number_tessellations(ctx, data, model_tree, &total_tess, &total_line_tess);

            if (code != 0)
            {
                fprintf(stderr, "verification: failed reading back the model tree from %s\n", paths[p]);
                prc_api_print_error_stack(ctx);
                prc_api_release_data(ctx, data, NULL, 0, NULL, 0, NULL);
                prc_api_release_context(ctx);
                return 1;
            }

            verified_tess[p] = total_tess;
            verified_faces[p] = total_tess > 0 ? prc_api_get_number_faces(ctx, data, 0) : 0;

            /* num_parts/num_products are preallocation counts for
               prc_api_create_model_tree's internal tree (they count API
               tree nodes, not literally "how many parts/products you
               wrote" -- a single representation item contributes to both),
               so they're not printed here; tessellations/faces map
               directly to what we asked the encoder to write and are what
               most callers care about. */
            printf("Verification (%s): tessellations=%u faces_in_tess0=%u\n",
                labels[p], verified_tess[p], verified_faces[p]);

            prc_api_release_data(ctx, data, NULL, 0, NULL, 0, model_tree);
        }

        if (verified_tess[0] != verified_tess[1] || verified_faces[0] != verified_faces[1])
        {
            fprintf(stderr, "verification: .prc and .pdf disagree on tessellation/face counts\n");
            prc_api_release_context(ctx);
            return 1;
        }
    }

    prc_api_release_context(ctx);
    return 0;
}
