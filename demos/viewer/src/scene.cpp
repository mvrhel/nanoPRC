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

#include "scene.h"

#include <cstdio>
#include <cstdlib>
#include <string>

#include <prc_api.h>
#include "text.h"
#include "camera.h"
#include "product.h"
#include "skybox.h"
#include "graphics2d.h"
#include "mesh.h"
#include "shadow.h"
#include "composite.h"
#include "bloom.h"
#include <cfloat>
#include <cmath>

#define SHADOW_MAP_TEXTURE_UNIT 8
#define COMPOSITE_TEXTURE_UNIT 9

static const Vertex kPlaneVertices[] = {
    Vertex{
        Vector3{-0.5f, 0.0f, -0.5f},
        Vector3{0.0f, 1.0f, 0.0f},
        Vector3{1.0f, 1.0f, 1.0f},
        Vector2{0.0f, 0.0f}},
    Vertex{
        Vector3{0.5f, 0.0f, -0.5f},
        Vector3{0.0f, 1.0f, 0.0f},
        Vector3{1.0f, 1.0f, 1.0f},
        Vector2{1.0f, 0.0f}},
    Vertex{
        Vector3{0.5f, 0.0f, 0.5f},
        Vector3{0.0f, 1.0f, 0.0f},
        Vector3{1.0f, 1.0f, 1.0f},
        Vector2{1.0f, 1.0f}},
    Vertex{
        Vector3{-0.5f, 0.0f, 0.5f},
        Vector3{0.0f, 1.0f, 0.0f},
        Vector3{1.0f, 1.0f, 1.0f},
        Vector2{1.0f, 0.0f}}};

static const GLuint kPlaneIndices[] = {
    0, 2, 1,
    0, 3, 2};

static void setCompanionVisibilityRecursive(Product *product, bool enabled)
{
    if (product == NULL)
        return;

    Product *companion = product->renderCompanion();
    if (companion)
        companion->setEnabled(enabled);

    for (uint32_t i = 0; i < product->numChildren(); i++)
    {
        setCompanionVisibilityRecursive(product->children()[i], enabled);
    }
}

void Scene::setCameraInitialPosition(Camera *camera)
{
    const float kFallbackDiagonal = 100.0f;
    const float kFallbackNear = 0.1f;
    const float kFallbackFar = 1000.0f;

    bool bboxFinite =
        std::isfinite(_bbox_min[0]) && std::isfinite(_bbox_min[1]) && std::isfinite(_bbox_min[2]) &&
        std::isfinite(_bbox_max[0]) && std::isfinite(_bbox_max[1]) && std::isfinite(_bbox_max[2]);
    bool bboxOrdered =
        (_bbox_max[0] >= _bbox_min[0]) &&
        (_bbox_max[1] >= _bbox_min[1]) &&
        (_bbox_max[2] >= _bbox_min[2]);
    bool bboxValid = bboxFinite && bboxOrdered;

    if (!bboxValid)
    {
        printf("Warning: invalid scene bounding box (min=[%g,%g,%g], max=[%g,%g,%g]); using camera fallback values.\n",
            _bbox_min[0], _bbox_min[1], _bbox_min[2],
            _bbox_max[0], _bbox_max[1], _bbox_max[2]);
    }

    Vector3 center = Vector3(0.0f, 0.0f, 0.0f);
    float diagonal = kFallbackDiagonal;

    if (bboxValid)
    {
        Vector3 max = Vector3(_bbox_max[0], _bbox_max[1], _bbox_max[2]);
        Vector3 min = Vector3(_bbox_min[0], _bbox_min[1], _bbox_min[2]);
        center = Vector3((_bbox_max[0] + _bbox_min[0]) * 0.5f,
            (_bbox_max[1] + _bbox_min[1]) * 0.5f,
            (_bbox_max[2] + _bbox_min[2]) * 0.5f);

        diagonal = (max - min).length();
        if (!std::isfinite(diagonal) || diagonal <= 1e-4f)
        {
            diagonal = kFallbackDiagonal;
        }
    }

    if (camera->getNumViews() > 0)
    {
        camera->setView(0);
    }
    else
    {
        /* Set the camera 1.5 times its diagonal length away from the center */
        Vector3 direction = Vector3(0.0f, 0.0f, 1.0f);
        Vector3 position = center + direction * diagonal * 1.5f;
        camera->setPosition(position);
    }

    camera->setFov(45.0f);

    // Near plane: 1% of diagonal (or minimum fallback near)
    float nearPlane = std::max(kFallbackNear, diagonal * 0.01f);

    // Far plane: 10x diagonal, but always keep a healthy ratio to near.
    float farPlane = std::max(diagonal * 10.0f, nearPlane * 100.0f);

    if (!std::isfinite(nearPlane) || !std::isfinite(farPlane) || farPlane <= nearPlane)
    {
        printf("Warning: invalid camera clip planes computed (near=%g, far=%g); using defaults.\n",
            nearPlane, farPlane);
        nearPlane = kFallbackNear;
        farPlane = kFallbackFar;
    }

    //printf("BBox diagonal=%.2f, near=%.2f, far=%.2f, ratio=%.1f:1\n",
    //    diagonal, nearPlane, farPlane, farPlane / nearPlane);

    camera->setNear(nearPlane);
    camera->setFar(farPlane);
}

void Scene::recommendLightingDefaults(float *ambientWeight, float *diffuseWeight,
    float *sunIntensity) const
{
    const float defaultAmbient = 0.6f;
    const float defaultDiffuse = 1.0f;
    const float defaultSunIntensity = 1.0f;

    double sumDiffuseLuma = 0.0;
    double sumDiffuseSaturation = 0.0;
    double sumSpecularLuma = 0.0;
    uint32_t count = 0;

    if (_products != NULL)
    {
        for (uint32_t i = 0; i < _nproducts; i++)
        {
            const Product &product = _products[i];
            const Material *materials = product.materials();
            uint32_t numMaterials = product.numMaterials();

            if (materials == NULL || numMaterials == 0)
                continue;

            for (uint32_t m = 0; m < numMaterials; m++)
            {
                const Vector3 &d = materials[m].diffuse;
                const Vector3 &s = materials[m].specular;

                float dMax = fmaxf(d.x, fmaxf(d.y, d.z));
                float dMin = fminf(d.x, fminf(d.y, d.z));
                float dSat = (dMax > 1e-5f) ? ((dMax - dMin) / dMax) : 0.0f;

                float dLuma = 0.2126f * d.x + 0.7152f * d.y + 0.0722f * d.z;
                float sLuma = 0.2126f * s.x + 0.7152f * s.y + 0.0722f * s.z;

                sumDiffuseLuma += dLuma;
                sumDiffuseSaturation += dSat;
                sumSpecularLuma += sLuma;
                count++;
            }
        }
    }

    if (count == 0)
    {
        if (ambientWeight)
            *ambientWeight = defaultAmbient;
        if (diffuseWeight)
            *diffuseWeight = defaultDiffuse;
        if (sunIntensity)
            *sunIntensity = defaultSunIntensity;
        return;
    }

    {
        float avgDiffuseLuma = (float)(sumDiffuseLuma / count);
        float avgDiffuseSat = (float)(sumDiffuseSaturation / count);
        float avgSpecularLuma = (float)(sumSpecularLuma / count);

        float ambient = 0.72f
            - 0.20f * avgDiffuseLuma
            - 0.12f * avgDiffuseSat
            + 0.08f * (1.0f - avgSpecularLuma);

        float diffuse = 0.95f
            + 0.18f * avgDiffuseSat
            + 0.10f * (1.0f - avgDiffuseLuma)
            - 0.06f * avgSpecularLuma;

        float sun = 0.95f
            + 0.30f * (1.0f - avgDiffuseLuma)
            - 0.18f * avgSpecularLuma;

        if (ambientWeight)
            *ambientWeight = clamp(ambient, 0.25f, 0.90f);
        if (diffuseWeight)
            *diffuseWeight = clamp(diffuse, 0.80f, 1.35f);
        if (sunIntensity)
            *sunIntensity = clamp(sun, 0.45f, 1.35f);
    }
}

void Scene::recommendLightingWeights(float *ambientWeight, float *diffuseWeight) const
{
    recommendLightingDefaults(ambientWeight, diffuseWeight, NULL);
}

void Scene::setShowExtraWireOverlays(bool enabled)
{
    _showExtraWireOverlays = enabled;

    if (_products != NULL && _nproducts > 0)
    {
        setCompanionVisibilityRecursive(&_products[0], enabled);
    }
}

void Scene::render(Camera *camera)
{
    glViewport(0, 0, _renderWidth, _renderHeight);

    camera->update();

    if (_enableGround && 0)
    {
        _groundModel = Matrix4(1.0f);
        _groundModel = mutil::scale(_groundModel, Vector3(_groundSize.x, 1.0f, _groundSize.y));
        _groundModel = mutil::translate(_groundModel, Vector3(0.0f, _groundHeight, 0.0f));
    }

    //glDisable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDisable(GL_CULL_FACE);

#ifdef __APPLE__
    /* Fallback path for stricter macOS drivers: render directly to default FBO. */
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, _width, _height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    if (_wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    if (_backfaceCull)
        glEnable(GL_CULL_FACE);
    else
        glDisable(GL_CULL_FACE);

    renderModels(camera);

    glDisable(GL_CULL_FACE);

    if (!_wireframe)
    {
        glDepthFunc(GL_LEQUAL);
        renderSkybox(camera);
    }

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    return;
#endif

    /* Shadow mapping */
    if (!_wireframe && _shadowMap)
    {
        _shadowMap->bind();
        glViewport(0, 0, _shadowMap->resolution(), _shadowMap->resolution());

        glClear(GL_DEPTH_BUFFER_BIT);

        renderLightmap();
    }

    /* Standard rendering */
    // glEnable(GL_CULL_FACE);
    _compositor->use();
    glViewport(0, 0, _compositor->width(), _compositor->height());

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    if (_wireframe)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    if (_backfaceCull)
        glEnable(GL_CULL_FACE);
    else
        glDisable(GL_CULL_FACE);

    renderModels(camera);

    glDisable(GL_CULL_FACE);

    if (!_wireframe)
    {
        glDepthFunc(GL_LEQUAL);
        renderSkybox(camera);

        /* Bloom */
        if (_enableBloom)
        {
            _bloom->use();
            _bloom->render(_compositor->texture());
        }
    }

    /* Composite render */
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, _width, _height);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    _compositeShader->getShader().use();

    _compositeShader->setEnableBloom(_enableBloom);
    _compositeShader->setBloomStrength(_bloom->strength());
    _compositeShader->setEnableToneMapping(_enableToneMapping);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _compositor->texture());
    _compositeShader->setTexture(0);

    if (_enableBloom)
    {
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, _bloom->texture());
        _compositeShader->setBloom(1);
    }

    _compositeShader->setGamma(_compositor->gamma());
    _compositeShader->setExposure(_compositor->exposure());

    _compositor->render();
}

static void setBounds(prc_context *ctx, Product *app_child, prc_api_tess *tess,
    Matrix4 matrix, uint8_t is_identity, Vector4 *minBound, Vector4 *maxBound)
{
    Vector4 min_bound;
    Vector4 max_bound;

    app_child->setBoundingBox(tess->bounding_box_min, tess->bounding_box_max);

    /* Adjust the bounding boxes for the model by the matrix */
    if (!is_identity)
    {
        min_bound = matrix * app_child->bboxMin();
        max_bound = matrix * app_child->bboxMax();
        app_child->setBoundingBox(min_bound, max_bound);
    }

    min_bound = app_child->bboxMin();
    max_bound = app_child->bboxMax();

    minBound->x = std::fmin(min_bound.x, minBound->x);
    minBound->y = std::fmin(min_bound.y, minBound->y);
    minBound->z = std::fmin(min_bound.z, minBound->z);

    maxBound->x = std::fmax(max_bound.x, maxBound->x);
    maxBound->y = std::fmax(max_bound.y, maxBound->y);
    maxBound->z = std::fmax(max_bound.z, maxBound->z);
}

void Scene::addRepItems(prc_context *ctx, prc_api_data data, prc_api_part *api_part,
    Product *app_product, Product *heap, uint32_t *product_count,
    Vector4 *minBound, Vector4 *maxBound, prc_api_transform location, Matrix4 matrix)
{
    uint32_t k;
    prc_api_tess *tess;
    prc_api_tess *tess_line;

    app_product->createChildren(api_part->num_rep_items);

    for (k = 0; k < api_part->num_rep_items; k++)
    {
        Product *app_child = app_product->getChildFromHeap(k, heap, product_count);
        *product_count += 1;
        app_child->setParent(app_product);
        app_child->setName(api_part->rep_items[k].name);
        app_child->setModel(Matrix4(1.0f));
        app_child->setBoundingBox(Vector4(0.0f), Vector4(0.0f));
        if (api_part->rep_items[k].attributes.num_base_attributes > 0)
        {
            app_child->setAttributes(&api_part->rep_items[k].attributes);
        }

        /* Get the tessellation for this model (part) */
        tess = prc_api_get_ri_tessellation(ctx, api_part, k);
        tess_line = prc_api_get_ri_line_tessellation(ctx, api_part, k);

        if (tess != NULL)
        {
            setBounds(ctx, app_child, tess, matrix, location.is_identity, minBound,
                maxBound);

            /* Attach the model */
            app_child->attach(ctx, data, tess, *getTextRenderPtr());

            if (tess_line != NULL && tess_line->type == PRC_API_TESS_3D_Wire_Extra)
            {
                Product *line_companion = &heap[*product_count];
                *product_count += 1;

                line_companion->setParent(app_child);
                line_companion->setEnabled(_showExtraWireOverlays);
                line_companion->setModel(Matrix4(1.0f));
                line_companion->setBoundingBox(app_child->bboxMin(), app_child->bboxMax());
                line_companion->attach(ctx, data, tess_line, *getTextRenderPtr());

                app_child->setRenderCompanion(line_companion);
            }
        }
        if (api_part->rep_items[k].num_rep_items > 0)
        {
            /* This is a recursive case */
            addRepItems(ctx, data, api_part->rep_items + k, app_child, heap,
                product_count, minBound, maxBound, location, matrix);
        }
    }
}

/* A recursive method that will go through the tree that is provided by nanoPRC and
   convert it to the tree that is used by this application */
void Scene::convertTree(prc_context *ctx, prc_api_data data, prc_api_product *api_product,
                 Product *app_product, Product *heap, uint32_t *product_count,
                 Vector4 *minBound, Vector4 *maxBound, uint8_t *font_atlas_created)
{
    uint8_t has_tess = prc_api_model_item_is_part(ctx, api_product);
    prc_api_tess *tess;
    prc_api_transform location = api_product->location;
    uint32_t num_children;
    uint32_t k;
    Matrix4 matrix = Matrix4(1.0f);
    uint8_t include_part;
    uint32_t start_index;
    uint32_t num_markups = prc_api_model_item_number_of_markups(ctx, api_product);
    uint8_t has_part = api_product->part != NULL;

    app_product->setName(api_product->name);
    if (!location.is_identity)
    {
        /* Set the tranformation matrix in the model*/
        matrix.columns[0] = Vector4((float)location.matrix[0], (float)location.matrix[1],
            (float)location.matrix[2], (float)location.matrix[3]);
        matrix.columns[1] = Vector4((float)location.matrix[4], (float)location.matrix[5],
            (float)location.matrix[6], (float)location.matrix[7]);
        matrix.columns[2] = Vector4((float)location.matrix[8], (float)location.matrix[9],
            (float)location.matrix[10], (float)location.matrix[11]);
        matrix.columns[3] = Vector4((float)location.matrix[12], (float)location.matrix[13],
            (float)location.matrix[14], (float)location.matrix[15]);

        app_product->setModel(matrix);
    }
    else
    {
        app_product->setModel(Matrix4(1.0f));
    }

    if (api_product->attributes.num_base_attributes > 0)
    {
        app_product->setAttributes(&api_product->attributes);
    }
    *product_count += 1;

    /* Special yoga pose if we have a part */
    include_part = has_part && (api_product->part->num_rep_items > 0);
    start_index = include_part ? 1 : 0;

    if (api_product->num_children > 0 || include_part)
    {
        num_children = api_product->num_children + include_part;
        app_product->createChildren(num_children); /* Get num_children ptr to ptrs */

        if (include_part)
        {
            /* Get the part child and set the parent as the product */
            Product *app_child = app_product->getChildFromHeap(0, heap, product_count);
            app_child->setParent(app_product);
            app_child->setName(api_product->part->name);
            app_child->setModel(Matrix4(1.0f));

            if (api_product->part->attributes.num_base_attributes > 0)
            {
                app_child->setAttributes(&api_product->part->attributes);
            }
            *product_count += 1;

            /* Add the RI items as children of the part child Product (app_child) */
            addRepItems(ctx, data, api_product->part, app_child, heap, product_count,
                minBound, maxBound, location, matrix);
        }

        /* Now go through the children that this may have beyond the part we just added.
           If we have a "3D PMI" product that is when the markups are stuffed */
        for (size_t i = 0; i < api_product->num_children; i++)
        {
            Product *app_child = app_product->getChildFromHeap(i + start_index, heap, product_count); /* Note no + after this as it occurs in convertTree */
            app_child->setParent(app_product);
            convertTree(ctx, data, api_product->children + i, app_child, heap,
                product_count, minBound, maxBound, font_atlas_created);
        }
    }

    if (num_markups > 0)
    {
        /* This should be essentially a "3D PMI" node with layout tessellations as children */
        app_product->createChildren(num_markups); /* Get num_markups ptr to ptrs */

        for (k = 0; k < num_markups; k++)
        {
            /* If it has both graphics and text primitives we will create a
             * parent and then two children, one for the graphics and one for
             * the text. If it has only graphics or only text, we will not create
             * the parent */
            tess = prc_api_get_markup_tessellation(ctx, api_product, k);
            if (tess == NULL)
            {
                /* No tessellation resolved for this markup (e.g. an invalid or
                   unresolved biased_index_tessellation in the file). Still
                   create a tree node for consistency, but skip geometry/bounds
                   since there is nothing to read -- dereferencing a NULL tess
                   here would crash. */
                Product *app_child = app_product->getChildFromHeap(k, heap, product_count);
                *product_count += 1;
                app_child->setParent(app_product);
                app_child->setName(api_product->markup[k].name);
                app_child->setModel(Matrix4(1.0f));
                continue;
            }
            uint8_t needs_parent = (tess->num_line_primitives > 0 && tess->num_text_primitives > 0);

            if (needs_parent)
            {
                /* Create the parent for this markup */
                Product *app_markup_parent = app_product->getChildFromHeap(k, heap, product_count);
                *product_count += 1;

                app_markup_parent->createChildren(2);
                app_markup_parent->setParent(app_product);
                app_markup_parent->setName(api_product->markup[k].name);
                app_markup_parent->setModel(Matrix4(1.0f));
                setBounds(ctx, app_markup_parent, tess, matrix, location.is_identity, minBound,
                    maxBound);

                /* Set the the markup (non-text) child */
                Product *app_child = app_markup_parent->getChildFromHeap(0, heap, product_count);
                *product_count += 1;
                app_child->setParent(app_markup_parent);

                /* Append a _1 to the name */
                std::string markup_name = std::string(api_product->markup[k].name) + "_1";
                app_child->setName(markup_name.c_str());

                app_child->setModel(Matrix4(1.0f));
                setBounds(ctx, app_child, tess, matrix, location.is_identity, minBound,
                    maxBound);
                app_child->attach(ctx, data, tess, *getTextRenderPtr());

                /* Now handle the text portion */
                Product *app_child_text = app_markup_parent->getChildFromHeap(1, heap, product_count);
                *product_count += 1;
                app_child_text->setParent(app_markup_parent);

                /* Append a _2 to the name */
                std::string markup_name2 = std::string(api_product->markup[k].name) + "_2";
                app_child_text->setName(markup_name2.c_str());

                app_child_text->setModel(Matrix4(1.0f));
                setBounds(ctx, app_child_text, tess, matrix, location.is_identity, minBound,
                    maxBound);
                app_child_text->attachTextContent(ctx, data, tess, *getTextRenderPtr());
            }
            else if (tess->num_line_primitives > 0 || tess->num_text_primitives > 0)
            {
                Product *app_child = app_product->getChildFromHeap(k, heap, product_count);
                *product_count += 1;
                app_child->setParent(app_product);

                /* Get the tessellation for this model (part) */
                tess = prc_api_get_markup_tessellation(ctx, api_product, k);
                app_child->setName(api_product->markup[k].name);
                app_child->setModel(Matrix4(1.0f));
                setBounds(ctx, app_child, tess, matrix, location.is_identity, minBound,
                    maxBound);

                /* Just a single child for this markup */
                if (tess->num_line_primitives > 0)
                {
                    app_child->attach(ctx, data, tess, *getTextRenderPtr());
                }
                else if (tess->num_text_primitives > 0)
                {
                    app_child->attachTextContent(ctx, data, tess, *getTextRenderPtr());
                }
            }
            else if (tess->num_line_primitives == 0 && tess->num_text_primitives == 0)
            {
                /* No geometry for this markup, but we still want to create a node for it in the tree */
                Product *app_child = app_product->getChildFromHeap(k, heap, product_count);
                *product_count += 1;
                app_child->setParent(app_product);
                app_child->setName(api_product->markup[k].name);
                app_child->setModel(Matrix4(1.0f));
                setBounds(ctx, app_child, tess, matrix, location.is_identity, minBound,
                    maxBound);
                /* Just don't attach it though */
            }
        }
    }
}

void Scene::load(const char *infile, Camera *camera, bool memoryLeakCheck)
{
    uint32_t tess_offset = 0;
    uint32_t num_models, num_products;
    uint32_t total_models = 0;
    uint32_t total_products = 0;
    uint32_t num_proc, num_parts, num_markups;
    char *model_name = NULL;
    prc_api_product *model_tree;
    uint32_t totalTesselations;
    uint32_t totalLineTesselations;
    int code;
    uint32_t j, k;
    int context_release_code;

    prc_context *ctx = prc_api_new_context(NULL);
    if (ctx == NULL)
    {
        printf("Scene::load: prc_api_new_context failed\n");
        exit(1);
    }

    /* Do all the parsing */
    prc_api_data data = prc_api_open_contents(ctx, infile);
    if (data == NULL)
    {
        /* If memory checking enabled this should catch any leaks that might
           occur due to a parsing error. Those should have been handled
           but this will provide a report if PRC_DEBUG_MEMORY is defined. */
        code = prc_api_release_context(ctx);
        printf("Scene::load: prc_api_open_contents failed\n");
        exit(1);
    }
    prc_api_print_error_stack(ctx);

    /* Lets create the model tree. */
    code = prc_api_prep_model_tree(ctx, data, &num_parts, &num_products, &num_markups);
    if (code < 0)
    {
        printf("Scene::load: prc_api_prep_model_tree failed\n");
        exit(1);
    }

    code = prc_api_create_model_tree(ctx, data, &model_tree, num_parts,
                                     num_products, num_markups);
    if (code < 0)
    {
        printf("Scene::load: prc_api_create_model_tree failed\n");
        exit(1);
    }

    /* Get the total number of tessellations we need, based upon the parts,
       the styles, the markups. Parts that use the same tessellation/style but just
       vary in spatial transformations will be using the same vertex information.
       Of course we still have to worry about the various faces of the
       tessellations.  Also, we may have to add line (wire) type tessellations
       when we in fact have a 3D compressed or 3D uncompressed tessellation.
       The compressed ones, we add line data for the extreme crease angles (not
       in the spec but Adobe does this) and we can have line data in the uncompressed
       tessellation */
    code = prc_api_get_number_tessellations(ctx, data, model_tree, &totalTesselations,
                                            &totalLineTesselations);
    if (code < 0)
    {
        printf("Scene::load: prc_api_get_number_tessellations failed\n");
        exit(1);
    }
    prc_api_tess *tesses = new prc_api_tess[totalTesselations];
    prc_api_tess *tesses_line = NULL;
    uint32_t line_tess_index = 0;

    if (tesses == NULL)
    {
        printf("Scene::load: failed to allocate tessellation array\n");
        exit(1);
    }
    if (totalLineTesselations > 0)
    {
        tesses_line = new prc_api_tess[totalLineTesselations];
        if (tesses_line == NULL)
        {
            printf("Scene::load: failed to allocate line tessellation array\n");
            exit(1);
        }
    }

    /* Lets go through all the tessellations and get the data assigning it to
       the part and markups. We have to worry about different faces due to
       the styles that can vary across them.  Also, the 3D tessellations
       can generate line (wire) tessellations */
    for (k = 0; k < totalTesselations; k++)
    {
        prc_api_tess &tess = tesses[k];
        uint8_t has_line = 0;
        prc_api_tess *tess_line;

        if (totalLineTesselations > 0)
        {
            tess_line = &tesses_line[line_tess_index];
        }
        else
        {
            tess_line = NULL;
        }

        uint32_t nFaces = prc_api_get_number_faces(ctx, data, k);

        /* If we are dealing with a compressed tessellation, force nFaces
           to one as prc_api_get_tessellation_vertices will handle this
           appropriately */
        if (prc_api_get_tessellation_type(ctx, data, k) == PRC_API_TESS_3D_Compressed)
        {
            nFaces = 1;
        }

        tess.num_faces = nFaces;
        tess.tess_faces = new prc_api_face[nFaces];

        int code = prc_api_initialize_tessellation(ctx, data, model_tree, k, &tess,
                                                   tess_line, &has_line);
        if (code < 0)
        {
            printf("Scene::load: prc_api_initialize_tessellation failed\n");
            exit(1);
        }

        /* Tess line sits in parallel in terms of faces with tess */
        if (has_line)
        {
            tess_line->num_faces = nFaces;
            tess_line->tess_faces = new prc_api_face[nFaces];
        }

        if (tess.type == PRC_API_TESS_UNKNOWN)
        {
            /* Who knows what this is all about. */
            continue;
        }

        if (tess.type == PRC_API_TESS_3D_Wire ||
            tess.type == PRC_API_TESS_MarkUp)
        {
            /* 3D wire case and 3D markup cases */
            code = prc_api_get_tessellation_vertices(ctx, data, model_tree,
                                                k, 0, NULL, &tess);
            if (code < 0)
            {
                printf("Scene::load: prc_api_get_tessallation_vertices failed\n");
                exit(1);
            }
        }
        else
        {
            for (j = 0; j < tess.num_faces; j++)
            {
                code = prc_api_get_tessellation_vertices(ctx, data, model_tree,
                                                        k, j, tess.tess_faces + j,
                                                        &tess);
                if (code < 0)
                {
                    printf("Scene::load: prc_api_get_tessallation_vertices failed\n");
                    exit(1);
                }
            }
        }

        /* This is a special case where we have wire data in the 3D tessellation
           data, or we have added wire data in the compressed 3D tessellation.
           This gets put into a hidden object in the model tree */
        if (has_line)
        {
            /* We have a line tessellation to add for this model. Get the line
               tessellation data. Note if it is a 3D tessellation some faces may
               have lines and some may not. */
            for (j = 0; j < tess.num_faces; j++)
            {
                 code = prc_api_get_line_tessellation_vertices(ctx, data, model_tree,
                                                    k, j, tess_line->tess_faces + j,
                                                    tess_line);
                if (code < 0)
                {
                    printf("Scene::load: prc_api_get_line_tessallation_vertices failed\n");
                    exit(1);
                }
            }
            line_tess_index++;
        }
    }

    Vector4 min_bound;
    Vector4 max_bound;

    max_bound.x = FLT_MIN;
    max_bound.y = FLT_MIN;
    max_bound.z = FLT_MIN;
    min_bound.x = FLT_MAX;
    min_bound.y = FLT_MAX;
    min_bound.z = FLT_MAX;

    /* Allocate 3x on the markups in case we have text and lines (parent plus
       one for text and one for lines)*/
    _nproducts = num_products + num_parts + 3 * num_markups + totalLineTesselations;
    _products = new Product[_nproducts];
    uint32_t product_index = 0;
    uint8_t font_atlas_created = 0;

    /* This is used to emit the vertices for the text drawing which are determined
       during the tree conversion. */
    _textRenderer = new Graphics2D();
    _textRenderer->load();

    convertTree(ctx, data, model_tree, &_products[0], _products,
        &product_index, &min_bound, &max_bound, &font_atlas_created);

    _bbox_min[0] = min_bound.x;
    _bbox_min[1] = min_bound.y;
    _bbox_min[2] = min_bound.z;
    _bbox_max[0] = max_bound.x;
    _bbox_max[1] = max_bound.y;
    _bbox_max[2] = max_bound.z;

    /* Lets get the camera views if we have any */
    uint32_t num_views = prc_api_get_number_of_view(ctx, data);
    if (num_views > 0)
    {
        double *matrix;
        double camera_z;
        char *name;

        camera->setNumViews(num_views);
        for (uint32_t i = 0; i < num_views; i++)
        {
            code = prc_api_get_view(ctx, data, i, &name, &matrix, &camera_z);
            if (code < 0)
            {
                printf("Scene::load: prc_api_get_view failed\n");
                exit(1);
            }
            camera->addView(matrix, camera_z, i, name);
        }
    }
    else
    {
        /* Create a default view based on bounding box */
        Vector3 max = Vector3(_bbox_max[0], _bbox_max[1], _bbox_max[2]);
        Vector3 min = Vector3(_bbox_min[0], _bbox_min[1], _bbox_min[2]);
        float diagonal = (max - min).length();
        Vector3 center = Vector3((_bbox_max[0] + _bbox_min[0]) * 0.5f,
            (_bbox_max[1] + _bbox_min[1]) * 0.5f,
            (_bbox_max[2] + _bbox_min[2]) * 0.5f);

        /* Position camera at a reasonable distance to see the entire object */
        float distance = diagonal * 1.5f;

        /* Create a standard isometric-style view similar to CAD applications
           Pitch ~-30 degrees (looking down), Yaw ~45 degrees (from front-right) */
        float pitch = -30.0f;
        float yaw = 45.0f;
        float roll = 0.0f;

        /* Build rotation matrices matching Camera::update() convention */
        double pitch_rad = mutil::radians((double)pitch);
        double yaw_rad = mutil::radians((double)yaw);
        double roll_rad = mutil::radians((double)roll);

        /* These match the rotation order in Camera::update(): R_phi * R_theta * R_psi */
        Matrix3 R_yaw = Matrix3(1, 0, 0,
            0, std::cos(yaw_rad), -std::sin(yaw_rad),
            0, std::sin(yaw_rad), std::cos(yaw_rad));

        Matrix3 R_pitch = Matrix3(std::cos(pitch_rad), 0, std::sin(pitch_rad),
            0, 1, 0,
            -std::sin(pitch_rad), 0, std::cos(pitch_rad));

        Matrix3 R_roll = Matrix3(std::cos(roll_rad), -std::sin(roll_rad), 0,
            std::sin(roll_rad), std::cos(roll_rad), 0,
            0, 0, 1);

        Matrix3 R = R_roll * R_pitch * R_yaw;

        /* Transform the camera's forward direction (+Z in camera space) by rotation */
        Vector3 camera_forward = Vector3(R.columns[2].x, R.columns[2].y, R.columns[2].z);

        /* Position camera behind the object (opposite of forward direction) */
        Vector3 position = center - camera_forward * distance;

        /* Build PRC camera matrix (column-major: 3x3 rotation + position) */
        double default_matrix[12] = {
            R.columns[0].x, R.columns[0].y, R.columns[0].z,     // Column 0
            R.columns[1].x, R.columns[1].y, R.columns[1].z,     // Column 1
            R.columns[2].x, R.columns[2].y, R.columns[2].z,     // Column 2
            position.x, position.y, position.z                   // Column 3
        };

        /* Center of orbit distance */
        double camera_z_orbit = distance;

        char *default_name = new char[8];
        strcpy(default_name, "default");

        camera->setNumViews(1);
        camera->addView(default_matrix, camera_z_orbit, 0, default_name);
    }
    /* Set the initial camera position based on bounding box */
   // setCameraInitialPosition(camera);

    /* Clean up */
    prc_api_release_data(ctx, data, tesses, totalTesselations, tesses_line,
        totalLineTesselations, model_tree);

    for (uint32_t i = 0; i < totalTesselations; i++)
        delete[] tesses[i].tess_faces;
    delete[] tesses;

    if (totalLineTesselations > 0)
    {
        delete[] tesses_line;
    }

    context_release_code = prc_api_release_context(ctx);
    if (memoryLeakCheck && context_release_code == PRC_API_MEMORY_LEAK_DETECTED)
    {
        printf("Scene::load: memory leak detected while releasing PRC context\n");
        exit(PRC_API_MEMORY_LEAK_DETECTED);
    }

    _meshShader = new MeshShader();
    _meshShader->load();

    _skyboxShader = new SkyboxShader();
    _skyboxShader->load();

    _skybox = new Skybox();
    _skybox->load();

    _groundPlane = new Mesh();
    _groundPlane->load(GL_TRIANGLES, kPlaneVertices, 4, kPlaneIndices, 6);
    *_groundPlane->material() = kMaterial;

    _compositeShader = new CompositeShader();
    _compositeShader->load();

    _compositor = new Compositor();
    _compositor->load();

    _bloom = new Bloom();
    _bloom->load();
}

void Scene::unload()
{
    if (_bloom)
        delete _bloom, _bloom = nullptr;

    if (_compositor)
        delete _compositor, _compositor = nullptr;

    if (_compositeShader)
        delete _compositeShader, _compositeShader = nullptr;

    if (_shadowMap)
        delete _shadowMap, _shadowMap = nullptr;

    if (_shadowShader)
        delete _shadowShader, _shadowShader = nullptr;

    if (_skybox)
        delete _skybox, _skybox = nullptr;

    if (_skyboxShader)
        delete _skyboxShader, _skyboxShader = nullptr;

    if (_meshShader)
        delete _meshShader, _meshShader = nullptr;

    if (_products)
        delete[] _products, _products = nullptr;
}

void Scene::resize(int width, int height, float renderScale, int shadowResolution, bool enableShadows)
{
    _width = width;
    _height = height;

    _renderWidth = static_cast<int>(width * renderScale);
    _renderHeight = static_cast<int>(height * renderScale);

    _compositor->resize(_renderWidth, _renderHeight);
    _bloom->resize(_renderWidth, _renderHeight);

    if (enableShadows)
    {
        if (!_shadowShader)
        {
            _shadowShader = new ShadowShader();
            _shadowShader->load();
        }

        if (_shadowMap)
            delete _shadowMap, _shadowMap = nullptr;

        _shadowMap = new ShadowMap();
        _shadowMap->load(shadowResolution);
    }
    else
    {
        if (_shadowShader)
            delete _shadowShader, _shadowShader = nullptr;

        if (_shadowMap)
			delete _shadowMap, _shadowMap = nullptr;
    }
}

Scene::Scene() : _products(nullptr), _nproducts(0),
                 _meshShader(nullptr),
                 _skybox(nullptr), _skyboxShader(nullptr),
                 _wireframe(false),
                 _groundPlane(nullptr),
                 _groundHeight(-10.0f), _groundSize(Vector2(100.0f, 100.0f)), _enableGround(false),
                 _shadowShader(nullptr), _shadowMap(nullptr),
                 _compositeShader(nullptr), _compositor(nullptr),
                 _fullbright(false), _ambientWeight(1.0f), _diffuseWeight(1.0f),
                 _normalDebugMode(0), _backfaceCull(false),
                 _enableBloom(true), _bloom(nullptr),
                 _enableToneMapping(true), _enableMotion(true),
                 _showExtraWireOverlays(true),
                 _width(0), _height(0), _textRenderer(nullptr),
                 _renderWidth(0), _renderHeight(0)
{
}

Scene::~Scene()
{
    unload();
}

void Scene::renderModels(Camera *camera) const
{
    _meshShader->getShader().use();

    /* Camera */
    _meshShader->setView(camera->view());
    _meshShader->setProjection(camera->projection());
    _meshShader->setViewPos(camera->position());

    /* Lighting */
    _meshShader->setDirLight(_dirLight);
    _meshShader->setPointLight(_pointLight);
    _meshShader->setSkyColorHorizon(_skybox->horizonColor());
    _meshShader->setSkyColorZenith(_skybox->zenithColor());

    _meshShader->setWireframe(_wireframe);
    _meshShader->setFullbright(_fullbright);
    _meshShader->setAmbientWeight(_ambientWeight);
    _meshShader->setDiffuseWeight(_diffuseWeight);
    _meshShader->setDebugMode(_normalDebugMode);

    /* Shadow mapping */
    if (_shadowMap)
    {
        _meshShader->setLightSpaceMatrix(_shadowMap->lightSpaceMatrix());
        _meshShader->setShadowMap(SHADOW_MAP_TEXTURE_UNIT);
        glActiveTexture(GL_TEXTURE0 + SHADOW_MAP_TEXTURE_UNIT);
        glBindTexture(GL_TEXTURE_2D, _shadowMap->depthMap());

        _meshShader->setEnableShadows(true);
    }
    else
        _meshShader->setEnableShadows(false);

    /* Render products.  Start out at root and work through the children */
    Product &root = _products[0];
    root.update();
    root.render(_meshShader);

    /* Render ground */
    if (_enableGround && 0)
    {
        _meshShader->setModel(_groundModel);
        _meshShader->setMaterial(*_groundPlane->material());

        _groundPlane->render();
    }
}

void Scene::renderSkybox(Camera *camera) const
{
    _skyboxShader->getShader().use();

    Matrix4 skyView = Matrix4(Matrix3(camera->view()));
    _skyboxShader->setSkyView(skyView);

    _skyboxShader->setProjection(camera->projection());

    _skyboxShader->setSunDir(_dirLight.direction);
    _skyboxShader->setSunColor(_dirLight.color);
    _skyboxShader->setSunIntensity(_dirLight.intensity);

    _skyboxShader->setSkyColorHorizon(_skybox->horizonColor());
    _skyboxShader->setSkyColorZenith(_skybox->zenithColor());

    _skyboxShader->setSunTightness(_skybox->sunTightness());

    _skybox->render();
}

void Scene::renderLightmap() const
{
 /*   _shadowShader->getShader().use();

    _shadowMap->setLightDirection(_dirLight.direction);

    _shadowShader->setLightSpaceMatrix(_shadowMap->lightSpaceMatrix());

    for (uint32_t i = 0; i < _nModels; i++)
    {
        Model &model = _models[i];
        if (model.enabled())
        {
            _shadowShader->setModel(model.model());
            _models[i].render();
        }
    }

    if (_enableGround)
    {
        _shadowShader->setModel(_groundModel);
        _groundPlane->render();
    } */
}
