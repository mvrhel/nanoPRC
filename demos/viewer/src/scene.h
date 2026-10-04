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

#ifndef _SCENE_H_
#define _SCENE_H_

#include <cstdint>
#include "text.h"
#include "shader.h"

class Camera;
class Mesh;
class Model;
class MeshShader;
class Skybox;
class SkyboxShader;
class ShadowShader;
class ShadowMap;
class Compositor;
class CompositeShader;
class Bloom;
class Product;
class Graphics2D;

class Scene
{
public:
    void render(Camera *camera);
    void setCameraInitialPosition(Camera *camera);
    void recommendLightingWeights(float *ambientWeight, float *diffuseWeight) const;
    void recommendLightingDefaults(float *ambientWeight, float *diffuseWeight,
        float *sunIntensity) const;

    void load(const char *infile, Camera *camera, bool memoryLeakCheck = false, bool exactGeomOnly = false);
    void unload();

    constexpr Graphics2D *getTextRenderPtr() { return _textRenderer; }

    constexpr Product *models() { return _products; }
    constexpr const Product *models() const { return _products; }
    constexpr uint32_t mumProducts() const { return _nproducts; }

    constexpr Skybox *skybox() { return _skybox; }
    constexpr const Skybox *skybox() const { return _skybox; }

    constexpr bool &wireframe() { return _wireframe; }

    constexpr Mesh *ground() { return _groundPlane; }
    constexpr bool &enableGround() { return _enableGround; }
    constexpr float &groundHeight() { return _groundHeight; }
    constexpr Vector2 &groundSize() { return _groundSize; }

    constexpr bool &fullbright() { return _fullbright; }
    constexpr bool &backfaceCull() { return _backfaceCull; }
    constexpr float &ambientWeight() { return _ambientWeight; }
    constexpr float ambientWeight() const { return _ambientWeight; }
    constexpr float &diffuseWeight() { return _diffuseWeight; }
    constexpr float diffuseWeight() const { return _diffuseWeight; }
    constexpr int &normalDebugMode() { return _normalDebugMode; }
    constexpr int normalDebugMode() const { return _normalDebugMode; }

    constexpr DirLight *dirLight() { return &_dirLight; }
    constexpr const DirLight *dirLight() const { return &_dirLight; }

    constexpr PointLight *pointLight() { return &_pointLight; }
    constexpr const PointLight *pointLight() const { return &_pointLight; }

    constexpr ShadowMap *shadowMap() { return _shadowMap; }
    constexpr const ShadowMap *shadowMap() const { return _shadowMap; }

    constexpr Compositor *compositor() { return _compositor; }
    constexpr const Compositor *compositor() const { return _compositor; }

    constexpr bool &enableBloom() { return _enableBloom; }
    constexpr Bloom *bloom() { return _bloom; }
    constexpr const Bloom *bloom() const { return _bloom; }

    constexpr bool &enableToneMapping() { return _enableToneMapping; }

    constexpr bool &enableMotion() { return _enableMotion; }

    void setShowExtraWireOverlays(bool enabled);
    constexpr bool showExtraWireOverlays() const { return _showExtraWireOverlays; }

    constexpr int width() const { return _width; }
    constexpr int height() const { return _height; }

    constexpr int renderWidth() const { return _renderWidth; }
    constexpr int renderHeight() const { return _renderHeight; }

    void resize(
        int width, int height, float renderScale,
        int shadowResolution, bool enableShadows);

    Scene();
    ~Scene();

private:
    //Model *_models;
    //uint32_t _nModels;

    Product *_products;
    uint32_t _nproducts;

    MeshShader *_meshShader;

    Graphics2D *_textRenderer;

    SkyboxShader *_skyboxShader;
    Skybox *_skybox;

    bool _wireframe;
    bool _backfaceCull;

    /* Ground */
    Mesh *_groundPlane;
    bool _enableGround;
    float _groundHeight;
    Vector2 _groundSize;
    Matrix4 _groundModel;

    /* Lighting */
    bool _fullbright;
    float _ambientWeight;
    float _diffuseWeight;
    int _normalDebugMode;

    DirLight _dirLight;
    PointLight _pointLight;

    ShadowShader *_shadowShader;
    ShadowMap *_shadowMap;

    CompositeShader *_compositeShader;
    Compositor *_compositor;

    bool _enableBloom;
    Bloom *_bloom;

    bool _enableToneMapping;

    bool _enableMotion;

    bool _showExtraWireOverlays;

    int _width, _height;
    int _renderWidth, _renderHeight;

    double _bbox_min[3];
    double _bbox_max[3];

    void renderModels(Camera *camera) const;
    void renderSkybox(Camera *camera) const;

    void renderLightmap() const;

    void convertTree(prc_context *ctx, prc_api_data data, prc_api_product *api_product,
        Product *app_product, Product *heap, uint32_t *product_count,
        Vector4 *minBound, Vector4 *maxBound, uint8_t *font_atlas_created);

    void addRepItems(prc_context *ctx, prc_api_data data, prc_api_part *api_part,
        Product *app_product, Product *heap, uint32_t *product_count,
        Vector4 *minBound, Vector4 *maxBound, prc_api_transform location, Matrix4 matrix);
};

#endif // _SCENE_H_
