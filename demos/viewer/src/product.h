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

#ifndef _PRODUCT_H_
#define _PRODUCT_H_

#include <string.h>
#include <string>
#include <vector>
#include "shader.h"
#include "text.h"

class MeshShader;
class Graphics2D;

struct MeshSpec
{
    GLenum primitive;
    GLsizei offset;
    GLsizei numIndices;
    bool isMarkup;
    bool verticesHaveStyle;
    uint32_t faceIndex;
};

static constexpr Vector3 kObjectColor_emissive = Vector3(0.0f, 0.0f, 0.0f);
static constexpr Vector3 kObjectColor_diffuse = Vector3(0.1f, 0.1f, 0.1f);
static constexpr Vector3 kObjectColor_specular = Vector3(0.8f, 0.8f, 0.8f);
static constexpr Vector3 kObjectColor_tint = Vector3(0.0f, 0.0f, 0.0f);

static const Material kMaterial = {
    kObjectColor_emissive, // emissive
    kObjectColor_diffuse, // diffuse
    kObjectColor_specular, // specular
    0.4f,        // shininess
    1.0f,        // alpha -- we really need 4 alpha values.
    kObjectColor_tint // tint (ambient in PRC spec)
};

class Product final
{
public:
    struct AttributeRow
    {
        std::string title;
        std::string value;
    };

    struct AttributeBase
    {
        std::string title;
        std::vector<AttributeRow> entries;
    };

    void render(MeshShader *shader) const;
    void update();
    void invalidate();

    constexpr void setParent(Product *parent) { _parent = parent; }
    constexpr Product *parent() { return _parent; }

    constexpr void setEnabled(bool enabled) { _enabled = enabled; }
    constexpr bool enabled() const { return _enabled; }
    bool *enabledPtr() { return &_enabled; }

    void setName(const char *name) { _name = strdup(name); }
    constexpr const char *name() const { return _name; }

    void createChildren(uint32_t num_children);
    Product *getChildFromHeap(uint32_t child_index, Product *product_heap, uint32_t *index);

    constexpr Product **children() const { return _children; }
    constexpr uint32_t numChildren() const { return _nchildren; }
    constexpr void setRenderCompanion(Product *companion) { _renderCompanion = companion; }
    constexpr Product *renderCompanion() const { return _renderCompanion; }
    constexpr const Material *materials() const { return _material; }
    constexpr uint32_t numMaterials() const { return _numMaterials; }
    constexpr const MeshSpec *meshes() const { return _meshes; }
    constexpr uint32_t numMeshes() const { return _numMeshes; }
    constexpr const std::vector<prc_api_vertex> &cpuVertices() const { return _cpuVertices; }
    constexpr const std::vector<unsigned int> &cpuIndices() const { return _cpuIndices; }

    void attach(prc_context *ctx, prc_api_data data, const prc_api_tess *tess, Graphics2D &textRenderer);
    void attachTextContent(prc_context *ctx, prc_api_data data, const prc_api_tess *tess, Graphics2D &textRenderer);
    void setAttributes(prc_api_attributes *attributes);
    const std::vector<AttributeRow> &attributes() const { return _attributes; }
    const std::vector<AttributeBase> &attributeBases() const { return _attributeBases; }
    const char *attributesTitle() const { return _attributesTitle.empty() ? nullptr : _attributesTitle.c_str(); }
    bool hasAttributes() const { return !_attributes.empty(); }

    constexpr void setModel(const Matrix4 &m)
    {
        _model = m;
        _dirty = true;
    }

    constexpr const Matrix4 &model() const { return _model; }
    constexpr const Matrix4 &localToWorld() const { return _local_to_world; }

    constexpr const Vector4 &bboxMin() const { return _bbox_min; }
    constexpr const Vector4 &bboxMax() const { return _bbox_max; }
    constexpr void setBoundingBox(const double min[3], const double max[3])
    {
        _bbox_max = Vector4((float)max[0], (float)max[1], (float)max[2], 1.0f);
        _bbox_min = Vector4((float)min[0], (float)min[1], (float)min[2], 1.0f);
    }
    constexpr void setBoundingBox(const Vector4 &min, const Vector4 &max)
    {
        _bbox_max = max;
        _bbox_min = min;
    }

    Product();
    Product(const Product &) = delete;
    ~Product();

private:
    bool _enabled;

    char *_name;
    Product *_parent;

    Product **_children;
    uint32_t _nchildren;

     /* Optional non-UI child-like Product rendered with this Product.
         Used for auxiliary overlays (e.g. extra wire edges). */
     Product *_renderCompanion;

    bool _dirty;

    Matrix4 _model;
    Matrix4 _local_to_world;

    GLuint _vao, _vbo, _ebo;
    MeshSpec *_meshes;
    uint32_t _numMeshes;    /* Each face can have its own mesh */

    Material *_material;
    uint32_t _numMaterials; /* Each face can have its own material. */

    Material *_textMaterial;

    /* CPU copies for debug picking/introspection. */
    std::vector<prc_api_vertex> _cpuVertices;
    std::vector<unsigned int> _cpuIndices;

    Vector4 _bbox_min;
    Vector4 _bbox_max;

    uint32_t _numAttributes;
    std::string _attributesTitle;
    std::vector<AttributeBase> _attributeBases;
    std::vector<AttributeRow> _attributes;

    bool _hasTransparency;  // Flag indicating if this product has transparent geometry

    void uploadGPU(uint32_t num_vertices, prc_api_vertex *vertices_in, std::vector<unsigned int> &indices_in);

};

#endif // _PRODUCT_H_
