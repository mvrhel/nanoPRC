#include "product.h"
#include <algorithm> // For std::sort
#include <cassert>
#include <vector>
#include <unordered_map>
#include <cstddef> /* For offsetof macro */
#include <cstdint>
#include <inttypes.h>
#include <ctime>
#include <glad/glad.h>
#include "mesh.h"
#include "roboto_font.h"
#include "graphics2d.h"
#include <string.h>
#include <cstdio>

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

/* For debug of font textures */
//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"


static std::unordered_map<unsigned char *, Texture *> textureMap;

/* Optional limits for CPU-side pick/introspection buffers.
    Set any value to 0 to disable that specific limit. */
#ifndef NANOPRC_PICK_CPU_MAX_VERTICES_PER_PRODUCT
#define NANOPRC_PICK_CPU_MAX_VERTICES_PER_PRODUCT 0
#endif

#ifndef NANOPRC_PICK_CPU_MAX_INDICES_PER_PRODUCT
#define NANOPRC_PICK_CPU_MAX_INDICES_PER_PRODUCT 0
#endif

#ifndef NANOPRC_PICK_CPU_MAX_TOTAL_BYTES
#define NANOPRC_PICK_CPU_MAX_TOTAL_BYTES 0
#endif

static uint64_t g_pick_cpu_bytes_stored = 0;
static uint32_t g_pick_cpu_skipped_products = 0;
static bool g_pick_cpu_limit_hit = false;

/* Transform debug toggles for Product::update()
   Set NANOPRC_DEBUG_UPDATE_TRANSFORM to 1 to enable logging.
   Set NANOPRC_DEBUG_UPDATE_TRANSFORM_NAME to a substring filter (or "" for all). */
#ifndef NANOPRC_DEBUG_UPDATE_TRANSFORM
#define NANOPRC_DEBUG_UPDATE_TRANSFORM 0
#endif

#ifndef NANOPRC_DEBUG_UPDATE_TRANSFORM_NAME
#define NANOPRC_DEBUG_UPDATE_TRANSFORM_NAME ""
#endif

static bool product_debug_transform_enabled()
{
    return NANOPRC_DEBUG_UPDATE_TRANSFORM != 0;
}

static bool product_debug_transform_matches_name(const char *name)
{
    const char *filter = NANOPRC_DEBUG_UPDATE_TRANSFORM_NAME;
    if (filter == nullptr || filter[0] == '\0')
    {
        return true;
    }
    if (name == nullptr)
    {
        return false;
    }

    /* Substring match keeps filtering flexible for long hierarchical names. */
    return strstr(name, filter) != nullptr;
}

static void product_debug_print_local_to_world(const char *name, const Matrix4 &mat)
{
    float m[16] = { 0.0f };
    size_t copy_size = sizeof(m) < sizeof(Matrix4) ? sizeof(m) : sizeof(Matrix4);
    memcpy(m, &mat, copy_size);

    printf("[Product::update] name=%s local_to_world=",
        name != nullptr ? name : "(null)");
    printf("  [(%.2f %.2f %.2f %.2f)", m[0], m[1], m[2], m[3]);
    printf("(%.2f %.2f %.2f %.2f)", m[4], m[5], m[6], m[7]);
    printf("(%.2f %.2f %.2f %.2f)", m[8], m[9], m[10], m[11]);
    printf("(%.2f %.2f %.2f %.2f)\n", m[12], m[13], m[14], m[15]);
}

static Texture *loadTexture(prc_api_texture *texture)
{
    auto it = textureMap.find(texture->data);
    if (it != textureMap.end())
        return it->second;
    Texture *tex = new Texture;
    tex->load(*texture);
    textureMap[texture->data] = tex;
    tex->retain();
    return tex;
}

void releaseTextures()
{
    for (auto &p : textureMap)
        p.second->release();
    textureMap.clear();
}

void Product::render(MeshShader *shader) const
{
    if (_vao)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBlendEquation(GL_FUNC_ADD);

        glBindVertexArray(_vao);
        shader->setModel(_local_to_world);

        // Check if this is text markup (needs special depth handling)
        bool isTextMarkup = _numMeshes > 0 && _meshes[0].isMarkup && _hasTransparency;

        // PASS 1: Render all opaque meshes (only if this Product is NOT transparent)
        if (!_hasTransparency)
        {
            glDepthMask(GL_TRUE);  // Write to depth buffer
            glEnable(GL_DEPTH_TEST);

            for (uint32_t i = 0; i < _numMeshes; i++)
            {
                const MeshSpec &mesh = _meshes[i];
                shader->setMaterial(_material[mesh.faceIndex]);

                bool isLine = mesh.primitive == GL_LINES || mesh.primitive == GL_LINE_STRIP || mesh.primitive == GL_LINE_LOOP;
                bool isMarkUp = mesh.isMarkup;
                shader->setIsLine(isLine || isMarkUp);
                shader->setVertexHasMaterial(mesh.verticesHaveStyle);

                glDrawElements(
                    mesh.primitive,
                    mesh.numIndices,
                    GL_UNSIGNED_INT,
                    (void *)(_meshes[i].offset * sizeof(GLuint)));
            }
        }

        // PASS 2: Render all transparent meshes (only if this Product IS transparent)
// PASS 2: Render all transparent meshes (only if this Product IS transparent)
        if (_hasTransparency)
        {
            // For text, we want depth testing ON but depth writing OFF
            // This allows text to respect geometry depth but not block other transparent objects
            glDepthMask(GL_FALSE);
            glEnable(GL_DEPTH_TEST);

            // Use LEQUAL for text to handle coincident geometry better
            if (isTextMarkup)
            {
                glDepthFunc(GL_LEQUAL);
            }

            Vector3 cameraPos = shader->getViewPos();

            struct TransparentMesh
            {
                uint32_t index;
                float distanceToCamera;
            };
            std::vector<TransparentMesh> transparentMeshes;
            transparentMeshes.reserve(_numMeshes);

            for (uint32_t i = 0; i < _numMeshes; i++)
            {
                Vector4 meshPosWorld = _local_to_world * Vector4(0.0f, 0.0f, 0.0f, 1.0f);
                Vector3 meshPos3(meshPosWorld.x, meshPosWorld.y, meshPosWorld.z);
                float distance = (meshPos3 - cameraPos).length();
                transparentMeshes.push_back({ i, distance });
            }

            std::sort(transparentMeshes.begin(), transparentMeshes.end(),
                [](const TransparentMesh &a, const TransparentMesh &b) {
                    return a.distanceToCamera > b.distanceToCamera;
                });

            for (const auto &tm : transparentMeshes)
            {
                const MeshSpec &mesh = _meshes[tm.index];
                shader->setMaterial(_material[mesh.faceIndex]);

                bool isLine = mesh.primitive == GL_LINES || mesh.primitive == GL_LINE_STRIP || mesh.primitive == GL_LINE_LOOP;
                bool isMarkUp = mesh.isMarkup;
                shader->setIsLine(isLine || isMarkUp);
                shader->setVertexHasMaterial(mesh.verticesHaveStyle);

                glDrawElements(mesh.primitive, mesh.numIndices, GL_UNSIGNED_INT,
                    (void *)(_meshes[tm.index].offset * sizeof(GLuint)));
            }

            if (isTextMarkup)
            {
                //glDepthFunc(GL_LESS);
            }
            glDepthMask(GL_TRUE);
        }
    }

    if (_renderCompanion && _renderCompanion->_enabled)
    {
        _renderCompanion->render(shader);
    }

    // Recursively render children
    for (uint32_t i = 0; i < _nchildren; i++)
    {
        if (_children[i]->_enabled)
            _children[i]->render(shader);
    }
}

void Product::update()
{
    if (!_dirty)
        return;

    _dirty = false;

    if (_parent)
    {
        if (_parent->_dirty)
            _parent->update();
        _local_to_world = _parent->_local_to_world * _model;

#if NANOPRC_DEBUG_UPDATE_TRANSFORM
        if (product_debug_transform_enabled() &&
            product_debug_transform_matches_name(_name))
        {
            product_debug_print_local_to_world(_name, _local_to_world);
        }
#endif

    }
    else
        _local_to_world = _model;

    for (uint32_t i = 0; i < _nchildren; i++)
    {
        Product *child = _children[i];
        // if (child->_dirty)
        child->_dirty = true;
        child->update();
    }

    if (_renderCompanion)
    {
        _renderCompanion->_dirty = true;
        _renderCompanion->update();
    }
}

void Product::invalidate()
{
    _dirty = false;
    if (_parent && !_parent->_dirty)
        _parent->invalidate();
}

void Product::createChildren(uint32_t num_children)
{
    _children = (Product **)malloc(sizeof(Product *) * num_children);
    if (_children == NULL)
    {
        printf("Product::createChildren: malloc failed\n");
        exit(1);
    }
    _nchildren = num_children;
}

Product *Product::getChildFromHeap(uint32_t child_index, Product *product_heap, uint32_t *array_index)
{
    uint32_t index = *array_index;

    children()[child_index] = &product_heap[index];

    return &product_heap[index];
}

void Product::uploadGPU(uint32_t num_vertices, prc_api_vertex *vertices_in,
    std::vector<unsigned int> &indices_in)
{
    const uint64_t vertex_bytes = (uint64_t)num_vertices * (uint64_t)sizeof(prc_api_vertex);
    const uint64_t index_bytes = (uint64_t)indices_in.size() * (uint64_t)sizeof(unsigned int);
    const uint64_t upload_bytes = vertex_bytes + index_bytes;
    bool keep_cpu_pick_data = true;

#if NANOPRC_PICK_CPU_MAX_VERTICES_PER_PRODUCT > 0
    if ((uint64_t)num_vertices > (uint64_t)NANOPRC_PICK_CPU_MAX_VERTICES_PER_PRODUCT)
        keep_cpu_pick_data = false;
#endif

#if NANOPRC_PICK_CPU_MAX_INDICES_PER_PRODUCT > 0
    if ((uint64_t)indices_in.size() > (uint64_t)NANOPRC_PICK_CPU_MAX_INDICES_PER_PRODUCT)
        keep_cpu_pick_data = false;
#endif

#if NANOPRC_PICK_CPU_MAX_TOTAL_BYTES > 0
    if (g_pick_cpu_bytes_stored + upload_bytes > (uint64_t)NANOPRC_PICK_CPU_MAX_TOTAL_BYTES)
        keep_cpu_pick_data = false;
#endif

    if (keep_cpu_pick_data)
    {
        _cpuVertices.assign(vertices_in, vertices_in + num_vertices);
        _cpuIndices = indices_in;
        g_pick_cpu_bytes_stored += upload_bytes;
    }
    else
    {
        _cpuVertices.clear();
        _cpuIndices.clear();
        g_pick_cpu_skipped_products++;

        if (!g_pick_cpu_limit_hit)
        {
            g_pick_cpu_limit_hit = true;
            printf("[TrianglePick] CPU pick storage disabled by tessellation limits.\n");
            printf("[TrianglePick] Limits: maxVertsPerProduct=%u maxIndicesPerProduct=%u maxTotalBytes=%" PRIu64 "\n",
                (uint32_t)NANOPRC_PICK_CPU_MAX_VERTICES_PER_PRODUCT,
                (uint32_t)NANOPRC_PICK_CPU_MAX_INDICES_PER_PRODUCT,
                (uint64_t)NANOPRC_PICK_CPU_MAX_TOTAL_BYTES);
            printf("[TrianglePick] Current upload: verts=%u indices=%zu bytes=%" PRIu64 "\n",
                num_vertices, indices_in.size(), upload_bytes);
        }
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glBlendEquation(GL_FUNC_ADD);

    /* Upload to GPU */
    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_vbo);
    glGenBuffers(1, &_ebo);

    glBindVertexArray(_vao);

    /* vertex buffer */
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER, num_vertices * sizeof(prc_api_vertex), vertices_in, GL_STATIC_DRAW);

    /* index buffer */
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_in.size() * sizeof(unsigned int), indices_in.data(), GL_STATIC_DRAW);

    /* vertex attributes */
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, position));
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, normal));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, color));
    glEnableVertexAttribArray(2);

    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, uv));
    glEnableVertexAttribArray(3);

    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, diffuse));
    glEnableVertexAttribArray(4);

    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, tint));
    glEnableVertexAttribArray(5);

    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, specular));
    glEnableVertexAttribArray(6);

    glVertexAttribPointer(7, 3, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, emissive));
    glEnableVertexAttribArray(7);

    glVertexAttribPointer(8, 1, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, shininess));
    glEnableVertexAttribArray(8);

    glVertexAttribPointer(9, 1, GL_FLOAT, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, alpha));
    glEnableVertexAttribArray(9);

    //glVertexAttribPointer(9, 1, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(prc_api_vertex), (void *)offsetof(prc_api_vertex, tri_has_material));
    //glEnableVertexAttribArray(9);

    glBindVertexArray(0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

bool Product::cpuPickStorageLimited()
{
    return g_pick_cpu_limit_hit;
}

uint64_t Product::cpuPickStoredBytes()
{
    return g_pick_cpu_bytes_stored;
}

uint32_t Product::cpuPickSkippedProductCount()
{
    return g_pick_cpu_skipped_products;
}

/* A special version of attach where we handle the text portion. Mixing up
   the non-text markups with the text was causing lots of issues and resulting
   in confusing code */
   /* A special version of attach where we handle the text portion. */
void Product::attachTextContent(prc_context *ctx, prc_api_data data,
    const prc_api_tess *tess, Graphics2D &textRenderer)
{
    if (tess->type != PRC_API_TESS_MarkUp || tess->num_text_primitives < 1)
    {
        printf("attachTextContent: Skipping - type=%d, num_text=%zu\n", tess->type, tess->num_text_primitives);
        return;
    }

    printf("attachTextContent: Processing %zu text primitives\n", tess->num_text_primitives);

    _hasTransparency = true;

    _meshes = new MeshSpec[1];
    _numMeshes = 1;
    _numMaterials = 1;

    if (_material == nullptr)
    {
        _material = new Material[1];
        _material[0] = kMaterial;
    }

    static Texture *sharedAtlasTexture = nullptr;
    static Font *sharedFont = nullptr;

    textRenderer.Begin(1024, 1024);

    if (sharedFont == nullptr)
    {
        prc_api_texture text_texture;
        sharedFont = Font::CreateFromData("roboto_reg", (unsigned char *)roboto_reg, 0, 64, text_texture);
        if (sharedFont == NULL)
        {
            printf("Model::load: Font::CreateFromData failed\n");
            exit(1);
        }

        sharedAtlasTexture = new Texture();
        sharedAtlasTexture->load(text_texture, true);
        sharedFont->SetTextureId(sharedAtlasTexture->id());

        printf("Font atlas created: texture_id=%u, channels=%d\n",
            sharedAtlasTexture->id(), text_texture.num_channels);

        if (text_texture.data)
        {
            free(text_texture.data);
            text_texture.data = nullptr;
        }
    }

    textRenderer.SetFont(sharedFont);
    _material[0].setDiffuseTexture(sharedAtlasTexture);

    for (uint32_t k = 0; k < tess->num_text_primitives; k++)
    {
        prc_api_text_primitive text_primitive;
        int rc = prc_api_get_text_primitive(ctx, data, (prc_api_tess *)tess, k, &text_primitive);
        if (rc < 0)
        {
            printf("Model::load: prc_api_get_text_primitive failed\n");
            exit(1);
        }

        printf("  Text[%u]: '%s' at (%.2f, %.2f, %.2f) color=(%.2f, %.2f, %.2f)\n",
            k, text_primitive.text,
            text_primitive.origin[0], text_primitive.origin[1], text_primitive.origin[2],
            text_primitive.color[0], text_primitive.color[1], text_primitive.color[2]);

        textRenderer.SetColor(Vector4(text_primitive.color[0], text_primitive.color[1],
            text_primitive.color[2], 1.0f));

        textRenderer.DrawText(text_primitive.text,
            (float)text_primitive.origin[0],
            (float)text_primitive.origin[1],
            (float)text_primitive.origin[2]);
    }

    size_t numTextVertices = textRenderer.GetNumVertices();
    size_t numTextIndices = textRenderer.GetNumIndices();

    printf("Text mesh: %zu vertices, %zu indices\n", numTextVertices, numTextIndices);

    if (numTextVertices == 0)
    {
        printf("WARNING: No text vertices generated!\n");
        delete[] _meshes;
        _meshes = nullptr;
        _numMeshes = 0;
        delete[] _material;
        _material = nullptr;
        _numMaterials = 0;
        return;
    }

    MeshSpec &mesh = _meshes[0];
    mesh.faceIndex = 0;
    mesh.primitive = GL_TRIANGLES;
    mesh.offset = 0;
    mesh.numIndices = (uint32_t)numTextIndices;
    mesh.isMarkup = true;
    mesh.verticesHaveStyle = false;

    prc_api_vertex *textVertices = new prc_api_vertex[numTextVertices];
    textRenderer.GetVertices(textVertices, _material[0]);

    // Diagnostic: Check first vertex
    printf("First text vertex: pos=(%.2f, %.2f, %.2f) uv=(%.4f, %.4f) color=(%.2f, %.2f, %.2f, %.2f)\n",
        textVertices[0].position[0], textVertices[0].position[1], textVertices[0].position[2],
        textVertices[0].uv[0], textVertices[0].uv[1],
        textVertices[0].color[0], textVertices[0].color[1],
        textVertices[0].color[2], textVertices[0].color[3]);

    std::vector<unsigned int> text_indices;
    textRenderer.GetIndicesVector(text_indices);

    uploadGPU((uint32_t)numTextVertices, textVertices, text_indices);

    printf("Text mesh uploaded to GPU: VAO=%u\n", _vao);

    delete[] textVertices;
}

void Product::attach(prc_context *ctx, prc_api_data data, const prc_api_tess *tess, Graphics2D &textRenderer)
{
    size_t numGraphicObjects;
    size_t face_max;
    int is_material;
    int code;

    // Store transparency flag from tessellation
    _hasTransparency = tess->has_transparency;

    numGraphicObjects = prc_api_get_num_graphics_primitives(ctx, data, tess);
    _numMeshes = numGraphicObjects;
    _meshes = new MeshSpec[_numMeshes];

    /* Check if this product has multiple faces with multiple materials */
    _numMaterials = prc_api_number_of_materials(ctx, data, tess);
    if (_numMaterials > 1)
    {
        _material = new Material[_numMaterials];
        for (uint32_t i = 0; i < _numMaterials; i++)
        {
            _material[i] = kMaterial;
        }
    }
    else
    {
        _material = new Material[1];
        _material[0] = kMaterial;
    }

    /* We may need to go back and reconfigure the face/tessellation relationship
       as each face could in theory have different material but right now
       we set the whole part (tessellation) to have a material. Probably OK but
       there could be a case where we have different faces with textures and
       materials and solid colors.  Fun! */
    if (tess->type == PRC_API_TESS_3D_Wire || tess->type == PRC_API_TESS_MarkUp)
    {
        face_max = 1;
    }
    else
    {
        face_max = tess->num_faces;
    }

    uint32_t counter = 0;
    size_t num_text_primitives = 0;
    prc_api_material prc_material;

    /* A change here to simplify things when dealing with multiple faces
       in the compressed code.  The vertex splitting was getting
       overly complex. In the noncompressed case, we
       can have multiple faces and those faces each have their own
       vertices. We will squash them back together here though */
    std::vector<unsigned int> indices;
    std::vector<prc_api_vertex> combined_vertices;
    uint32_t vertex_offset = 0;

    for (uint32_t i = 0; i < face_max; i++)
    {
        size_t num_graphic_primitives = 0;
        prc_api_texture texture;
        bool hasTexture = false;
        bool verticesHaveMaterial = false;

        if (prc_api_skip_face(ctx, tess, i))
        {
            continue;
        }

        /* Get the material for this face */
        is_material = prc_api_face_is_material(ctx, tess, i);
        if (is_material < 0)
        {
            printf("Model::load: prc_api_face_is_material failed\n");
            exit(1);
        }
        if (is_material)
        {
            prc_api_get_face_material(ctx, tess, &prc_material, i);
            _material[i].diffuse = Vector3(prc_material.diffuse[0], prc_material.diffuse[1], prc_material.diffuse[2]);
            _material[i].tint = Vector3(prc_material.ambient[0], prc_material.ambient[1], prc_material.ambient[2]);
            _material[i].specular = Vector3(prc_material.specular[0], prc_material.specular[1], prc_material.specular[2]);
            _material[i].emissive = Vector3(prc_material.emissive[0], prc_material.emissive[1], prc_material.emissive[2]);
            _material[i].shininess = prc_material.shininess;
            _material[i].alpha = prc_material.diffuse_alpha; /* We really need to use all the alpha values... */
        }

        /* The vertices could have material definitions assigned to them */
        verticesHaveMaterial = prc_api_vertices_have_material(ctx, tess, i);

        if (tess->type == PRC_API_TESS_3D_Wire || tess->type == PRC_API_TESS_MarkUp)
        {
            num_graphic_primitives = tess->num_line_primitives;
            if (tess->type == PRC_API_TESS_MarkUp)
            {
                num_text_primitives = tess->num_text_primitives;
            }
        }
        else
        {
            num_graphic_primitives = tess->tess_faces[i].num_graphic_primitives;
            texture = tess->tess_faces[i].texture;
            hasTexture = texture.data != nullptr;
        }

        if (hasTexture)
        {
            if (!_material[i].diffuseTexture)
                _material[i].setDiffuseTexture(new Texture);

            /* Load the image texture here */
            _material[i].diffuseTexture->load(texture);
        }

        prc_api_vertex *face_vertices = NULL;
        uint32_t face_vertex_count;
        code = prc_api_get_face_vertices(ctx, tess, i, &face_vertex_count, &face_vertices);
        if (code < 0)
        {
            printf("Model::load: prc_api_get_face_vertices failed\n");
            exit(1);
        }

        // Append THIS face's vertices to combined buffer
        for (size_t v = 0; v < face_vertex_count; v++) {
            combined_vertices.push_back(face_vertices[v]);
        }

        for (uint32_t j = 0; j < num_graphic_primitives; j++)
        {
            /* Get primitive from PRC data */
            prc_api_graphic_primitive primitive;
            int rc = prc_api_get_graphics_primitive( ctx, data,
                (prc_api_tess *)tess, i, j, &primitive);
            if (rc < 0)
            {
                printf("Model::load: prc_api_get_graphics_primitive failed\n");
                exit(1);
            }

            /* Create a mesh */
            MeshSpec &mesh = _meshes[counter];
            mesh.faceIndex = i;
            switch (primitive.type)
            {
            default:
                printf("Model::load: unknown primitive type\n");
                exit(1);
            case PRC_API_TRIANGLES:
                mesh.primitive = GL_TRIANGLES;
                break;
            case PRC_API_FAN:
                mesh.primitive = GL_TRIANGLE_FAN;
                break;
            case PRC_API_STRIP:
                mesh.primitive = GL_TRIANGLE_STRIP;
                break;
            case PRC_API_LINE:
                mesh.primitive = GL_LINES;
                break;
            case PRC_API_LINE_STRIP:
                mesh.primitive = GL_LINE_STRIP;
                break;
            case PRC_API_LINE_LOOP:
                mesh.primitive = GL_LINE_LOOP;
                break;
            }

            /* Offset the primitive.indices by the vertex_offset of the face */
            if (vertex_offset > 0)
            {
                for (size_t idx = 0; idx < primitive.num_indices; idx++)
                {
                    primitive.indices[idx] += vertex_offset;
                }
            }

            mesh.offset = indices.size();
            mesh.numIndices = primitive.num_indices;

            /* Copy index data */
            indices.resize(indices.size() + primitive.num_indices);
            memcpy(&indices[mesh.offset], primitive.indices, primitive.num_indices * sizeof(unsigned int));

            if (tess->type == PRC_API_TESS_MarkUp)
            {
                mesh.isMarkup = true;
            }
            else
            {
                mesh.isMarkup = false;
            }

            mesh.verticesHaveStyle = verticesHaveMaterial;
            counter++;
        } // End graphics primitives

        /* For the indices offset in the multi-face case */
        vertex_offset += face_vertex_count;
    } // End faces (and materials)

    if (combined_vertices.size() > 0 && indices.size() > 0)
        uploadGPU(combined_vertices.size(), combined_vertices.data(), indices);
}

static std::string formatAttributeValue(const prc_api_attribute_entry &entry)
{
    char buffer[64];

    auto formatEpochUtc = [](uint64_t epochSeconds, char *out, size_t outSize) -> bool
    {
        time_t seconds = static_cast<time_t>(epochSeconds);
        struct tm tm_utc;

#if defined(_WIN32)
        if (gmtime_s(&tm_utc, &seconds) != 0)
            return false;
#else
        if (gmtime_r(&seconds, &tm_utc) == NULL)
            return false;
#endif

        return strftime(out, outSize, "%Y-%m-%d %H:%M:%S UTC", &tm_utc) > 0;
    };

    switch (entry.type)
    {
    case PRC_API_INTEGER_ATTRIBUTE:
        snprintf(buffer, sizeof(buffer), "%d", entry.value_integer);
        return buffer;
    case PRC_API_DOUBLE_ATTRIBUTE:
        snprintf(buffer, sizeof(buffer), "%.17g", entry.value_double);
        return buffer;
    case PRC_API_VALUE_SECS_INTEGER_ATTRIBUTE:
    {
        char time_buffer[64];
        if (formatEpochUtc((uint64_t)entry.value_secs_integer, time_buffer, sizeof(time_buffer)))
        {
            return std::string(time_buffer) + " (" + std::to_string(entry.value_secs_integer) + ")";
        }

        snprintf(buffer, sizeof(buffer), "%" PRIu32, entry.value_secs_integer);
        return buffer;
    }
    case PRC_API_STRING_ATTRIBUTE:
        return entry.value_string ? entry.value_string : "";
    case PRC_API_VALUE_TIME_ATTRIBUTE:
    {
        char time_buffer[64];
        if (formatEpochUtc(entry.value_time, time_buffer, sizeof(time_buffer)))
        {
            return std::string(time_buffer) + " (" + std::to_string(entry.value_time) + ")";
        }

        snprintf(buffer, sizeof(buffer), "%" PRIu64, entry.value_time);
        return buffer;
    }
    default:
        return "<unsupported>";
    }
}

static std::string composeAttributeDisplayTitle(const std::string &baseTitle,
    const std::string &entryTitle, size_t entryIndex, size_t numEntries)
{
    if (numEntries <= 1)
    {
        if (!baseTitle.empty())
            return baseTitle;
        if (!entryTitle.empty())
            return entryTitle;
        return "<attribute>";
    }

    if (!baseTitle.empty() && !entryTitle.empty())
        return baseTitle + " / " + entryTitle;

    if (!baseTitle.empty())
    {
        char indexBuffer[32];
        snprintf(indexBuffer, sizeof(indexBuffer), "%s / #%zu", baseTitle.c_str(), entryIndex + 1);
        return indexBuffer;
    }

    if (!entryTitle.empty())
        return entryTitle;

    char fallback[32];
    snprintf(fallback, sizeof(fallback), "#%zu", entryIndex + 1);
    return fallback;
}

void Product::setAttributes(prc_api_attributes *attributes)
{
    if (attributes == nullptr)
    {
        _numAttributes = 0;
        _attributesTitle.clear();
        _attributeBases.clear();
        _attributes.clear();
        return;
    }

    _attributesTitle.clear();
    _attributeBases.clear();
    _attributeBases.reserve(attributes->num_base_attributes);

    _attributes.clear();

    for (size_t baseIndex = 0; baseIndex < attributes->num_base_attributes; ++baseIndex)
    {
        const prc_api_attribute_base &baseAttribute = attributes->base_attributes[baseIndex];

        Product::AttributeBase displayBase;
        displayBase.title = baseAttribute.attribute_base_title ?
            baseAttribute.attribute_base_title : "";
        displayBase.entries.reserve(baseAttribute.num_attributes);

        for (size_t entryIndex = 0; entryIndex < baseAttribute.num_attributes; ++entryIndex)
        {
            const prc_api_attribute_entry &entry = baseAttribute.attributes[entryIndex];
            const std::string entryTitle = entry.entry_title ? entry.entry_title : "";

            Product::AttributeRow row;
            row.title = composeAttributeDisplayTitle(displayBase.title, entryTitle,
                entryIndex, baseAttribute.num_attributes);
            row.value = formatAttributeValue(entry);

            displayBase.entries.push_back(row);
            _attributes.push_back(std::move(row));
        }

        _attributeBases.push_back(std::move(displayBase));
    }

    _numAttributes = static_cast<uint32_t>(_attributes.size());
}

Product::Product() :
    _enabled(true),
    _name(nullptr),
    _parent(nullptr),
    _isPartNode(false),
    _partNameSameAsProduct(false),
    _children(nullptr), _nchildren(0),
    _renderCompanion(nullptr),
    _dirty(true),
    _vao(0), _vbo(0), _ebo(0),
    _meshes(nullptr), _numMeshes(0),
    _material(nullptr), _numMaterials(0),
    _hasTransparency(false)
{
}

Product::~Product()
{
    /* Children freed with freeing of heap */
    if (_children)
        free(_children);

    if (_name)
        delete[] _name;

    if (_numMeshes)
        delete[] _meshes;

    if (_numMaterials)
        delete[] _material;

    if (_ebo)
        glDeleteBuffers(1, &_ebo);

    if (_vbo)
        glDeleteBuffers(1, &_vbo);

    if (_vao)
        glDeleteVertexArrays(1, &_vao);
}
