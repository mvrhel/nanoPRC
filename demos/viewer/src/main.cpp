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

#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <cinttypes>
#include <cstring>
#include <limits>
#include <cmath>

#include <SDL3/SDL.h>

#include <prc_api.h>

#include <imgui.h>
#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_opengl3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "scene.h"
#include "camera.h"
#include "skybox.h"
#include "product.h"
#include "util.h"
#include "shadow.h"
#include "composite.h"
#include "bloom.h"
#include "mesh.h"
#include "config.h"

static constexpr int kWindowWidth = 480;
static constexpr int kWindowHeight = 360;

static constexpr float kMoveSpeed = 4.0f;
static constexpr float kShiftMultiplier = 300.0f;

static constexpr float kTurnSpeed = 90.0f;
static constexpr float kMouseSensitivity = 15.0f;

static GLuint g_default_vao = 0;

static const DirLight kDirLight = {
    Vector3(-0.174078, -0.69311, 0.696311), // direction
    colorRGB(255, 255, 224),                // color
    1.0f                                    // intensity
};

static constexpr PointLight kPointLight = {
    Vector3(14.0f, 7.0f, 3.0f), // position
    Vector3(1.0f, 1.0f, 1.0f),  // color
    0.0f                        // intensity
};

struct AtmosphereProperties
{
    float sunAltitude;
    float sunAzimuth;

    Vector3 sunColor;
    float sunIntensity;

    Vector3 skyColorHorizon;
    Vector3 skyColorZenith;

    float sunTightness;
};

static AtmosphereProperties _atmosphere;

static constexpr float kDayLength = 60.0f;

static float _renderScale = 1.0f;
static Scene _scene;
static int _mouse_up_x, _mouse_up_y; /* Needed to maintain mouse pos for trackball */
static Camera _camera;
static Product *_selectedProduct = nullptr;

static void run(Config &config, SDL_Window *window, const char *file, bool headless = false, const char *output_file = NULL, bool memoryLeakCheck = false);
static bool debugMenu(float time, float deltaTime);

struct PickResult
{
    bool valid;
    Product *product;
    uint32_t meshIndex;
    uint32_t triangleIndexInMesh;
    uint32_t index0;
    uint32_t index1;
    uint32_t index2;
    Vector3 worldPos0;
    Vector3 worldPos1;
    Vector3 worldPos2;
    Vector3 worldN0;
    Vector3 worldN1;
    Vector3 worldN2;
    Vector3 hitPos;
    Vector3 faceNormal;
    float dotFaceAvgNormal;
};

static PickResult g_lastPick = { false };

static bool rayTriangleIntersect(const Vector3 &origin, const Vector3 &dir,
    const Vector3 &v0, const Vector3 &v1, const Vector3 &v2, float *tOut)
{
    const float kEps = 1e-7f;
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 p = cross(dir, edge2);
    float det = dot(edge1, p);

    if (fabsf(det) < kEps)
        return false;

    float invDet = 1.0f / det;
    Vector3 tvec = origin - v0;
    float u = dot(tvec, p) * invDet;
    if (u < 0.0f || u > 1.0f)
        return false;

    Vector3 q = cross(tvec, edge1);
    float v = dot(dir, q) * invDet;
    if (v < 0.0f || (u + v) > 1.0f)
        return false;

    float t = dot(edge2, q) * invDet;
    if (t <= kEps)
        return false;

    *tOut = t;
    return true;
}

static Vector3 transformPoint(const Matrix4 &m, const Vector3 &p)
{
    Vector4 v = m * Vector4(p.x, p.y, p.z, 1.0f);
    return Vector3(v.x, v.y, v.z);
}

static Vector3 transformDirection(const Matrix4 &m, const Vector3 &n)
{
    Vector4 v = m * Vector4(n.x, n.y, n.z, 0.0f);
    Vector3 out(v.x, v.y, v.z);
    float len2 = dot(out, out);
    if (len2 > 1e-12f)
        out = normalize(out);
    return out;
}

static bool computePickRay(int mouseX, int mouseY, int viewportW, int viewportH,
    Vector3 *rayOrigin, Vector3 *rayDir)
{
    if (viewportW <= 0 || viewportH <= 0)
        return false;

    float ndcX = (2.0f * (float)mouseX) / (float)viewportW - 1.0f;
    float ndcY = 1.0f - (2.0f * (float)mouseY) / (float)viewportH;

    Matrix4 invVP = inverse(_camera.projection() * _camera.view());
    Vector4 nearClip(ndcX, ndcY, -1.0f, 1.0f);
    Vector4 farClip(ndcX, ndcY, 1.0f, 1.0f);
    Vector4 nearWorld4 = invVP * nearClip;
    Vector4 farWorld4 = invVP * farClip;

    if (fabsf(nearWorld4.w) < 1e-8f || fabsf(farWorld4.w) < 1e-8f)
        return false;

    Vector3 nearWorld(nearWorld4.x / nearWorld4.w,
        nearWorld4.y / nearWorld4.w,
        nearWorld4.z / nearWorld4.w);
    Vector3 farWorld(farWorld4.x / farWorld4.w,
        farWorld4.y / farWorld4.w,
        farWorld4.z / farWorld4.w);

    Vector3 dir = farWorld - nearWorld;
    float len2 = dot(dir, dir);
    if (len2 <= 1e-12f)
        return false;

    *rayOrigin = _camera.position();
    *rayDir = normalize(dir);
    return true;
}

static bool pickTriangleAtScreenPoint(int mouseX, int mouseY, int viewportW,
    int viewportH, PickResult *out)
{
    if (Product::cpuPickStorageLimited())
        return false;

    Vector3 rayOrigin, rayDir;
    uint32_t p;
    float bestT = std::numeric_limits<float>::infinity();
    PickResult best = { false };

    if (!computePickRay(mouseX, mouseY, viewportW, viewportH, &rayOrigin, &rayDir))
        return false;

    Product *products = _scene.models();
    uint32_t numProducts = _scene.mumProducts();

    for (p = 0; p < numProducts; p++)
    {
        Product *prod = &products[p];
        const std::vector<prc_api_vertex> &verts = prod->cpuVertices();
        const std::vector<unsigned int> &indices = prod->cpuIndices();
        const MeshSpec *meshes = prod->meshes();
        uint32_t meshCount = prod->numMeshes();
        Matrix4 model = prod->localToWorld();

        if (!prod->enabled() || verts.empty() || indices.empty() || meshes == nullptr)
            continue;

        for (uint32_t m = 0; m < meshCount; m++)
        {
            const MeshSpec &mesh = meshes[m];
            uint32_t start = mesh.offset;
            uint32_t count = mesh.numIndices;

            if (start >= indices.size())
                continue;

            if ((start + count) > indices.size())
                count = (uint32_t)indices.size() - start;

            auto testTriangle = [&](uint32_t ia, uint32_t ib, uint32_t ic,
                                    uint32_t triIndexInMesh) {
                if (ia >= verts.size() || ib >= verts.size() || ic >= verts.size())
                    return;

                Vector3 p0 = transformPoint(model, Vector3(
                    verts[ia].position[0], verts[ia].position[1], verts[ia].position[2]));
                Vector3 p1 = transformPoint(model, Vector3(
                    verts[ib].position[0], verts[ib].position[1], verts[ib].position[2]));
                Vector3 p2 = transformPoint(model, Vector3(
                    verts[ic].position[0], verts[ic].position[1], verts[ic].position[2]));

                float t;
                if (!rayTriangleIntersect(rayOrigin, rayDir, p0, p1, p2, &t))
                    return;

                if (t >= bestT)
                    return;

                Vector3 wn0 = transformDirection(model, Vector3(
                    verts[ia].normal[0], verts[ia].normal[1], verts[ia].normal[2]));
                Vector3 wn1 = transformDirection(model, Vector3(
                    verts[ib].normal[0], verts[ib].normal[1], verts[ib].normal[2]));
                Vector3 wn2 = transformDirection(model, Vector3(
                    verts[ic].normal[0], verts[ic].normal[1], verts[ic].normal[2]));

                Vector3 faceN = normalize(cross(p1 - p0, p2 - p0));
                Vector3 avgN = normalize(wn0 + wn1 + wn2);

                bestT = t;
                best.valid = true;
                best.product = prod;
                best.meshIndex = m;
                best.triangleIndexInMesh = triIndexInMesh;
                best.index0 = ia;
                best.index1 = ib;
                best.index2 = ic;
                best.worldPos0 = p0;
                best.worldPos1 = p1;
                best.worldPos2 = p2;
                best.worldN0 = wn0;
                best.worldN1 = wn1;
                best.worldN2 = wn2;
                best.hitPos = rayOrigin + rayDir * t;
                best.faceNormal = faceN;
                best.dotFaceAvgNormal = dot(faceN, avgN);
            };

            if (mesh.primitive == GL_TRIANGLES)
            {
                uint32_t tri = 0;
                for (uint32_t i = 0; i + 2 < count; i += 3, tri++)
                    testTriangle(indices[start + i], indices[start + i + 1],
                        indices[start + i + 2], tri);
            }
            else if (mesh.primitive == GL_TRIANGLE_STRIP)
            {
                uint32_t tri = 0;
                for (uint32_t i = 0; i + 2 < count; i++, tri++)
                {
                    uint32_t a = indices[start + i];
                    uint32_t b = indices[start + i + 1];
                    uint32_t c = indices[start + i + 2];
                    if (i & 1u)
                    {
                        uint32_t tmp = a;
                        a = b;
                        b = tmp;
                    }
                    testTriangle(a, b, c, tri);
                }
            }
            else if (mesh.primitive == GL_TRIANGLE_FAN)
            {
                uint32_t tri = 0;
                for (uint32_t i = 1; i + 1 < count; i++, tri++)
                    testTriangle(indices[start], indices[start + i],
                        indices[start + i + 1], tri);
            }
        }
    }

    *out = best;
    return best.valid;
}

static void printPickResult(const PickResult &pick)
{
    if (!pick.valid)
        return;

    printf("\n[TrianglePick] product='%s' mesh=%u tri=%u indices=(%u,%u,%u)\n",
        pick.product != nullptr && pick.product->name() != nullptr ? pick.product->name() : "(unnamed)",
        pick.meshIndex, pick.triangleIndexInMesh,
        pick.index0, pick.index1, pick.index2);

    printf("  hit=(%.9f, %.9f, %.9f)\n",
        pick.hitPos.x, pick.hitPos.y, pick.hitPos.z);
    printf("  v0=(%.9f, %.9f, %.9f) n0=(%.9f, %.9f, %.9f)\n",
        pick.worldPos0.x, pick.worldPos0.y, pick.worldPos0.z,
        pick.worldN0.x, pick.worldN0.y, pick.worldN0.z);
    printf("  v1=(%.9f, %.9f, %.9f) n1=(%.9f, %.9f, %.9f)\n",
        pick.worldPos1.x, pick.worldPos1.y, pick.worldPos1.z,
        pick.worldN1.x, pick.worldN1.y, pick.worldN1.z);
    printf("  v2=(%.9f, %.9f, %.9f) n2=(%.9f, %.9f, %.9f)\n",
        pick.worldPos2.x, pick.worldPos2.y, pick.worldPos2.z,
        pick.worldN2.x, pick.worldN2.y, pick.worldN2.z);
    printf("  faceN=(%.9f, %.9f, %.9f) dot(face,avgVertexN)=%.9f\n",
        pick.faceNormal.x, pick.faceNormal.y, pick.faceNormal.z,
        pick.dotFaceAvgNormal);
}

static void drawTrianglePickOverlay()
{
    if (!g_lastPick.valid)
    {
        if (!Product::cpuPickStorageLimited())
            return;
    }

    ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings;

    ImGui::SetNextWindowBgAlpha(0.40f);
    if (ImGui::Begin("Triangle Pick", nullptr, flags))
    {
        if (Product::cpuPickStorageLimited())
        {
            ImGui::Text("Triangle pick disabled by tessellation size limits");
            ImGui::Text("Skipped products: %u", Product::cpuPickSkippedProductCount());
            ImGui::Text("CPU pick bytes kept: %.2f MB",
                (float)Product::cpuPickStoredBytes() / (1024.0f * 1024.0f));
            ImGui::Separator();
            ImGui::Text("Tune NANOPRC_PICK_CPU_* defines in product.cpp");
            ImGui::End();
            return;
        }

        ImGui::Text("Ctrl+LeftClick in viewport to pick triangle");
        ImGui::Separator();
        ImGui::Text("Product: %s",
            g_lastPick.product != nullptr && g_lastPick.product->name() != nullptr ?
            g_lastPick.product->name() : "(unnamed)");
        ImGui::Text("Mesh: %u  Tri: %u", g_lastPick.meshIndex,
            g_lastPick.triangleIndexInMesh);
        ImGui::Text("Indices: %u %u %u", g_lastPick.index0,
            g_lastPick.index1, g_lastPick.index2);
        ImGui::Text("dot(face, avgN): %.6f", g_lastPick.dotFaceAvgNormal);
    }
    ImGui::End();
}

static const char *normalDebugModeLabel(int mode)
{
    switch (mode)
    {
    default:
    case 0:
        return "Off";
    case 1:
        return "Normal RGB";
    case 2:
        return "Normal Length";
    case 3:
        return "Degenerate Mask";
    case 4:
        return "Shader Branch";
    case 5:
        return "Front/Back Facing";
    case 6:
        return "Raw Normal RGB";
    case 7:
        return "Geom vs Imported";
    }
}

static void drawNormalDebugOverlay()
{
    const int mode = _scene.normalDebugMode();

    if (mode == 0)
        return;

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav;

    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.35f);

    if (ImGui::Begin("Normal Debug Overlay", nullptr, flags))
    {
        ImGui::Text("Normals: %s (%d)", normalDebugModeLabel(mode), mode);
        ImGui::Text("Hotkeys: 0 Off | 1 RGB | 2 Len | 3 Mask | 4 Branch | 5 Face | 6 Raw | 7 Geom");
        ImGui::Text("Pick: Ctrl + LeftClick to dump triangle data");
    }
    ImGui::End();
}

static void initSDL()
{
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        printf("SDL_Init failed: %s\n", SDL_GetError());
        exit(1);
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#ifdef __APPLE__
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
#endif
}

static SDL_Window *createWindow(Config &config)
{
    SDL_Window *window = SDL_CreateWindow(
        "nanoprc",
        kWindowWidth, kWindowHeight,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN);
    if (!window)
    {
        printf("SDL_CreateWindow failed: %s\n", SDL_GetError());
        exit(1);
    }

    const SDL_DisplayMode *mode = SDL_GetDesktopDisplayMode(1);
    if (!mode)
        printf("SDL_GetCurrentDisplayMode failed: %s\n", SDL_GetError());
    else
    {
        int w = mode->w * 2 / 3;
        int h = mode->h * 2 / 3;

        SDL_SetWindowSize(window, w, h);
        SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    }

    SDL_ShowWindow(window);

    return window;
}

static SDL_GLContext createGLContext(Config &config, SDL_Window *window)
{
    SDL_GLContext gl = SDL_GL_CreateContext(window);
    if (!gl)
    {
        printf("SDL_GL_CreateContext failed: %s\n", SDL_GetError());
        exit(1);
    }

    if (!SDL_GL_MakeCurrent(window, gl))
    {
        printf("SDL_GL_MakeCurrent failed: %s\n", SDL_GetError());
        exit(1);
    }

    if (!gladLoadGLLoader((void *(*)(const char *))SDL_GL_GetProcAddress))
    {
        printf("gladLoadGLLoader failed\n");
        exit(1);
    }

    /* macOS core profile requires a non-zero VAO bound before draw calls. */
    glGenVertexArrays(1, &g_default_vao);
    glBindVertexArray(g_default_vao);

    return gl;
}

int main(int argc, char *argv[])
{
    const char *file = nullptr;
    bool batchMode = false;
    const char *outputFile = nullptr;
    bool memoryLeakMode = false;

    if (argc < 2)
    {
        printf("Usage: nano_prc_viewer <file> [--batch] [--output <filename.png>] [--MemoryLeak]\n");
        return 1;
    }

    /* Parse args: first non-option is file, --batch/--headless enable batch mode,
       --output specifies PNG output, --MemoryLeak enables leak-based exit code. */
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--batch") == 0 || strcmp(argv[i], "--headless") == 0)
        {
            batchMode = true;
        }
        else if (strcmp(argv[i], "--MemoryLeak") == 0 || strcmp(argv[i], "-MemoryLeak") == 0)
        {
            memoryLeakMode = true;
        }
        else if (strcmp(argv[i], "--output") == 0 || strcmp(argv[i], "-o") == 0)
        {
            if (i + 1 < argc)
            {
                outputFile = argv[++i];
            }
            else
            {
                printf("Error: --output requires a filename argument\n");
                return 1;
            }
        }
        else if (file == nullptr)
        {
            file = argv[i];
        }
    }

    if (!file)
    {
        printf("Usage: nano_prc_viewer <file> [--batch] [--output <filename.png>] [--MemoryLeak]\n");
        return 1;
    }

    Config config;
    if (!config.readIni("viewer.ini"))
        printf("Warning: no viewer.ini found\n");

    initSDL();

    SDL_Window *window = createWindow(config);
    SDL_GLContext gl = createGLContext(config, window);

    SDL_GL_SetSwapInterval(1); // Force vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL3_InitForOpenGL(window, gl);
    ImGui_ImplOpenGL3_Init();

    printf("OpenGL %s, GLSL %s\n",
        glGetString(GL_VERSION),
        glGetString(GL_SHADING_LANGUAGE_VERSION));

    run(config, window, file, batchMode, outputFile, memoryLeakMode);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    if (g_default_vao)
    {
        glDeleteVertexArrays(1, &g_default_vao);
        g_default_vao = 0;
    }

    SDL_GL_DestroyContext(gl);
    SDL_DestroyWindow(window);
    SDL_Quit();

    if (!config.writeIni("viewer.ini"))
        printf("Warning: failed to write viewer.ini\n");

    return 0;
}

extern void releaseTextures();

static float
arc_projection(float x, float y)
{
    float r = 1;
    float x2 = x * x;
    float y2 = y * y;
    float z;
    float z_arg;
    float dist2 = x2 + y2;
    float r2 = r * r;

    if (dist2 * 2 <= r2)
    {
        z_arg = r2 - dist2;
        if (z_arg < 0)
            z_arg = 0;
        z = sqrtf(z_arg);
    }
    else
    {
        z = (r2 / 2) / sqrtf(dist2);
    }
    return z;
}

static float
vec_length(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float
vec_dot(float x1, float y1, float z1, float x2, float y2, float z2)
{
    return x1 * x2 + y1 * y2 + z1 * z2;
}

static float
vec_angle(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dot = vec_dot(x1, y1, z1, x2, y2, z2);
    float len1 = vec_length(x1, y1, z1);
    float len2 = vec_length(x2, y2, z2);
    float arg = dot / (len1 * len2);

    if (arg > 1)
        arg = 1;
    else if (arg < -1)
        arg = -1;

    return acosf(arg);
}

static Matrix4
compute_rotation(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float angle = vec_angle(x1, y1, z1, x2, y2, z2);
    float axis_x = y1 * z2 - z1 * y2;
    float axis_y = z1 * x2 - x1 * z2;
    float axis_z = x1 * y2 - y1 * x2;
    float axis_len = vec_length(axis_x, axis_y, axis_z);
    if (axis_len > 0)
    {
        axis_x /= axis_len;
        axis_y /= axis_len;
        axis_z /= axis_len;
    }
    else
    {
        axis_x = 0;
        axis_y = 0;
        axis_z = 1;
    }
    float half_angle = angle * 0.5f;
    float sin_half_angle = sinf(half_angle);

    float q0 = cosf(half_angle);
    float q1 = axis_x * sin_half_angle;
    float q2 = axis_y * sin_half_angle;
    float q3 = axis_z * sin_half_angle;

    float r00 = 2 * (q0 * q0 + q1 * q1) - 1;
    float r01 = 2 * (q1 * q2 - q0 * q3);
    float r02 = 2 * (q1 * q3 + q0 * q2);

    float r10 = 2 * (q1 * q2 + q0 * q3);
    float r11 = 2 * (q0 * q0 + q2 * q2) - 1;
    float r12 = 2 * (q2 * q3 - q0 * q1);

    float r20 = 2 * (q1 * q3 - q0 * q2);
    float r21 = 2 * (q2 * q3 + q0 * q1);
    float r22 = 2 * (q0 * q0 + q3 * q3) - 1;

    return Matrix4(r00, r01, r02, 0.0f,
        r10, r11, r12, 0.0f,
        r20, r21, r22, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

static Matrix4
compute_rotation2(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float lengthSquared = x1 * x1 + y1 * y1;
    float z;

    if (lengthSquared <= 1.0f)
    {
        z = sqrtf(1.0f - lengthSquared);  // On unit sphere
    }
    else
    {
        z = 0.0f; // On hyperbola
    }
    Vector3 v0 = Vector3(x1, y1, z);
    v0 = normalize(v0);

    lengthSquared = x2 * x2 + y2 * y2;
    if (lengthSquared <= 1.0f)
    {
        z = sqrtf(1.0f - lengthSquared);  // On unit sphere
    }
    else
    {
        z = 0.0f; // On hyperbola
    }
    Vector3 v1 = Vector3(x2, y2, z);
    v1 = normalize(v1);

    Vector3 axis = cross(v0, v1);
    float angle = acosf(clamp(dot(v0, v1), -1.0f, 1.0f));

    // fromAxisAngle
    float c = cosf(angle);
    float s = sinf(angle);
    float t = 1.0f - c;

    float mag = sqrtf(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);

    if (mag > 0.0f)
    {
        axis.x /= mag;
        axis.y /= mag;
        axis.z /= mag;
    }
    else
    {
        axis.x = 0.0f;
        axis.y = 0.0f;
        axis.z = 1.0f;
    }
    float r00 = c + axis.x * axis.x * t;
    float r01 = axis.x * axis.y * t - axis.z * s;
    float r02 = axis.x * axis.z * t + axis.y * s;
    float r10 = axis.y * axis.x * t + axis.z * s;
    float r11 = c + axis.y * axis.y * t;
    float r12 = axis.y * axis.z * t - axis.x * s;
    float r20 = axis.z * axis.x * t - axis.y * s;
    float r21 = axis.z * axis.y * t + axis.x * s;
    float r22 = c + axis.z * axis.z * t;

    return Matrix4(r00, r01, r02, 0.0f,
        r10, r11, r12, 0.0f,
        r20, r21, r22, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

static void run(Config &config, SDL_Window *window, const char *file, bool headless, const char *outputFile, bool memoryLeakCheck)
{
    if (g_default_vao != 0)
        glBindVertexArray(g_default_vao);

    _camera.setPosition(Vector3(24, 8, -1));
    _camera.setPitch(0);
    _camera.setYaw(90);

    _camera.setFov(clamp(config.getFloat("Camera.fov", 60.0f), 1.0f, 179.0f));
    _camera.setNear(clamp(config.getFloat("Camera.near", 0.1f), 0.1f, 1.0f));
    _camera.setFar(clamp(config.getFloat("Camera.far", 500.0f), 10.0f, 8192.0f));

    _scene.load(file, &_camera, memoryLeakCheck);
    _scene.setCameraInitialPosition(&_camera);

    {
        float recommendedAmbient = 0.6f;
        float recommendedDiffuse = 1.0f;
        float recommendedSunIntensity = 1.0f;
        _scene.recommendLightingDefaults(&recommendedAmbient,
            &recommendedDiffuse, &recommendedSunIntensity);

        _scene.ambientWeight() = config.hasKey("Lighting.ambient_weight") ?
            clamp(config.getFloat("Lighting.ambient_weight", recommendedAmbient), 0.0f, 2.5f) :
            recommendedAmbient;

        _scene.diffuseWeight() = config.hasKey("Lighting.diffuse_weight") ?
            clamp(config.getFloat("Lighting.diffuse_weight", recommendedDiffuse), 0.0f, 2.5f) :
            recommendedDiffuse;

        _atmosphere.sunIntensity = config.hasKey("Lighting.sun_intensity") ?
            max(config.getFloat("Lighting.sun_intensity", recommendedSunIntensity), 0.0f) :
            recommendedSunIntensity;
    }

    _renderScale = clamp(config.getFloat("Display.render_scale", 1.0f), 0.25f, 1.0f);

    bool enableShadows = config.getBool("Shadows.enable", true);
    int shadowResolution = clamp(config.getInt("Shadows.resolution", 1024), 256, 4096);

    int w, h;
    SDL_GetWindowSizeInPixels(window, &w, &h);

    _scene.resize(w, h, _renderScale, shadowResolution, enableShadows);

    glEnable(GL_DEPTH_TEST);

    _atmosphere.skyColorHorizon = clamp(Vector3(config.getIntVector3("Atmosphere.horizon_color", IntVector3(56))) / 255.0f, 0.0f, 1.0f);
    _atmosphere.skyColorZenith = clamp(Vector3(config.getIntVector3("Atmosphere.zenith_color", IntVector3(26))) / 255.0f, 0.0f, 1.0f);
    _atmosphere.sunTightness = config.getFloat("Atmosphere.sun_tightness", 500.0f);

    _scene.fullbright() = config.getBool("Lighting.fullbright", false);
    _scene.setShowExtraWireOverlays(config.getBool("Display.extra_wire_overlays", true));
    _atmosphere.sunAltitude = fmodf(config.getFloat("Lighting.sun_altitude", 40.0f), 360.0f);
    _atmosphere.sunAzimuth = fmodf(config.getFloat("Lighting.sun_azimuth", 0.0f), 360.0f);
    _atmosphere.sunColor = clamp(Vector3(config.getIntVector3("Lighting.sun_color", IntVector3(255))) / 255.0f, 0.0f, 1.0f);

    Bloom *bloom = _scene.bloom();
    Compositor *compositor = _scene.compositor();

    _scene.enableBloom() = config.getBool("Postprocess.bloom_enable", true);
    bloom->filterRadius() = clamp(config.getFloat("Postprocess.bloom_radius", 0.005f), 0.0f, 0.025f);
    bloom->strength() = clamp(config.getFloat("Postprocess.bloom_strength", 0.2f), 0.0f, 1.0f);

    compositor->gamma() = clamp(config.getFloat("Postprocess.gamma", 2.2f), 0.1f, 5.0f);

    _scene.enableToneMapping() = config.getBool("Postprocess.tonemap_enable", true);
    compositor->exposure() = clamp(config.getFloat("Postprocess.tonemap_exposure", 1.0f), 0.1f, 10.0f);

    Material *gmat = _scene.ground()->material();

    _scene.enableGround() = config.getBool("Scene.ground_enable", true);
    _scene.groundSize().x = max(config.getFloat("Scene.ground_width", 200.0f), 0.0f);
    _scene.groundSize().y = max(config.getFloat("Scene.ground_height", 200.0f), 0.0f);
    gmat->diffuse = clamp(Vector3(config.getIntVector3("Scene.ground_diffuse", IntVector3(73))) / 255.0f, 0.0f, 1.0f);
    gmat->specular = clamp(Vector3(config.getIntVector3("Scene.ground_specular", IntVector3(73))) / 255.0f, 0.0f, 1.0f);
    gmat->tint = clamp(Vector3(config.getIntVector3("Scene.ground_tint", IntVector3(255))) / 255.0f, 0.0f, 1.0f);
    gmat->shininess = max(config.getFloat("Scene.ground_shininess", 256.0f), 0.0f);
    gmat->alpha = max(config.getFloat("Scene.ground_alpha", 1.0f), 0.0f);

    *_scene.dirLight() = kDirLight;
    *_scene.pointLight() = kPointLight;

    // If running in batch/headless mode, perform a single render pass and exit
    if (headless)
    {
        printf("Batch mode: loaded '%s' � performing single-frame render and exiting.\n", file);

        // Ensure viewport and clear state are correct for a single frame
        glViewport(0, 0, w, h);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update sun/atmosphere settings before rendering (normally done in debugMenu)
        DirLight &sun = *_scene.dirLight();
        Skybox *sky = _scene.skybox();

        Quaternion q1 = mutil::rotateaxis(Vector3(0.0f, 1.0f, 0.0f), mutil::radians(_atmosphere.sunAzimuth));
        Quaternion q2 = mutil::rotateaxis(Vector3(1.0f, 0.0f, 0.0f), mutil::radians(_atmosphere.sunAltitude));
        Quaternion q = q1 * q2;

        sun.direction = mutil::rotatevector(q, Vector3(0.0f, 0.0f, 1.0f));
        sun.color = _atmosphere.sunColor;
        sun.intensity = _atmosphere.sunIntensity;

        sky->setHorizonColor(_atmosphere.skyColorHorizon);
        sky->setZenithColor(_atmosphere.skyColorZenith);
        sky->setSunTightness(_atmosphere.sunTightness);

        // Render one frame (this exercises shaders, textures, upload paths)
        _scene.render(&_camera);

        // Make sure GL commands are flushed/completed and present the buffer
        glFinish();

        // Capture and save frame if output file is specified
        if (outputFile)
        {
            int channels = 3; // RGB
            int stride = channels * w;
            stride += (stride % 4) ? (4 - stride % 4) : 0; // Align to 4 bytes
            unsigned char *pixels = new unsigned char[stride * h];

            glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels);

            // Flip image vertically (OpenGL's origin is bottom-left, image formats use top-left)
            unsigned char *flipped = new unsigned char[stride * h];
            for (int y = 0; y < h; y++)
            {
                memcpy(flipped + y * stride, pixels + (h - 1 - y) * stride, stride);
            }

            // Save as PNG
            if (stbi_write_png(outputFile, w, h, channels, flipped, stride))
            {
                printf("Batch mode: frame saved to '%s'\n", outputFile);
            }
            else
            {
                printf("Batch mode: failed to save PNG to '%s'\n", outputFile);
            }

            delete[] pixels;
            delete[] flipped;
        }

        SDL_GL_SwapWindow(window);

        // Clean up GL resources the same way interactive shutdown would
        releaseTextures();
        _scene.unload();

        printf("Batch mode: single-frame render completed successfully.\n");
        return;
    }

    bool keys[SDL_SCANCODE_COUNT] = { 0 };
    bool mouseDown = false;
    bool mouseMoved = false;
    bool pickRequest = false;
    int pickX = 0;
    int pickY = 0;
    int mouseX_buttondown, mouseY_buttondown;
    Matrix4 curr_mat;
    Matrix4 transToCenter = Matrix4(1.0f);
    Matrix4 transFromCenter = Matrix4(1.0f);

    float time = SDL_GetTicks() / 1000.0f;
    float lastTime, deltaTime;
    bool mouse_over_debug = false;
    int32_t view_index = -1;

    double camZCenter;
    Matrix4 cam2World;
    Matrix4 World2Cam;
    Vector4 offset_vector;
    Vector4 modelWorldCenter;

    for (;;)
    {
        if (g_default_vao != 0)
            glBindVertexArray(g_default_vao);

        /* Make a translation matrix to and from the product Center point. This
           is defined by the current camera to world matrix and the Z distance
           defined in the PDF file. The arcball will rotate around this */

           // This should only be done if view has changed
        if (view_index != _camera.getCurrentViewIndex())
        {
            camZCenter = _camera.getCurrentViewCenterOrbitZ();
            cam2World = _camera.getCurrentViewCameraMatrix();
            offset_vector = Vector4(0.0, 0.0, camZCenter, 1.0);
            modelWorldCenter = cam2World * offset_vector;
            view_index = _camera.getCurrentViewIndex();

            transToCenter = Matrix4(1.0f);
            transFromCenter = Matrix4(1.0f);
            transToCenter = mutil::translate(transToCenter,
                Vector3(-modelWorldCenter.x, -modelWorldCenter.y, -modelWorldCenter.z));
            transFromCenter = mutil::translate(transFromCenter,
                Vector3(modelWorldCenter.x, modelWorldCenter.y, modelWorldCenter.z));

            // Lets remove the translation from the cam2World matrix as this causes issues
            cam2World.columns[3] = Vector4(0.0, 0.0, 0.0, 1.0);
            // The transpose is the inverse for rotation matrices. It may be that we should do an inverse here instead.
            World2Cam = mutil::transpose(cam2World);
        }

        lastTime = time;
        time = SDL_GetTicks() / 1000.0f;
        deltaTime = time - lastTime;

        SDL_Event evt;
        bool quit = false;
        float mouse_wheel_delta = 0.0f;

        while (SDL_PollEvent(&evt))
        {
            ImGui_ImplSDL3_ProcessEvent(&evt);

            switch (evt.type)
            {
            default:
                break;
            case SDL_EVENT_QUIT:
                quit = true;
                break;
            case SDL_EVENT_KEY_DOWN:
                keys[(uint8_t)evt.key.scancode] = true;
                switch (evt.key.scancode)
                {
                default:
                    break;
                case SDL_SCANCODE_0:
                    _scene.normalDebugMode() = 0;
                    break;
                case SDL_SCANCODE_1:
                    _scene.normalDebugMode() = 1;
                    break;
                case SDL_SCANCODE_2:
                    _scene.normalDebugMode() = 2;
                    break;
                case SDL_SCANCODE_3:
                    _scene.normalDebugMode() = 3;
                    break;
                case SDL_SCANCODE_4:
                    _scene.normalDebugMode() = 4;
                    break;
                case SDL_SCANCODE_5:
                    _scene.normalDebugMode() = 5;
                    break;
                case SDL_SCANCODE_6:
                    _scene.normalDebugMode() = 6;
                    break;
                case SDL_SCANCODE_7:
                    _scene.normalDebugMode() = 7;
                    break;
                }
                break;
            case SDL_EVENT_KEY_UP:
                keys[(uint8_t)evt.key.scancode] = false;
                break;
            case SDL_EVENT_WINDOW_RESIZED:
                _camera.setAspect((float)evt.window.data1 / (float)evt.window.data2);
                SDL_GetWindowSizeInPixels(window, &w, &h);
                _scene.resize(w, h, _renderScale, shadowResolution, enableShadows);
                break;
            case SDL_EVENT_MOUSE_MOTION:
                mouseMoved = true;
                if (mouseDown)
                {
                    mouseX_buttondown = evt.motion.x;
                    mouseY_buttondown = evt.motion.y;
                }
                else if (!mouseDown)
                {
                    _mouse_up_x = evt.motion.x;
                    _mouse_up_y = evt.motion.y;
                    mouseX_buttondown = evt.motion.x;
                    mouseY_buttondown = evt.motion.y;
                }
                break;
            case SDL_EVENT_MOUSE_WHEEL:
                curr_mat = _scene.models()[0].model();
                mouse_wheel_delta = evt.wheel.y;
                break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
                if (evt.button.button == SDL_BUTTON_LEFT)
                {
                    curr_mat = _scene.models()[0].model();
                    mouseDown = true;
                }
                break;
            case SDL_EVENT_MOUSE_BUTTON_UP:
                if (evt.button.button == SDL_BUTTON_LEFT)
                {
                    int dx = (int)evt.button.x - mouseX_buttondown;
                    int dy = (int)evt.button.y - mouseY_buttondown;
                    int dragPixels = std::abs(dx) + std::abs(dy);
                    SDL_Keymod mod = SDL_GetModState();
                    if ((mod & SDL_KMOD_CTRL) && !mouse_over_debug && dragPixels <= 3)
                    {
                        if (Product::cpuPickStorageLimited())
                        {
                            printf("\n[TrianglePick] disabled by tessellation size limits (skipped products=%u, kept CPU bytes=%" PRIu64 ")\n",
                                Product::cpuPickSkippedProductCount(), Product::cpuPickStoredBytes());
                        }
                        else
                        {
                            pickRequest = true;
                            pickX = (int)evt.button.x;
                            pickY = (int)evt.button.y;
                        }
                    }
                    mouseDown = false;
                }
                break;
            }
        }

        if (keys[SDL_SCANCODE_ESCAPE] || quit)
            break;

        float moveSpeed = keys[SDL_SCANCODE_LSHIFT] ? kMoveSpeed * kShiftMultiplier : kMoveSpeed;

        Vector3 cameraPos = _camera.position();
        if (keys[SDL_SCANCODE_W])
            cameraPos += moveSpeed * deltaTime * _camera.front();
        if (keys[SDL_SCANCODE_S])
            cameraPos -= moveSpeed * deltaTime * _camera.front();
        if (keys[SDL_SCANCODE_A])
            cameraPos -= moveSpeed * deltaTime * _camera.right();
        if (keys[SDL_SCANCODE_D])
            cameraPos += moveSpeed * deltaTime * _camera.right();
        if (keys[SDL_SCANCODE_E])
            cameraPos += moveSpeed * deltaTime * Vector3(0.0f, 1.0f, 0.0f);
        if (keys[SDL_SCANCODE_Q])
            cameraPos -= moveSpeed * deltaTime * Vector3(0.0f, 1.0f, 0.0f);
        _camera.setPosition(cameraPos);

        float pitch = _camera.pitch();
        float yaw = _camera.yaw();

        /* This is an arcball or trackball implementation for rotating the 3D part
           with the mouse. Math from Ken Shoemake 1992 */
        if ((mouseDown && mouseMoved && !mouse_over_debug) || mouse_wheel_delta != 0)
        {
            Matrix4 mat;

            if (mouse_wheel_delta == 0)
            {
                int windowWidth, windowHeight;
                SDL_GetWindowSizeInPixels(window, &windowWidth, &windowHeight);

                /* Normalize the mouse click position so that the screen center
                   is at [0, 0] */
                int min_dim = std::min(windowWidth, windowHeight) - 1;
                float px = (1.0f / (float)min_dim) * (2.0f * _mouse_up_x - windowWidth - 1);
                float py = (1.0f / (float)min_dim) * (2.0f * _mouse_up_y - windowHeight - 1);
                float qx = (1.0f / (float)min_dim) * (2.0f * mouseX_buttondown - windowWidth - 1);
                float qy = (1.0f / (float)min_dim) * (2.0f * mouseY_buttondown - windowHeight - 1);
                float zp, zq;

                zp = arc_projection(px, py);
                zq = arc_projection(qx, qy);

                // This is the rotation matrix from the arcball but in camera space
                Matrix4 mat_rot = compute_rotation2(px, py, zp, qx, qy, zq);

                /* Now convert this to world space rotation */
                Matrix4 mat_rot2 = cam2World * mat_rot * World2Cam;

                /* Now apply the rotation around the product center */
                mat = transFromCenter * mat_rot2 * transToCenter;

                /* Apply to current matrix */
                mat = mat * curr_mat;
            }
            else
            {
                /* Make a scaling matrix based upon the wheel value */
                float scaleFactor = mouse_wheel_delta * 0.1f;
                Matrix4 scaleMat = Matrix4(1.0f);
                scaleMat = mutil::scale(scaleMat, Vector3(1.0f + scaleFactor, 1.0f + scaleFactor, 1.0f + scaleFactor));
                mat = scaleMat * curr_mat;
            }

            _scene.models()[0].setModel(mat);
            mouseMoved = false;
        }

        if (keys[SDL_SCANCODE_UP])
            pitch += kTurnSpeed * deltaTime;
        if (keys[SDL_SCANCODE_DOWN])
            pitch -= kTurnSpeed * deltaTime;

        if (keys[SDL_SCANCODE_LEFT])
            yaw += kTurnSpeed * deltaTime;
        if (keys[SDL_SCANCODE_RIGHT])
            yaw -= kTurnSpeed * deltaTime;

        _camera.setPitch(mutil::clamp(pitch, -89.0f, 89.0f));
        _camera.setYaw(yaw);

        if (pickRequest)
        {
            Product *products = _scene.models();
            uint32_t nprod = _scene.mumProducts();
            for (uint32_t pi = 0; pi < nprod; pi++)
                products[pi].update();

            if (pickTriangleAtScreenPoint(pickX, pickY, _scene.width(), _scene.height(),
                &g_lastPick))
            {
                printPickResult(g_lastPick);
            }
            else
            {
                g_lastPick.valid = false;
                printf("\n[TrianglePick] no triangle hit at (%d,%d)\n", pickX, pickY);
            }
            pickRequest = false;
        }

        /* Rotation code */
#if 0
        Quaternion q = mutil::rotateaxis(Vector3(0.0f, 1.0f, 0.0f), time / 4.0f);
        // Quaternion q2 = mutil::rotateaxis(Vector3(1.0f, 0.0f, 0.0f), time / 2.0f);
        // Quaternion q = q1 * q2;

        for (uint32_t i = 0; i < _scene.numModels(); i++)
            _scene.models()[i].setRotation(q);
#endif
        // daylightCycle(scene, time);

        glDisable(GL_CULL_FACE);

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        _scene.render(&_camera);

        glViewport(0, 0, _scene.width(), _scene.height());

        mouse_over_debug = debugMenu(time, deltaTime);
        drawNormalDebugOverlay();
        drawTrianglePickOverlay();

        _camera.upDateView(_scene.models()[0]);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);
    }

    releaseTextures();

    /* Write configuration */

    config.setIntVector3("Atmosphere.horizon_color", IntVector3(_atmosphere.skyColorHorizon * 255.0f));
    config.setFloat("Atmosphere.sun_tightness", _atmosphere.sunTightness);
    config.setIntVector3("Atmosphere.zenith_color", IntVector3(_atmosphere.skyColorZenith * 255.0f));

    config.setFloat("Camera.far", _camera.far());
    config.setFloat("Camera.fov", _camera.fov());
    config.setFloat("Camera.near", _camera.near());

    config.setFloat("Display.render_scale", _renderScale);
    config.setBool("Display.extra_wire_overlays", _scene.showExtraWireOverlays());

    config.setFloat("Input.mouse_sensitivity", kMouseSensitivity);
    config.setFloat("Input.move_speed", kMoveSpeed);
    config.setFloat("Input.shift_multiplier", kShiftMultiplier);
    config.setFloat("Input.turn_speed", kTurnSpeed);

    config.setBool("Lighting.fullbright", _scene.fullbright());
    config.setFloat("Lighting.ambient_weight", _scene.ambientWeight());
    config.setFloat("Lighting.diffuse_weight", _scene.diffuseWeight());
    config.setFloat("Lighting.sun_altitude", _atmosphere.sunAltitude);
    config.setFloat("Lighting.sun_azimuth", _atmosphere.sunAzimuth);
    config.setIntVector3("Lighting.sun_color", IntVector3(_atmosphere.sunColor * 255.0f));
    config.setFloat("Lighting.sun_intensity", _atmosphere.sunIntensity);

    config.setBool("Postprocess.bloom_enable", _scene.enableBloom());
    config.setFloat("Postprocess.bloom_radius", _scene.bloom()->filterRadius());
    config.setFloat("Postprocess.bloom_strength", _scene.bloom()->strength());
    config.setFloat("Postprocess.gamma", _scene.compositor()->gamma());
    config.setBool("Postprocess.tonemap_enable", _scene.enableToneMapping());
    config.setFloat("Postprocess.tonemap_exposure", _scene.compositor()->exposure());

    config.setIntVector3("Scene.ground_diffuse", IntVector3(gmat->diffuse * 255.0f));
    config.setBool("Scene.ground_enable", _scene.enableGround());
    config.setFloat("Scene.ground_height", _scene.groundSize().x);
    config.setFloat("Scene.ground_shininess", gmat->shininess);
    config.setFloat("Scene.ground_alpha", gmat->alpha);
    config.setIntVector3("Scene.ground_specular", IntVector3(gmat->specular * 255.0f));
    config.setIntVector3("Scene.ground_tint", IntVector3(gmat->tint * 255.0f));
    config.setFloat("Scene.ground_width", _scene.groundSize().y);

    config.setBool("Shadows.enable", _scene.shadowMap() != nullptr);
    if (_scene.shadowMap())
        config.setInt("Shadows.resolution", _scene.shadowMap()->resolution());

    _scene.unload();
}

static void SetAllProductsEnabled(Product &prod, bool enabled)
{
    prod.setEnabled(enabled);
    for (uint32_t i = 0; i < prod.numChildren(); i++)
    {
        SetAllProductsEnabled(*prod.children()[i], enabled);
    }
}

static void EnableProductAndChildren(Product &prod)
{
    prod.setEnabled(true);
    for (uint32_t i = 0; i < prod.numChildren(); i++)
    {
        EnableProductAndChildren(*prod.children()[i]);
    }
}

static void EnableProductAndParents(Product &prod)
{
    prod.setEnabled(true);
    Product *parent = prod.parent();
    while (parent != nullptr)
    {
        parent->setEnabled(true);
        parent = parent->parent();
    }
}

static const char *ProductLabel(const Product &prod)
{
    return prod.name() ? prod.name() : "<unnamed product>";
}

static void DrawSelectedProductAttributesWindow()
{
    if (_selectedProduct == nullptr)
        return;

    if (ImGui::Begin("Product Attributes"))
    {
        ImGui::LabelText("Product", "%s", ProductLabel(*_selectedProduct));

        const char *attributesTitle = _selectedProduct->attributesTitle();
        if (attributesTitle && attributesTitle[0] != '\0')
            ImGui::LabelText("Group", "%s", attributesTitle);

        if (!_selectedProduct->hasAttributes())
        {
            ImGui::TextDisabled("No attributes on selected product.");
        }
        else
        {
            const std::vector<Product::AttributeBase> &bases = _selectedProduct->attributeBases();

            if (ImGui::BeginTable("SelectedProductAttributes_Single", 2,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("Property");
                ImGui::TableSetupColumn("Value");
                ImGui::TableHeadersRow();

                for (size_t baseIndex = 0; baseIndex < bases.size(); ++baseIndex)
                {
                    const Product::AttributeBase &base = bases[baseIndex];
                    if (base.entries.size() != 1)
                        continue;

                    const Product::AttributeRow &row = base.entries[0];
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(row.title.c_str());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(row.value.c_str());
                }

                ImGui::EndTable();
            }

            for (size_t baseIndex = 0; baseIndex < bases.size(); ++baseIndex)
            {
                const Product::AttributeBase &base = bases[baseIndex];
                if (base.entries.size() <= 1)
                    continue;

                char headerLabel[128];
                if (!base.title.empty())
                    snprintf(headerLabel, sizeof(headerLabel), "%s", base.title.c_str());
                else
                    snprintf(headerLabel, sizeof(headerLabel), "Attribute Group #%zu", baseIndex + 1);

                if (!ImGui::CollapsingHeader(headerLabel, ImGuiTreeNodeFlags_DefaultOpen))
                    continue;

                char tableId[64];
                snprintf(tableId, sizeof(tableId), "SelectedProductAttributes_%zu", baseIndex);

                if (ImGui::BeginTable(tableId, 2,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                    ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("Property");
                    ImGui::TableSetupColumn("Value");
                    ImGui::TableHeadersRow();

                    for (size_t entryIndex = 0; entryIndex < base.entries.size(); ++entryIndex)
                    {
                        const Product::AttributeRow &row = base.entries[entryIndex];
                        ImGui::TableNextRow();

                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(row.title.c_str());

                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted(row.value.c_str());
                    }

                    ImGui::EndTable();
                }
            }
        }
    }
    ImGui::End();
}

static void DrawTree(Product &prod)
{
    if (prod.numChildren() == 1)
    {
        Product *onlyChild = prod.children()[0];
        if (onlyChild->isPartNode() && onlyChild->partNameSameAsProduct())
        {
            DrawTree(*onlyChild);
            return;
        }
    }

    ImGui::PushID(&prod);
    ImGuiTreeNodeFlags flag = 0; // Removed ImGuiTreeNodeFlags_DefaultOpen to collapse by default

    if (&prod == _selectedProduct)
        flag |= ImGuiTreeNodeFlags_Selected;

    if (prod.numChildren() == 0)
    {
        flag |= ImGuiTreeNodeFlags_Leaf;
    }
    if (ImGui::TreeNodeEx(ProductLabel(prod), flag))
    {
        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
            _selectedProduct = &prod;

        ImGui::SameLine(0, -2);
        ImGui::PushID(prod.enabledPtr());
        ImGui::Checkbox("", prod.enabledPtr());
        ImGui::PopID();

        // Context menu for isolating parts
        if (ImGui::BeginPopupContextItem())
        {
            if (ImGui::MenuItem("Isolate Part"))
            {
                // Find the root product by traversing up the tree
                Product *root = &prod;
                while (root->parent() != nullptr)
                {
                    root = root->parent();
                }

                // Disable all products
                SetAllProductsEnabled(*root, false);

                // Enable this product and all its children
                EnableProductAndChildren(prod);

                // Enable all parent products up to the root
                EnableProductAndParents(prod);
            }
            ImGui::EndPopup();
        }

        for (uint32_t k = 0; k < prod.numChildren(); k++)
        {
            Product *child = prod.children()[k];
            DrawTree(*child);
        };
        ImGui::TreePop();
    }
    else
    {
        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
            _selectedProduct = &prod;

        // Tree node is collapsed, but we still want the context menu on the name
        ImGui::SameLine(0, -2);
        ImGui::PushID(prod.enabledPtr());
        ImGui::Checkbox("", prod.enabledPtr());
        ImGui::PopID();

        // Context menu when collapsed
        if (ImGui::BeginPopupContextItem())
        {
            if (ImGui::MenuItem("Isolate Part"))
            {
                // Find the root product by traversing up the tree
                Product *root = &prod;
                while (root->parent() != nullptr)
                {
                    root = root->parent();
                }

                // Disable all products
                SetAllProductsEnabled(*root, false);

                // Enable this product and all its children
                EnableProductAndChildren(prod);

                // Enable all parent products up to the root
                EnableProductAndParents(prod);
            }
            ImGui::EndPopup();
        }
    }
    ImGui::PopID();
}

static bool debugMenu(float time, float deltaTime)
{
    /* Debug menu */
    ImGuiIO &io = ImGui::GetIO();
    bool mouse_used = 0;

    ImGui::Begin("Debug");

    if (ImGui::BeginTabBar("Tabs"))
    {
        if (ImGui::BeginTabItem("Main"))
        {
            ImGui::SeparatorText("Performance");
            ImGui::LabelText("Framerate", "%.2f f/s", 1.0f / deltaTime);
            // ImGui::LabelText("Object Count", "%" PRIu32, _scene.numModels());

            ImGui::SeparatorText("Display");
            ImGui::LabelText("Resolution", "%dx%d", _scene.width(), _scene.height());

            float oldScale = _renderScale;
            ImGui::SliderFloat("Render Scale", &_renderScale, 0.25f, 1.0f);
            if (oldScale != _renderScale)
                _scene.resize(_scene.width(), _scene.height(), _renderScale, _scene.shadowMap()->resolution(), _scene.shadowMap() != nullptr);

            ImGui::SeparatorText("Debug");
            ImGui::Checkbox("Backface Culling", &_scene.backfaceCull());
            ImGui::Checkbox("Wireframe", &_scene.wireframe());
            bool showExtraWireOverlays = _scene.showExtraWireOverlays();
            if (ImGui::Checkbox("Extra Wire Overlays", &showExtraWireOverlays))
                _scene.setShowExtraWireOverlays(showExtraWireOverlays);
            ImGui::SliderInt("Normal Debug Mode", &_scene.normalDebugMode(), 0, 7);
            ImGui::Text("Hotkeys: 0=Off, 1=Normal RGB, 2=Normal Length, 3=Degenerate Mask, 4=Branch, 5=Front/Back, 6=Raw Normal, 7=Geom vs Imported");
            ImGui::Text("Triangle Pick: Ctrl+LeftClick (prints vertices/normals)");

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Camera"))
        {
            ImGui::SeparatorText("Transform");
            ImGui::InputFloat3("Position", (float *)&_camera.position(), "%.3f", ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Pitch", (float *)&_camera.pitch(), 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Yaw", (float *)&_camera.yaw(), 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);

            ImGui::SeparatorText("Projection");
            ImGui::SliderFloat("Field of View", (float *)&_camera.fov(), 1.0f, 179.0f, "%.3f");
            ImGui::SliderFloat("Near Plane", (float *)&_camera.near(), 0.1f, 1.0f, "%.3f");
            ImGui::SliderFloat("Far Plane", (float *)&_camera.far(), 10.0f, 81920.0f, "%.3f");

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Lighting"))
        {
            ImGui::SeparatorText("General");
            ImGui::Checkbox("Fullbright", &_scene.fullbright());
            if (ImGui::Button("Compute From Current File"))
            {
                _scene.recommendLightingDefaults(&_scene.ambientWeight(),
                    &_scene.diffuseWeight(), &_atmosphere.sunIntensity);
            }
            ImGui::SliderFloat("Ambient Weight", &_scene.ambientWeight(), 0.0f, 2.5f, "%.3f");
            ImGui::SliderFloat("Diffuse Weight", &_scene.diffuseWeight(), 0.0f, 2.5f, "%.3f");

            ImGui::SeparatorText("Sun");
            ImGui::SliderFloat("Altitude", &_atmosphere.sunAltitude, 0.0f, 360.0f);
            ImGui::SliderFloat("Azimuth", &_atmosphere.sunAzimuth, 0.0f, 360.0f);
            ImGui::SliderFloat("Intensity", &_atmosphere.sunIntensity, 0.0f, 10.0f);
            ImGui::ColorEdit3("Color", _atmosphere.sunColor.vec);

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Atmosphere"))
        {
            ImGui::SeparatorText("Colors");
            ImGui::ColorEdit3("Horizon Color", _atmosphere.skyColorHorizon.vec);
            ImGui::ColorEdit3("Zenith Color", _atmosphere.skyColorZenith.vec);

            ImGui::SeparatorText("Diffusion");
            ImGui::SliderFloat("Sun Tightness", &_atmosphere.sunTightness, 0.0f, 1000.0f);

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Scene"))
        {
            ImGui::PushID("Ground");

            ImGui::SeparatorText("Ground");
            ImGui::Checkbox("Enable", &_scene.enableGround());
            ImGui::SliderFloat("Ground", &_scene.groundHeight(), -20.0f, 20.0f);
            ImGui::SliderFloat2("Size", _scene.groundSize().vec, 0.0f, 200.0f);

            ImGui::SeparatorText("Ground Material");
            Material *material = _scene.ground()->material();
            ImGui::ColorEdit3("Diffuse", material->diffuse.vec);
            ImGui::ColorEdit3("Specular", material->specular.vec);
            ImGui::SliderFloat("Shininess", &material->shininess, 0.0f, 512.0f);
            ImGui::SliderFloat("Alpha", &material->alpha, 0.0f, 1.0f);
            ImGui::ColorEdit3("Tint", material->tint.vec);

            ImGui::PopID();

            /* Product Tree display */
            ImGui::SeparatorText("Objects");

            // Add "Show All Parts" button
            if (ImGui::Button("Show All Parts"))
            {
                Product &root = _scene.models()[0];
                SetAllProductsEnabled(root, true);
            }

            Product &root = _scene.models()[0];
            DrawTree(root);

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Shadows"))
        {
            ShadowMap *shadowMap = _scene.shadowMap();
            if (shadowMap)
            {
                GLuint depthMap = shadowMap->depthMap();

                ImGui::SeparatorText("Information");
                ImGui::LabelText("Resolution", "%d", shadowMap->resolution());

                ImGui::SeparatorText("Depth Map");
                ImGui::Image((void *)(intptr_t)depthMap, ImVec2(256, 256));
            }
            else
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Shadows not enabled");

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Postprocess"))
        {
            ImGui::SeparatorText("Basic");
            ImGui::SliderFloat("Gamma", &_scene.compositor()->gamma(), 0.1f, 5.0f);

            ImGui::PushID("Bloom");
            ImGui::SeparatorText("Bloom");
            ImGui::Checkbox("Enable", &_scene.enableBloom());
            ImGui::SliderFloat("Strength", &_scene.bloom()->strength(), 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Radius", &_scene.bloom()->filterRadius(), 0.001f, 0.025f, "%.4f");
            ImGui::PopID();

            ImGui::PushID("ToneMapping");
            ImGui::SeparatorText("Tone Mapping");
            ImGui::Checkbox("Enable", &_scene.enableToneMapping());
            ImGui::SliderFloat("Exposure", &_scene.compositor()->exposure(), 0.1f, 10.0f);
            ImGui::PopID();

            ImGui::EndTabItem();
        }

        uint32_t numViews = _camera.getNumViews();
        if (ImGui::BeginTabItem("Views"))
        {
            static int selectedValue = 0;

            if (numViews > 0)
            {
                for (uint32_t i = 0; i < numViews; i++)
                {
                    const char *name = _camera.getViewName(i);
                    if (name)
                    {
                        ImGui::RadioButton(name, &selectedValue, i);
                        if (ImGui::IsItemClicked())
                        {
                            /* If selected value is different than current one then update it */
                            if (i != _camera.getCurrentViewIndex())
                            {
                                _camera.setNewViewIndex(i);
                            }
                        }
                    }
                }
            }
            /* If selected value is different than current one then update it */

            ImGui::Checkbox("Enable Auto Motion", &_scene.enableMotion());
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    //bool isHovered = ImGui::IsItemHovered();
    //bool isFocused = ImGui::IsItemFocused();
    ImVec2 mousePositionAbsolute = ImGui::GetMousePos();

    /* Get the window rectangle */
    ImVec2 windowPositionAbsolute = ImGui::GetWindowPos();
    ImVec2 windowSize = ImGui::GetWindowSize();

    /* Check if the mouse is over the window */
    bool isMouseOverWindow = (mousePositionAbsolute.x >= windowPositionAbsolute.x) &&
        (mousePositionAbsolute.x <= windowPositionAbsolute.x + windowSize.x) &&
        (mousePositionAbsolute.y >= windowPositionAbsolute.y) &&
        (mousePositionAbsolute.y <= windowPositionAbsolute.y + windowSize.y);
    bool mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    mouse_used = mouse_down && isMouseOverWindow;

    ImGui::End();

    DrawSelectedProductAttributesWindow();

    /* Set scene properties */

    DirLight &sun = *_scene.dirLight();
    Skybox *sky = _scene.skybox();

    Quaternion q1 = mutil::rotateaxis(Vector3(0.0f, 1.0f, 0.0f), mutil::radians(_atmosphere.sunAzimuth));
    Quaternion q2 = mutil::rotateaxis(Vector3(1.0f, 0.0f, 0.0f), mutil::radians(_atmosphere.sunAltitude));
    Quaternion q = q1 * q2;

    sun.direction = mutil::rotatevector(q, Vector3(0.0f, 0.0f, 1.0f));
    sun.color = _atmosphere.sunColor;
    sun.intensity = _atmosphere.sunIntensity;

    sky->setHorizonColor(_atmosphere.skyColorHorizon);
    sky->setZenithColor(_atmosphere.skyColorZenith);
    sky->setSunTightness(_atmosphere.sunTightness);

    return mouse_used;
}
