cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(nano_prc_viewer)

set(CMAKE_CXX_STANDARD 14)

add_executable(nano_prc_viewer
	src/bloom.cpp
	src/camera.cpp
	src/composite.cpp
	src/config.cpp
    src/main.cpp
	src/mesh.cpp
	src/product.cpp
	src/scene.cpp
    src/shader.cpp
	src/shadow.cpp
	src/skybox.cpp
	src/texture.cpp
	src/text.cpp
	src/graphics2d.cpp
    glad/src/glad.c
	../../thirdparty/imgui/imgui.cpp
	../../thirdparty/imgui/imgui_draw.cpp
	../../thirdparty/imgui/imgui_widgets.cpp
	../../thirdparty/imgui/imgui_tables.cpp
	../../thirdparty/imgui/backends/imgui_impl_sdl3.cpp
	../../thirdparty/imgui/backends/imgui_impl_opengl3.cpp)

target_link_libraries(nano_prc_viewer PRIVATE nano_prc)
target_link_libraries(nano_prc_viewer PRIVATE SDL3::SDL3)

# Only link the math library if we aren't targeting Windows (MSVC's CRT
# bundles sqrt() etc.; glibc requires linking libm explicitly) -- camera.cpp/
# scene.cpp call libm functions directly, and MatrixUtil is header-only (no
# library target to pull this in transitively). Same pattern as every other
# demo's CMakeLists.txt.
if(NOT WIN32)
	target_link_libraries(nano_prc_viewer PRIVATE m)
endif()

target_include_directories(nano_prc_viewer PRIVATE "${}/include")

target_include_directories(nano_prc_viewer PRIVATE glad/include)

target_include_directories(nano_prc_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/MatrixUtil/MatrixUtil/include)

target_include_directories(nano_prc_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/imgui)

# pack shaders into header files (guarded, skip missing source files)
message("Creating shader output")
set(SHADER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_OUT_DIR src/shaders)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/src/shaders)

set(GENERATOR_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/shader_to_header.cmake)

# helper macro to add a shader only if the source exists
macro(add_shader_if_exists SRC_PATH OUT_REL HEADER_VAR VAR_NAME)
    if(EXISTS "${SRC_PATH}")
        set(OUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/${OUT_REL}")
        add_custom_command(
            OUTPUT ${OUT_PATH}
            COMMAND ${CMAKE_COMMAND} -DINPUT_FILE=${SRC_PATH} -DOUTPUT_FILE=${OUT_PATH} -DVARIABLE_NAME=${VAR_NAME} -P ${GENERATOR_SCRIPT}
            DEPENDS ${SRC_PATH}
            COMMENT "Generating ${OUT_REL}"
        )
        list(APPEND GENERATED_SHADER_HEADERS ${OUT_PATH})
        set(${HEADER_VAR} ${OUT_REL} PARENT_SCOPE)
    else()
        message(WARNING "Shader source not found: ${SRC_PATH} -> skipping generation of ${OUT_REL}")
    endif()
endmacro()

# call the helper for each shader you want to generate
add_shader_if_exists("${SHADER_SRC_DIR}/generic.vert" "${SHADER_OUT_DIR}/generic.vert.h" GENERIC_VERT_HEADER generic_vert_source)
add_shader_if_exists("${SHADER_SRC_DIR}/generic.frag" "${SHADER_OUT_DIR}/generic.frag.h" GENERIC_FRAG_HEADER generic_frag_source)
add_shader_if_exists("${SHADER_SRC_DIR}/skybox.vert" "${SHADER_OUT_DIR}/skybox.vert.h" SKYBOX_VERT_HEADER skybox_vert_source)
add_shader_if_exists("${SHADER_SRC_DIR}/skybox.frag" "${SHADER_OUT_DIR}/skybox.frag.h" SKYBOX_FRAG_HEADER skybox_frag_source)
add_shader_if_exists("${SHADER_SRC_DIR}/shadow.vert" "${SHADER_OUT_DIR}/shadow.vert.h" SHADOW_VERT_HEADER shadow_vert_source)
add_shader_if_exists("${SHADER_SRC_DIR}/shadow.frag" "${SHADER_OUT_DIR}/shadow.frag.h" SHADOW_FRAG_HEADER shadow_frag_source)
add_shader_if_exists("${SHADER_SRC_DIR}/composite.vert" "${SHADER_OUT_DIR}/composite.vert.h" COMPOSITE_VERT_HEADER composite_vert_source)
add_shader_if_exists("${SHADER_SRC_DIR}/composite.frag" "${SHADER_OUT_DIR}/composite.frag.h" COMPOSITE_FRAG_HEADER composite_frag_source)
add_shader_if_exists("${SHADER_SRC_DIR}/gui.vert" "${SHADER_OUT_DIR}/gui.vert.h" GUI_VERT_HEADER gui_vert_source)
add_shader_if_exists("${SHADER_SRC_DIR}/gui.frag" "${SHADER_OUT_DIR}/gui.frag.h" GUI_FRAG_HEADER gui_frag_source)
add_shader_if_exists("${SHADER_SRC_DIR}/downsample.frag" "${SHADER_OUT_DIR}/downsample.frag.h" DOWNSAMPLE_FRAG_HEADER downsample_frag_source)
add_shader_if_exists("${SHADER_SRC_DIR}/upsample.frag" "${SHADER_OUT_DIR}/upsample.frag.h" UPSAMPLE_FRAG_HEADER upsample_frag_source)

# if none found, warn (optional)
if(NOT GENERATED_SHADER_HEADERS)
    message(WARNING "No shader headers will be generated: no shader source files found in ${SHADER_SRC_DIR}")
else()
    add_custom_target(generate_nano_prc_shaders DEPENDS ${GENERATED_SHADER_HEADERS})
    add_dependencies(nano_prc_viewer generate_nano_prc_shaders)

    # Add the generated headers to the target sources and mark GENERATED
    target_sources(nano_prc_viewer PRIVATE ${GENERATED_SHADER_HEADERS})
    set_source_files_properties(${GENERATED_SHADER_HEADERS} PROPERTIES GENERATED TRUE)
endif()

target_include_directories(nano_prc_viewer PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/src)
