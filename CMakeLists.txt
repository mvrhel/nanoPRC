cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(nano_prc VERSION 0.1.0)

find_package(Git QUIET)

set(PRC_GIT_DESCRIBE "v${PROJECT_VERSION}")
if (GIT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
	execute_process(
		COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always --match "v[0-9]*"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		OUTPUT_VARIABLE PRC_GIT_DESCRIBE
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET)
endif()

if (NOT PRC_GIT_DESCRIBE)
	set(PRC_GIT_DESCRIBE "v${PROJECT_VERSION}")
endif()

string(REGEX REPLACE "^v" "" PRC_VERSION_STRING "${PRC_GIT_DESCRIBE}")

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/include/prc_version.h.in"
	"${CMAKE_CURRENT_BINARY_DIR}/generated/prc_version.h"
	@ONLY)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

option(PRC_ENABLE_DEBUG_MEMORY "Enable PRC debug memory tracking" OFF)
option(PRC_ENABLE_UNZIPPED_FUZZ "Enable deterministic fuzzing of unzipped PRC sections" OFF)
option(NANOPRC_ENABLE_BREP "Enable B-Rep write facility (Phase 2)" OFF)
option(NANOPRC_BUILD_DEMOS "Build nanoPRC demo executables" ON)
option(NANOPRC_BUILD_TESTS "Build nanoPRC tests" ON)
option(NANOPRC_BUILD_SDL "Build SDL dependency (needed for viewer demos)" ON)

if (NANOPRC_BUILD_DEMOS AND NOT NANOPRC_BUILD_SDL)
	message(FATAL_ERROR "NANOPRC_BUILD_DEMOS=ON requires NANOPRC_BUILD_SDL=ON")
endif()

add_subdirectory(thirdparty) # thirdparty libraries
# demos are added after the library target so demo executables can link
# against the `nano_prc` target defined below

add_definitions(-DPRC_LITTLE_ENDIAN -DPRC_BUILD_SHARED)

# Our source files
set(PRC_CORE_SOURCES
	"src/prc_api.c"
	"src/prc_api_debug.c"
	"src/prc_api_write.c"
	"src/prc_api_write_pdf.c"
	"src/prc_style_api.c"
	"src/prc_tri_primitives_api.c"
	"src/prc_tri_primitives_helper_api.c"
	"src/prc_uncompressed_primitives_api.c"
	"src/prc_bit.c"
	"src/prc_write_bit.c"
	"src/prc_write_common.c"
	"src/prc_write_compress_tess.c"
	"src/prc_write_tess_3d.c"
	"src/prc_write_wire_tess.c"
	"src/prc_write_global.c"
	"src/prc_write_tree.c"
	"src/prc_write_file_structure.c"
	"src/prc_write_model.c"
	"src/prc_write_pdf.c"
	"src/prc_write_pdf_3d.c"
	"src/prc_context.c"
	"src/prc_double.c"
	"src/prc_error.c"
	"src/prc_huff.c"
	"src/prc_memory.c"
	"src/prc_parse_common.c"
	"src/prc_parse_extra_geometry.c"
	"src/prc_parse_file_structure.c"
	"src/prc_parse_global.c"
	"src/prc_parse_main.c"
	"src/prc_parse_tess.c"
	"src/prc_parse_tree.c"
	"src/prc_pdf.c"
    "src/prc_pdf_dict.c"
	"src/prc_pdf_stream.c"
	"src/prc_pdf_decrypt.c"
	"src/prc_pdf_xref.c"
	"src/prc_release.c"
	"src/prc_schema.c"
	"src/prc_vector_util.c"
	"src/prc_decode_compressed_tess.c"
	"src/prc_decode_markup_tess.c")

add_library(nano_prc SHARED ${PRC_CORE_SOURCES})

# Specify C99 Standard
set_property(TARGET nano_prc PROPERTY C_STANDARD 99)

# Include directories
target_include_directories(nano_prc PUBLIC "include")

# Link zlib
target_link_libraries(nano_prc PRIVATE zlib)

# Link math library (Linux only)
if (UNIX)
	target_link_libraries(nano_prc PRIVATE m)
endif()

# Specify C99 Standard
set_property(TARGET nano_prc PROPERTY C_STANDARD 99)

target_include_directories(nano_prc PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_compile_definitions(nano_prc PRIVATE PRC_VERSION_STRING="${PRC_VERSION_STRING}")
target_compile_definitions(nano_prc PUBLIC PRC_DEBUG_MEMORY=$<BOOL:${PRC_ENABLE_DEBUG_MEMORY}>)
target_compile_definitions(nano_prc PUBLIC PRC_ENABLE_UNZIPPED_FUZZ=$<BOOL:${PRC_ENABLE_UNZIPPED_FUZZ}>)

# Static twin of nano_prc, built from the same sources, for tests that need
# to call internal (non-PRC_EXPORT) functions directly -- those aren't
# visible across a shared-library boundary on Windows, since PRC_EXPORT is
# the only thing that gets dllexport'd. "src" is PUBLIC here (unlike on
# nano_prc) so test binaries can reach internal headers like prc_bit.h.
add_library(prc_static STATIC ${PRC_CORE_SOURCES})
set_property(TARGET prc_static PROPERTY C_STANDARD 99)
set_property(TARGET prc_static PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(prc_static PUBLIC "include" "src" "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(prc_static PUBLIC zlib)
if (UNIX)
	target_link_libraries(prc_static PUBLIC m)
endif()
target_compile_definitions(prc_static PRIVATE PRC_VERSION_STRING="${PRC_VERSION_STRING}")
target_compile_definitions(prc_static PUBLIC PRC_DEBUG_MEMORY=$<BOOL:${PRC_ENABLE_DEBUG_MEMORY}>)
target_compile_definitions(prc_static PUBLIC PRC_ENABLE_UNZIPPED_FUZZ=$<BOOL:${PRC_ENABLE_UNZIPPED_FUZZ}>)

if (NANOPRC_BUILD_DEMOS)
	add_subdirectory(demos)
endif()

if (NANOPRC_BUILD_TESTS)
	enable_testing()
	include(tests/CMakeLists.txt)
	include(tests/internal/CMakeLists.txt)
endif()

# Install what we built
install(TARGETS nano_prc)
