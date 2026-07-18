cmake_minimum_required(VERSION 3.20)
project(nanoprc_py LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

# Prefer an already-built nano_prc shared library from the main project build
# tree for local development. For wheel builds/CI where that does not exist,
# fall back to building nanoPRC from source in-tree and link statically.
find_library(NANOPRC_LIBRARY
    NAMES nano_prc
    PATHS
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/lib"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/bin"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/Release"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/Debug"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build-debug/Debug"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build-debug/bin/Debug"
    NO_DEFAULT_PATH
)

pybind11_add_module(_core MODULE src/bindings.cpp)
target_include_directories(_core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")

if(NANOPRC_LIBRARY)
    message(STATUS "Using prebuilt nano_prc library: ${NANOPRC_LIBRARY}")
    target_link_libraries(_core PRIVATE ${NANOPRC_LIBRARY})
else()
    message(STATUS "Prebuilt nano_prc library not found; building in-tree static nanoPRC for _core")
    set(NANOPRC_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
    set(NANOPRC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(NANOPRC_BUILD_SDL OFF CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}/nanoprc-core")
    target_link_libraries(_core PRIVATE prc_static)
endif()

install(TARGETS _core LIBRARY DESTINATION nanoprc_py)
