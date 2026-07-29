cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

# Do not call `project()` here to avoid creating a separate CMake project
# for the demos. The top-level project will add this directory after the
# `nano_prc` library target is defined so demo executables appear in the
# root solution and can link to the `nano_prc` target.
add_subdirectory(viewer)
add_subdirectory(quick_start)
add_subdirectory(json_export)
add_subdirectory(stl_export)
add_subdirectory(obj_export)
add_subdirectory(teapot_write)
add_subdirectory(stl_import)