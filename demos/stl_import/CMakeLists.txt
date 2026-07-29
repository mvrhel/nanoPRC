cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

# Do not call `project()` here so the demo joins the top-level project.
# Set C standard appropriate for the demo source file and declare the
# executable. Use source directory for include path.
set(CMAKE_C_STANDARD 99)

add_executable(nano_prc_stl_import
	src/stl_import.c)

target_link_libraries(nano_prc_stl_import PRIVATE nano_prc)
target_include_directories(nano_prc_stl_import PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Only link the math library if we aren't targeting Windows
if(NOT WIN32)
    target_link_libraries(nano_prc_stl_import PRIVATE m)
endif()
