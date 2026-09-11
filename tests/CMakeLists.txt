# Test binaries are registered here as they are added in later phases.
# Each test is a standalone executable using tests/prc_test.h (see that
# header for the assertion macros), registered as a CTest test.
#
# Pattern for adding a new test:
#
#   add_executable(test_NAME tests/unit/test_NAME.c)
#   target_link_libraries(test_NAME prc_static)
#   target_include_directories(test_NAME PRIVATE tests)
#   add_test(NAME test_NAME COMMAND test_NAME)

add_executable(test_bitwrite tests/unit/test_bitwrite.c)
target_link_libraries(test_bitwrite prc_static)
target_include_directories(test_bitwrite PRIVATE tests)
add_test(NAME test_bitwrite COMMAND test_bitwrite)

add_executable(test_compress_tess tests/unit/test_compress_tess.c)
target_link_libraries(test_compress_tess prc_static)
target_include_directories(test_compress_tess PRIVATE tests)
add_test(NAME test_compress_tess COMMAND test_compress_tess)

add_executable(test_traversal tests/unit/test_traversal.c)
target_link_libraries(test_traversal prc_static)
target_include_directories(test_traversal PRIVATE tests)
add_test(NAME test_traversal COMMAND test_traversal)

add_executable(test_normals tests/unit/test_normals.c)
target_link_libraries(test_normals prc_static)
target_include_directories(test_normals PRIVATE tests)
add_test(NAME test_normals COMMAND test_normals)

add_executable(test_tess_3d tests/unit/test_tess_3d.c)
target_link_libraries(test_tess_3d prc_static)
target_include_directories(test_tess_3d PRIVATE tests)
add_test(NAME test_tess_3d COMMAND test_tess_3d)

add_executable(test_wire_tess tests/unit/test_wire_tess.c)
target_link_libraries(test_wire_tess prc_static)
target_include_directories(test_wire_tess PRIVATE tests)
add_test(NAME test_wire_tess COMMAND test_wire_tess)

add_executable(test_globals tests/unit/test_globals.c)
target_link_libraries(test_globals prc_static)
target_include_directories(test_globals PRIVATE tests)
add_test(NAME test_globals COMMAND test_globals)

add_executable(test_file_structure tests/unit/test_file_structure.c)
target_link_libraries(test_file_structure prc_static)
target_include_directories(test_file_structure PRIVATE tests)
add_test(NAME test_file_structure COMMAND test_file_structure)

add_executable(test_write_tree tests/unit/test_write_tree.c)
target_link_libraries(test_write_tree prc_static)
target_include_directories(test_write_tree PRIVATE tests)
add_test(NAME test_write_tree COMMAND test_write_tree)

add_executable(test_compressed_curve_release tests/unit/test_compressed_curve_release.c)
target_link_libraries(test_compressed_curve_release prc_static)
target_include_directories(test_compressed_curve_release PRIVATE tests)
add_test(NAME test_compressed_curve_release COMMAND test_compressed_curve_release)
