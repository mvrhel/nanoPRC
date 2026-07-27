# Black-box, public-API-only integration tests (as opposed to tests/unit's
# white-box tests, which link the internal prc_static target). Same
# add_executable/target_link_libraries/add_test pattern as tests/CMakeLists.txt,
# but linking the public nano_prc target since these tests only call public
# API -- same as any demo would.
#
# Like every CTest test in this project, these only run correctly after a
# full build: CTest does not build target dependencies itself, so the demo
# executables a test shells out to (via the $<TARGET_FILE:...> paths below)
# must already exist.

add_executable(test_stl_roundtrip tests/functional/test_stl_roundtrip.c)
target_link_libraries(test_stl_roundtrip nano_prc)
# Only link the math library if we aren't targeting Windows (MSVC's CRT
# bundles sqrt() etc.; glibc requires linking libm explicitly) -- same
# pattern as the root CMakeLists.txt's nano_prc/prc_static targets and
# every demo's CMakeLists.txt. Plain (no PUBLIC/PRIVATE keyword) signature,
# matching the target_link_libraries call just above -- CMake forbids
# mixing plain and keyword signatures for the same target.
if(NOT WIN32)
	target_link_libraries(test_stl_roundtrip m)
endif()
target_include_directories(test_stl_roundtrip PRIVATE tests)
target_compile_definitions(test_stl_roundtrip PRIVATE
	STL_EXPORT_EXE="$<TARGET_FILE:nano_prc_stl_export>"
	STL_IMPORT_EXE="$<TARGET_FILE:nano_prc_stl_import>"
	EXAMPLE_CUBE_PDF="${CMAKE_SOURCE_DIR}/examples/cube.pdf"
	ROUNDTRIP_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME test_stl_roundtrip COMMAND test_stl_roundtrip)
