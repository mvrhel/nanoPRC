# Internal development/diagnostic tools -- NOT part of the permanent test
# suite (see tests/CMakeLists.txt for that; none of these are registered
# via add_test/CTest). Each is a narrow, single-purpose command-line tool
# built during active debugging of the compressed-tessellation write path
# (see each file's own header comment for what it does specifically, how
# it works, and its known limitations). Kept as permanent reference tooling
# -- these found and precisely isolated three real, committed bit-encoding
# bugs during that investigation, and remain useful for any future format-
# level debugging -- but deliberately separated from tests/unit/ so they
# are never mistaken for permanent tests or part of the public API surface.

add_executable(check_single_tri_pdf tests/internal/check_single_tri_pdf.c)
target_link_libraries(check_single_tri_pdf prc_static)
target_include_directories(check_single_tri_pdf PRIVATE tests)

add_executable(scan_prc tests/internal/scan_prc.c)
target_link_libraries(scan_prc prc_static)
target_include_directories(scan_prc PRIVATE tests)

add_executable(embed_real_prc tests/internal/embed_real_prc.c)
target_link_libraries(embed_real_prc prc_static)
target_include_directories(embed_real_prc PRIVATE tests)

add_executable(extract_raw_prc tests/internal/extract_raw_prc.c)
target_link_libraries(extract_raw_prc prc_static)
target_include_directories(extract_raw_prc PRIVATE tests)

add_executable(splice_model_section tests/internal/splice_model_section.c)
target_link_libraries(splice_model_section prc_static)
target_include_directories(splice_model_section PRIVATE tests)

add_executable(splice_tree_section tests/internal/splice_tree_section.c)
target_link_libraries(splice_tree_section prc_static)
target_include_directories(splice_tree_section PRIVATE tests)

add_executable(splice_schema_tree_section tests/internal/splice_schema_tree_section.c)
target_link_libraries(splice_schema_tree_section prc_static)
target_include_directories(splice_schema_tree_section PRIVATE tests)

add_executable(splice_schema_tree_tess_section tests/internal/splice_schema_tree_tess_section.c)
target_link_libraries(splice_schema_tree_tess_section prc_static)
target_include_directories(splice_schema_tree_tess_section PRIVATE tests)

# Pure byte-level splicer, no nanoPRC linkage needed.
add_executable(splice_geom_from_real tests/internal/splice_geom_from_real.c)

add_executable(reencode_real_tess tests/internal/reencode_real_tess.c)
target_link_libraries(reencode_real_tess prc_static)
target_include_directories(reencode_real_tess PRIVATE tests)

add_executable(verify_tess_positions tests/internal/verify_tess_positions.c)
target_link_libraries(verify_tess_positions prc_static)
target_include_directories(verify_tess_positions PRIVATE tests)

add_executable(splice_tess_field tests/internal/splice_tess_field.c)
target_link_libraries(splice_tess_field prc_static)
target_include_directories(splice_tess_field PRIVATE tests)

add_executable(verify_manifold_real tests/internal/verify_manifold_real.c)
target_link_libraries(verify_manifold_real prc_static)
target_include_directories(verify_manifold_real PRIVATE tests)

add_executable(verify_manifold_pdf tests/internal/verify_manifold_pdf.c)
target_link_libraries(verify_manifold_pdf prc_static)
target_include_directories(verify_manifold_pdf PRIVATE tests)

add_executable(detect_tess_spikes tests/internal/detect_tess_spikes.c)
target_link_libraries(detect_tess_spikes prc_static)
target_include_directories(detect_tess_spikes PRIVATE tests)

add_executable(dump_raw_tess_arrays tests/internal/dump_raw_tess_arrays.c)
target_link_libraries(dump_raw_tess_arrays prc_static)
target_include_directories(dump_raw_tess_arrays PRIVATE tests)
add_executable(prc_to_pdf tests/internal/prc_to_pdf.c)
target_link_libraries(prc_to_pdf prc_static)
target_include_directories(prc_to_pdf PRIVATE tests)

add_executable(reauthor_tess_pdf tests/internal/reauthor_tess_pdf.c)
target_link_libraries(reauthor_tess_pdf prc_static)
target_include_directories(reauthor_tess_pdf PRIVATE tests)
