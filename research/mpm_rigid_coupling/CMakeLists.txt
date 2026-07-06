# research/mpm_rigid_coupling — Paper #1 benchmark suite.
# Self-contained: only depends on basements_math + basements_coupling (M0/M1).

add_executable(bench_compare_all
    bench_compare_all.cpp
)
target_include_directories(bench_compare_all PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(bench_compare_all PRIVATE
    basements_math
    basements_coupling
)
target_compile_definitions(bench_compare_all PRIVATE
    RESEARCH_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/outputs"
)

# Ensure output directory tree exists.
add_custom_command(TARGET bench_compare_all POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/outputs/csv"
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/outputs/figures"
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/outputs/videos"
)

# Convenience: run all (method × scenario) and regenerate figures + videos.
add_custom_target(paper_figures
    COMMAND $<TARGET_FILE:bench_compare_all> --all
    COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/scripts/plot_compare.py"
    COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/scripts/render_animation.py"
    DEPENDS bench_compare_all
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating all paper figures and videos"
    USES_TERMINAL
)
