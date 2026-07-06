# ============================================================
# Basements Library - CMake Configuration
# ============================================================

cmake_minimum_required(VERSION 3.18)
project(Basements LANGUAGES CXX)

# C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CUDA if available
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    message(STATUS "CUDA enabled: ${CMAKE_CUDA_COMPILER}")
else()
    message(STATUS "CUDA not found - GPU features disabled")
endif()

# ============================================================
# Compiler Flags
# ============================================================

if(MSVC)
    add_compile_options(/W4 /permissive-)
    add_compile_options(/arch:AVX2)  # Enable AVX2
    add_compile_options(/utf-8)      # Enable UTF-8 source/execution charset
    if(WIN32)
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
        add_definitions(-DNOMINMAX)
    endif()
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options(-mavx2)  # Enable AVX2
endif()

# ============================================================
# Header-Only Math Library
# ============================================================

add_library(basements_math INTERFACE)
target_include_directories(basements_math INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(basements_math INTERFACE glm)

# ============================================================
# GLM (Moved to global scope)
# ============================================================
include(FetchContent)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

# ============================================================
# Google Test
# ============================================================

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ============================================================
# STB Libraries (Image Loading)
# ============================================================

FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master
)
FetchContent_MakeAvailable(stb)

# ============================================================
# TinyObjLoader
# ============================================================

FetchContent_Declare(
    tinyobjloader
    GIT_REPOSITORY https://github.com/tinyobjloader/tinyobjloader.git
    GIT_TAG release
)
FetchContent_MakeAvailable(tinyobjloader)

# ============================================================
# TinyGLTF (Modern 3D Format)
# ============================================================

FetchContent_Declare(
    tinygltf
    GIT_REPOSITORY https://github.com/syoyo/tinygltf.git
    GIT_TAG release
)
# tinygltf is header-only but often needs STB libraries. 
# We will use it as header-only library interface.
set(TINYGLTF_HEADER_ONLY ON CACHE BOOL "" FORCE)
set(TINYGLTF_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tinygltf)

# ============================================================
# TinyXML2 (URDF Support)
# ============================================================
# TinyXML2 (for URDF)
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(tinyxml2)

# FastNoiseLite (for Procedural Generation)
FetchContent_Declare(
    FastNoiseLite
    GIT_REPOSITORY https://github.com/Auburn/FastNoiseLite.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(FastNoiseLite)

# nlohmann/json (Scene Serializer)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

# ============================================================



# ============================================================
# Scene Editor Dependencies (GLFW, ImGui)
# ============================================================

FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# ============================================================
# Scene Editor Dependencies (GLFW, ImGui)
# ============================================================

option(BUILD_EDITOR "Build Scene Editor" OFF)

option(ENABLE_CUDA "Enable CUDA support" ON)
option(ENABLE_ROCM "Enable ROCm/HIP support" OFF)
option(ENABLE_METAL "Enable Apple Metal support" OFF)

# Backend selection (mutually exclusive for now)
set(BASEMENTS_BACKEND "CPU" CACHE STRING "Compute backend: CPU, CUDA, ROCM, METAL")
set_property(CACHE BASEMENTS_BACKEND PROPERTY STRINGS CPU CUDA ROCM METAL)

message(STATUS "Basements Backend: ${BASEMENTS_BACKEND}")

if(BUILD_EDITOR)
    # GLFW
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.3.9
    )
    FetchContent_MakeAvailable(glfw)
    


    # GLM is fetched in global scope

    # ImGui (with docking branch)
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG docking
    )
    FetchContent_MakeAvailable(imgui)
    
    # ImGui library (manual build since it's not CMake-based)
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC glfw)
    
    # Find OpenGL
    find_package(OpenGL REQUIRED)
    target_link_libraries(imgui PUBLIC OpenGL::GL)
    
    message(STATUS "Scene Editor: ENABLED")
endif()

# ============================================================
# Pybind11 (Python Interop) - Preparation
# ============================================================
# Creating a placeholder for Python integration. 
# Actual FetchContent will be added when we start Stage 6.2 to keep build time low for now.
# fetch_content_declare(pybind11 ...)

enable_testing()
include(GoogleTest)

# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)
    # ========================================
    # Core Math Tests
    # ========================================
    
    add_executable(test_vec3 tests/test_vec3.cpp)
    target_link_libraries(test_vec3 basements_math gtest_main)
    add_test(NAME Vec3Tests COMMAND test_vec3)
    
    add_executable(test_quaternion tests/test_quaternion.cpp)
    target_link_libraries(test_quaternion basements_math gtest_main)
    add_test(NAME QuaternionTests COMMAND test_quaternion)
    
    add_executable(test_matrix3 tests/test_matrix3.cpp)
    target_link_libraries(test_matrix3 basements_math gtest_main)
    add_test(NAME Matrix3Tests COMMAND test_matrix3)
    
    add_executable(test_core_math tests/test_core_math.cpp)
    target_link_libraries(test_core_math basements_math gtest_main)
    add_test(NAME CoreMathTests COMMAND test_core_math)

    add_executable(test_advanced_math tests/test_advanced_math.cpp)
    target_link_libraries(test_advanced_math basements_math gtest_main)
    add_test(NAME AdvancedMathTests COMMAND test_advanced_math)
    
    # ========================================
    # Physics Validation Tests
    # ========================================
    
    add_executable(test_free_fall tests/test_free_fall.cpp)
    target_link_libraries(test_free_fall basements_math gtest_main)
    add_test(NAME FreeFallValidation COMMAND test_free_fall)
    
    add_executable(test_energy_conservation tests/test_energy_conservation.cpp)
    target_link_libraries(test_energy_conservation basements_math gtest_main)
    add_test(NAME EnergyConservation COMMAND test_energy_conservation)
    
    add_executable(test_stacking tests/test_stacking.cpp)
    target_link_libraries(test_stacking basements_math gtest_main)
    add_test(NAME StackingStability COMMAND test_stacking)
    
    # New Physics Validation Tests
    add_executable(test_pendulum tests/test_pendulum.cpp)
    target_link_libraries(test_pendulum basements_math gtest_main)
    add_test(NAME PendulumValidation COMMAND test_pendulum)
    
    add_executable(test_newtons_cradle tests/test_newtons_cradle.cpp)
    target_link_libraries(test_newtons_cradle basements_math gtest_main)
    add_test(NAME NewtonsCradleValidation COMMAND test_newtons_cradle)
    
    add_executable(test_joints tests/test_joints.cpp)
    target_link_libraries(test_joints basements_math gtest_main)
    add_test(NAME JointConstraints COMMAND test_joints)
    
    add_executable(test_joint_limits tests/test_joint_limits.cpp)
    target_link_libraries(test_joint_limits basements_math gtest_main)
    add_test(NAME JointLimits COMMAND test_joint_limits)
    
    add_executable(test_material tests/test_material.cpp)
    target_link_libraries(test_material basements_math gtest_main)
    add_test(NAME MaterialSystem COMMAND test_material)
    
    add_executable(test_stress_strain tests/test_stress_strain.cpp)
    target_link_libraries(test_stress_strain basements_math gtest_main)
    add_test(NAME StressStrain COMMAND test_stress_strain)
    
    add_executable(test_thermodynamics tests/test_thermodynamics.cpp)
    target_link_libraries(test_thermodynamics basements_math gtest_main)
    add_test(NAME Thermodynamics COMMAND test_thermodynamics)
    
    add_executable(test_thermodynamics_integration tests/test_thermodynamics_integration.cpp)
    target_link_libraries(test_thermodynamics_integration basements_math gtest_main)
    add_test(NAME ThermoIntegration COMMAND test_thermodynamics_integration)
    
    add_executable(test_fracture tests/test_fracture.cpp)
    target_link_libraries(test_fracture basements_math gtest_main)
    add_test(NAME Fracture COMMAND test_fracture)
    
    add_executable(test_soft_body tests/test_soft_body.cpp)
    target_link_libraries(test_soft_body basements_math gtest_main)
    add_test(NAME SoftBody COMMAND test_soft_body)

    add_executable(test_projectile_motion tests/test_projectile_motion.cpp)
    target_link_libraries(test_projectile_motion basements_math gtest_main)
    add_test(NAME ProjectileMotion COMMAND test_projectile_motion)

    add_executable(test_elastic_collision tests/test_elastic_collision.cpp)
    target_link_libraries(test_elastic_collision basements_math gtest_main)
    add_test(NAME ElasticCollision COMMAND test_elastic_collision)
    
    # ========================================
    # Force Generators
    # ========================================
    
    add_executable(test_forces tests/test_forces.cpp)
    target_link_libraries(test_forces basements_math gtest_main)
    add_test(NAME ForceGenerators COMMAND test_forces)
    
    add_executable(test_fatigue tests/test_fatigue.cpp)
    target_link_libraries(test_fatigue basements_math gtest_main)
    add_test(NAME Fatigue COMMAND test_fatigue)
    
    add_executable(test_gear tests/test_gear.cpp)
    target_link_libraries(test_gear basements_math gtest_main)
    add_test(NAME Gear COMMAND test_gear)

    # ========================================
    # MPM Solver (CPU)
    # ========================================
    add_executable(test_mpm_cpu tests/test_mpm_cpu.cpp)
    target_link_libraries(test_mpm_cpu basements_math gtest_main)
    add_test(NAME MPM_CPU COMMAND test_mpm_cpu)

    add_executable(test_mpm_stress_isolation tests/test_mpm_stress_isolation.cpp)
    target_link_libraries(test_mpm_stress_isolation basements_math gtest_main)
    add_test(NAME MPMStressIsolation COMMAND test_mpm_stress_isolation)

    add_executable(test_mpm_constitutive tests/test_mpm_constitutive.cpp)
    target_link_libraries(test_mpm_constitutive basements_math gtest_main)
    add_test(NAME MPMConstitutive COMMAND test_mpm_constitutive)

    add_executable(test_mpm_autodiff_tape_neutral tests/test_mpm_autodiff_tape_neutral.cpp)
    target_link_libraries(test_mpm_autodiff_tape_neutral basements_math gtest_main)
    add_test(NAME MPMAutodiffTapeNeutral COMMAND test_mpm_autodiff_tape_neutral)
    
    add_executable(test_sensor tests/test_sensor.cpp)
    target_link_libraries(test_sensor basements_math gtest_main)
    add_test(NAME Sensor COMMAND test_sensor)
    
    add_executable(test_memory_manager tests/test_memory_manager.cpp)
    target_link_libraries(test_memory_manager basements_math gtest_main)
    add_test(NAME MemoryManager COMMAND test_memory_manager)
    
    add_executable(test_plugin_loader tests/test_plugin_loader.cpp)
    target_link_libraries(test_plugin_loader basements_math gtest_main)
    add_test(NAME PluginLoader COMMAND test_plugin_loader)
    
    add_executable(test_aerodynamics tests/test_aerodynamics.cpp)
    target_link_libraries(test_aerodynamics basements_math gtest_main)
    add_test(NAME Aerodynamics COMMAND test_aerodynamics)
    
    add_executable(test_sph tests/test_sph.cpp)
    target_link_libraries(test_sph basements_math gtest_main)
    add_test(NAME SPH COMMAND test_sph)
    
    # ========================================
    # Graphics Tests
    # ========================================
    
    add_executable(test_pbr_material tests/test_pbr_material.cpp)
    target_link_libraries(test_pbr_material basements_math gtest_main)
    add_test(NAME PBRMaterial COMMAND test_pbr_material)
    
    # ========================================
    # Backend Abstraction Tests
    # ========================================
    
    add_library(basements_backend STATIC
        src/backend/compute_backend.cpp
    )
    target_include_directories(basements_backend PUBLIC include)
    target_link_libraries(basements_backend PUBLIC basements_math)
    
    # CUDA Backend
    if(ENABLE_CUDA AND CMAKE_CUDA_COMPILER)
        find_package(CUDAToolkit REQUIRED)
        target_compile_definitions(basements_backend PUBLIC BASEMENTS_USE_CUDA)
        target_link_libraries(basements_backend PUBLIC CUDA::cudart)
        message(STATUS "Backend: CUDA support enabled")
    endif()
    
    # ROCm Backend (HIP)
    if(ENABLE_ROCM)
        find_package(hip QUIET)
        if(hip_FOUND)
            target_compile_definitions(basements_backend PUBLIC BASEMENTS_USE_ROCM)
            target_link_libraries(basements_backend PUBLIC hip::host)
            message(STATUS "Backend: ROCm/HIP support enabled")
        else()
            message(WARNING "ROCm requested but HIP not found")
        endif()
    endif()
    
    add_executable(test_backend tests/test_backend.cpp)
    target_link_libraries(test_backend basements_backend gtest_main)
    add_test(NAME Backend COMMAND test_backend)
    
    # ========================================
    # Scene Graph Library
    # ========================================
    
    add_library(basements_scene STATIC
        src/engine/scene_graph.cpp
        src/engine/command_history.cpp
        src/engine/workspace.cpp
        src/engine/module_manager.cpp
    )
    target_include_directories(basements_scene PUBLIC include)
    target_link_libraries(basements_scene PUBLIC basements_math)
    
    # ========================================
    # I/O Library (URDF Parser)
    # ========================================
    
    add_library(basements_io STATIC
        src/io/urdf_parser.cpp
        src/io/urdf_exporter.cpp
        src/io/urdf_physics_bridge.cpp
        src/io/mesh_importer.cpp
        ${tinyxml2_SOURCE_DIR}/tinyxml2.cpp
    )
    target_include_directories(basements_io PUBLIC
        include
        ${tinyxml2_SOURCE_DIR}
        ${tinyobjloader_SOURCE_DIR}
        ${tinygltf_SOURCE_DIR}
        ${stb_SOURCE_DIR}
    )
    target_compile_definitions(basements_io PRIVATE
        TINYGLTF_NO_STB_IMAGE
        TINYGLTF_NO_STB_IMAGE_WRITE
    )
    target_link_libraries(basements_io PUBLIC
        basements_math
    )
    
    add_executable(test_urdf_parser tests/test_urdf_parser.cpp)
    target_link_libraries(test_urdf_parser basements_io basements_scene gtest_main)
    add_test(NAME URDFParser COMMAND test_urdf_parser)

    add_executable(test_urdf_physics_bridge
        tests/test_urdf_physics_bridge.cpp
        src/engine/physics_world_gpu_stub.cpp
    )
    target_link_libraries(test_urdf_physics_bridge basements_io basements_scene gtest_main)
    add_test(NAME URDFPhysicsBridge COMMAND test_urdf_physics_bridge)
    set_tests_properties(URDFParser PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests")
    # Copy test_data to build output so exe runs correctly from build/Debug/
    add_custom_command(TARGET test_urdf_parser POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/tests/test_data"
            "$<TARGET_FILE_DIR:test_urdf_parser>/test_data"
    )
    
    # Physics pipeline validation (수직심화 검증)
    add_executable(test_physics_validation
        tests/test_physics_validation.cpp
        src/engine/physics_world_gpu_stub.cpp
    )
    target_link_libraries(test_physics_validation basements_math basements_scene gtest_main tinyxml2)
    add_test(NAME PhysicsValidation COMMAND test_physics_validation)

    add_executable(test_contact_cache
        tests/test_contact_cache.cpp
        src/engine/physics_world_gpu_stub.cpp
    )
    target_link_libraries(test_contact_cache basements_math gtest_main)
    add_test(NAME ContactCache COMMAND test_contact_cache)

    # ========================================
    # MPM ↔ RigidBody Coupling (수평확장 MVP)
    # ========================================
    add_library(basements_coupling STATIC
        src/physics/coupling/mpm_rigid_coupler.cpp
    )
    target_include_directories(basements_coupling PUBLIC include)
    target_link_libraries(basements_coupling PUBLIC basements_math)

    add_executable(test_mpm_rigid_coupler tests/test_mpm_rigid_coupler.cpp)
    target_link_libraries(test_mpm_rigid_coupler basements_coupling gtest_main)
    add_test(NAME MPMRigidCoupler COMMAND test_mpm_rigid_coupler)

    add_executable(test_mpm_rigid_cylinder tests/test_mpm_rigid_cylinder.cpp)
    target_link_libraries(test_mpm_rigid_cylinder basements_coupling gtest_main)
    add_test(NAME MPMRigidCylinder COMMAND test_mpm_rigid_cylinder)

    add_executable(test_mpm_rigid_sweep tests/test_mpm_rigid_sweep.cpp)
    target_include_directories(test_mpm_rigid_sweep PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_mpm_rigid_sweep basements_coupling gtest_main)
    add_test(NAME MPMRigidSweep COMMAND test_mpm_rigid_sweep)

    # ========================================
    # Energy / Momentum Drift Benchmark (수직검증)
    # CSVs dump to outputs/figures/, post-processed by scripts/plot_energy_drift.py
    # ========================================
    add_executable(bench_energy_drift
        benchmarks/bench_energy_drift.cpp
        src/engine/physics_world_gpu_stub.cpp
    )
    target_link_libraries(bench_energy_drift basements_math)
    target_compile_definitions(bench_energy_drift PRIVATE
        BASEMENTS_FIGURES_DIR="${CMAKE_SOURCE_DIR}/outputs/figures"
    )
    # Ensure output directory exists before the benchmark runs.
    add_custom_command(TARGET bench_energy_drift POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_SOURCE_DIR}/outputs/figures"
    )

    # One-shot target: build the benchmark, run it with --long-form (60 s
    # solver scenarios), then re-render figures. Use:
    #   cmake --build build --config Release --target drift_bench_long_form
    add_custom_target(drift_bench_long_form
        COMMAND $<TARGET_FILE:bench_energy_drift> --long-form
        COMMAND ${CMAKE_COMMAND} -E echo "[drift_bench_long_form] running plot_energy_drift.py"
        COMMAND python "${CMAKE_SOURCE_DIR}/scripts/plot_energy_drift.py" "${CMAKE_SOURCE_DIR}/outputs/figures"
        DEPENDS bench_energy_drift
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Running bench_energy_drift --long-form + plot regeneration"
        USES_TERMINAL
    )

    # ========================================
    # MPM ↔ Rigid Coupling Drift Baseline (M0 reference)
    # ========================================
    add_executable(bench_mpm_rigid_drift
        benchmarks/bench_mpm_rigid_drift.cpp
    )
    target_link_libraries(bench_mpm_rigid_drift basements_math basements_coupling)
    target_compile_definitions(bench_mpm_rigid_drift PRIVATE
        BASEMENTS_FIGURES_DIR="${CMAKE_SOURCE_DIR}/outputs/figures"
    )

    # ========================================
    # Compilation Tests
    # ========================================
    
    add_executable(test_formulas_compilation tests/test_formulas_compilation.cpp)
    target_link_libraries(test_formulas_compilation basements_math)
    add_test(NAME FormulasCompilation COMMAND test_formulas_compilation)
    
    add_executable(test_unified_engine_complete 
        tests/test_unified_engine_complete.cpp
        src/engine/unified_engine.cpp
        src/core/task_system.cpp
    )
    target_link_libraries(test_unified_engine_complete basements_math)
    add_test(NAME UnifiedEngineComplete COMMAND test_unified_engine_complete)
    
    add_executable(test_physics_world_gpu
        tests/test_physics_world_gpu.cpp
        src/engine/physics_world_gpu.cpp
    )
    target_link_libraries(test_physics_world_gpu basements_math gtest_main)
    add_test(NAME PhysicsWorldGPU COMMAND test_physics_world_gpu)
    
    # ========================================
    # GPU Tests (CUDA)
    # ========================================
    
    if(CMAKE_CUDA_COMPILER)
        add_executable(test_cuda_hello tests/test_cuda_hello.cu)
        target_link_libraries(test_cuda_hello basements_math)
        add_test(NAME CudaHello COMMAND test_cuda_hello)
        
        add_executable(test_math_gpu tests/test_math_gpu.cu)
        target_link_libraries(test_math_gpu basements_math)
        add_test(NAME MathGPU COMMAND test_math_gpu)
        
        add_executable(test_broadphase_gpu tests/test_broadphase_gpu.cu)
        target_link_libraries(test_broadphase_gpu basements_math gtest_main)
        add_test(NAME BroadphaseGPU COMMAND test_broadphase_gpu)
        
        add_executable(test_narrowphase_gpu tests/test_narrowphase_gpu.cu)
        target_link_libraries(test_narrowphase_gpu basements_math gtest_main)
        add_test(NAME NarrowphaseGPU COMMAND test_narrowphase_gpu)
        
        add_executable(test_solver_gpu tests/test_solver_gpu.cu)
        target_link_libraries(test_solver_gpu basements_math gtest_main)
        add_test(NAME SolverGPU COMMAND test_solver_gpu)
    endif()

    # ========================================
    # Standalone MPM Visualizer
    # ========================================
    add_executable(test_mpm_visualizer tests/test_mpm_visualizer.cpp)
    target_link_libraries(test_mpm_visualizer basements_math glfw opengl32)
    # target_include_directories(test_mpm_visualizer PRIVATE ${glad_SOURCE_DIR}/include)
endif()

# ============================================================
# Scene Editor Executable
# ============================================================

if(BUILD_EDITOR)
    add_executable(basements_editor
        apps/editor/main.cpp
        src/editor/editor_app.cpp
        src/editor/graphics/shader.cpp
        src/editor/graphics/framebuffer.cpp
        src/editor/graphics/renderer.cpp
        src/editor/graphics/camera.cpp
        src/editor/graphics/gl_loader.cpp
        src/editor/graphics/gizmo.cpp
        src/editor/graphics/primitive_factory.cpp
        src/editor/selection_manager.cpp
        src/editor/localization.cpp
        src/editor/ui/panels/inspector_panel.cpp
        src/editor/ui/panels/physics_panel.cpp
        src/editor/ui/panels/hierarchy_panel.cpp
        src/editor/ui/panels/console_panel.cpp
        src/io/mesh_importer.cpp
        src/engine/scene_serializer.cpp
        src/engine/physics_world_gpu_stub.cpp
        src/editor/scripting/python_bridge_stub.cpp
    )
    target_link_libraries(basements_editor PRIVATE
        basements_scene
        basements_io
        basements_coupling
        glfw
        imgui
        OpenGL::GL
        tinyobjloader
        tinygltf
        nlohmann_json::nlohmann_json
    )
    target_include_directories(basements_editor PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${glm_SOURCE_DIR}
    )
    target_compile_definitions(basements_editor PRIVATE
        GLM_ENABLE_EXPERIMENTAL
        TINYGLTF_NO_STB_IMAGE
        TINYGLTF_NO_STB_IMAGE_WRITE
    )

    # Editor trace logging. ON by default while UI is in active stabilisation;
    # turn OFF for a clean release build with no [trace] noise.
    option(BASEMENTS_EDITOR_TRACE "Enable Editor trace logging (menu/play/import callbacks)" ON)
    if(BASEMENTS_EDITOR_TRACE)
        target_compile_definitions(basements_editor PRIVATE BASEMENTS_EDITOR_TRACE=1)
    endif()
    
    # Copy to bin folder
    set_target_properties(basements_editor PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
    
    # Copy Shaders
    add_custom_command(TARGET basements_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/src/editor/shaders
        ${CMAKE_BINARY_DIR}/bin/Release/editor/shaders
    )
    # Also copy for Debug config if needed (or just generic bin/editor/shaders)
    add_custom_command(TARGET basements_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/src/editor/shaders
        ${CMAKE_BINARY_DIR}/bin/editor/shaders
    )
    
    message(STATUS "Scene Editor target: basements_editor")
endif()

# ============================================================
# Examples (Optional)
# ============================================================

option(BUILD_EXAMPLES "Build example programs" OFF)

if(BUILD_EXAMPLES)
    # Add examples here when needed
endif()

# ============================================================
# Research sub-projects
# ------------------------------------------------------------
# Self-contained projects that live alongside the main engine but ship
# independent build targets. Currently:
#   - research/mpm_rigid_coupling — Paper #1 benchmark suite (M0..M4.5).
# Disabling this option does not affect the rest of the build.
# ============================================================

option(BUILD_RESEARCH_MPM_RIGID
    "Build the MPM ↔ rigid coupling benchmark sub-project" ON)
if(BUILD_RESEARCH_MPM_RIGID)
    add_subdirectory(research/mpm_rigid_coupling)
    message(STATUS "Research sub-project: mpm_rigid_coupling (M0..M4.5)")
endif()

# ============================================================
# Summary
# ============================================================

message(STATUS "")
message(STATUS "=== Basements Configuration ===")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CUDA Enabled: ${CMAKE_CUDA_COMPILER}")
message(STATUS "Build Tests: ${BUILD_TESTING}")
message(STATUS "Build Examples: ${BUILD_EXAMPLES}")
message(STATUS "==============================")
message(STATUS "")
