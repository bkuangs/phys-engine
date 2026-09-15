cmake_minimum_required(VERSION 3.20)

project(phys-engine LANGUAGES CXX)

option(PHYS_BUILD_SANDBOX "Build the headless sandbox entry point" ON)
option(PHYS_BUILD_BENCHMARKS "Build the benchmark entry points" OFF)

add_library(phys STATIC
    src/math/math_utils.cpp
    src/dynamics/rigid_body.cpp
    src/dynamics/mass_properties.cpp
    src/dynamics/integrator.cpp
    src/collision/aabb.cpp
    src/collision/broadphase.cpp
    src/collision/narrowphase.cpp
    src/collision/sat.cpp
    src/collision/gjk.cpp
    src/collision/epa.cpp
    src/solver/contact_constraint.cpp
    src/solver/sequential_impulse_solver.cpp
    src/world/physics_world.cpp
)
add_library(phys::phys ALIAS phys)

target_compile_features(phys PUBLIC cxx_std_20)
target_include_directories(phys PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_target_properties(phys PROPERTIES CXX_EXTENSIONS OFF)

if(PHYS_BUILD_SANDBOX)
    include(FetchContent)

    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
    )
    FetchContent_MakeAvailable(raylib)

    add_executable(phys_sandbox
        apps/sandbox/main.cpp
        apps/sandbox/camera.cpp
        apps/sandbox/debug_draw.cpp
        apps/sandbox/scenes/falling_boxes.cpp
        apps/sandbox/scenes/collision_stress.cpp
        apps/sandbox/scenes/friction_demo.cpp
        apps/sandbox/scenes/tunneling_demo.cpp
    )
    target_link_libraries(phys_sandbox PRIVATE phys::phys raylib)
    set_target_properties(phys_sandbox PROPERTIES CXX_EXTENSIONS OFF)
endif()

if(PHYS_BUILD_BENCHMARKS)
    add_executable(phys_broadphase_bench
        benchmarks/broadphase_bench.cpp
        benchmarks/bench_common.cpp
        benchmarks/alloc_counter.cpp
    )
    target_link_libraries(phys_broadphase_bench PRIVATE phys::phys)
    set_target_properties(phys_broadphase_bench PROPERTIES CXX_EXTENSIONS OFF)

    add_executable(phys_collision_bench
        benchmarks/collision_bench.cpp
        benchmarks/bench_common.cpp
        benchmarks/alloc_counter.cpp
    )
    target_link_libraries(phys_collision_bench PRIVATE phys::phys)
    set_target_properties(phys_collision_bench PROPERTIES CXX_EXTENSIONS OFF)
endif()

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
