add_executable(phys_broadphase_tests
	collision/broadphase_tests.cpp
)
target_link_libraries(phys_broadphase_tests PRIVATE phys::phys)
set_target_properties(phys_broadphase_tests PROPERTIES CXX_EXTENSIONS OFF)

add_test(NAME phys_broadphase_tests COMMAND phys_broadphase_tests)

add_executable(phys_dynamic_tree_tests
	collision/dynamic_aabb_tree_tests.cpp
)
target_link_libraries(phys_dynamic_tree_tests PRIVATE phys::phys)
set_target_properties(phys_dynamic_tree_tests PROPERTIES CXX_EXTENSIONS OFF)

add_test(NAME phys_dynamic_tree_tests COMMAND phys_dynamic_tree_tests)

add_executable(phys_narrowphase_tests
	collision/narrowphase_tests.cpp
)
target_link_libraries(phys_narrowphase_tests PRIVATE phys::phys)
set_target_properties(phys_narrowphase_tests PROPERTIES CXX_EXTENSIONS OFF)

add_test(NAME phys_narrowphase_tests COMMAND phys_narrowphase_tests)

add_executable(phys_solver_tests
	solver/sequential_impulse_solver_tests.cpp
)
target_link_libraries(phys_solver_tests PRIVATE phys::phys)
set_target_properties(phys_solver_tests PROPERTIES CXX_EXTENSIONS OFF)

add_test(NAME phys_solver_tests COMMAND phys_solver_tests)

add_executable(phys_rigid_body_tests
	dynamics/rigid_body_tests.cpp
)
target_link_libraries(phys_rigid_body_tests PRIVATE phys::phys)
set_target_properties(phys_rigid_body_tests PROPERTIES CXX_EXTENSIONS OFF)

add_test(NAME phys_rigid_body_tests COMMAND phys_rigid_body_tests)

if(PHYS_BUILD_BENCHMARKS)
    add_executable(phys_benchmark_tests
        benchmarks/benchmark_tests.cpp
        ../benchmarks/bench_common.cpp
        ../benchmarks/bench_scenes.cpp
        ../benchmarks/alloc_counter.cpp
    )
    target_include_directories(phys_benchmark_tests PRIVATE "${PROJECT_SOURCE_DIR}/benchmarks")
    target_link_libraries(phys_benchmark_tests PRIVATE phys::phys)
    set_target_properties(phys_benchmark_tests PROPERTIES CXX_EXTENSIONS OFF)
    add_test(NAME phys_benchmark_tests COMMAND phys_benchmark_tests)
endif()

add_executable(phys_sleeping_tests
	world/sleeping_tests.cpp
)
target_link_libraries(phys_sleeping_tests PRIVATE phys::phys)
set_target_properties(phys_sleeping_tests PROPERTIES CXX_EXTENSIONS OFF)

add_test(NAME phys_sleeping_tests COMMAND phys_sleeping_tests)
