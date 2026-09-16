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
