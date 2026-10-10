include(GoogleTest)

add_executable(fleet_core_tests
  test_config.cpp
  test_rng.cpp
  test_grid_map.cpp
  test_map_generator.cpp
  test_distance_oracle.cpp
  test_world_model.cpp
  test_order_generator.cpp
  test_task_manager.cpp
  test_station_manager.cpp
  test_allocation_planning.cpp
  test_grid_executor.cpp
  test_invariant_checker.cpp
  test_fault_injector.cpp
  test_metrics_collector.cpp
  test_simulation.cpp
)
target_link_libraries(fleet_core_tests PRIVATE fleet_core ${FLEET_YAML_TARGET} GTest::gtest_main)
target_compile_definitions(fleet_core_tests PRIVATE FLEET_REPO_ROOT="${PROJECT_SOURCE_DIR}/..")
fleet_configure_target(fleet_core_tests)
gtest_discover_tests(fleet_core_tests DISCOVERY_TIMEOUT 60)
