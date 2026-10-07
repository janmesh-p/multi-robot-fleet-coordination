cmake_minimum_required(VERSION 3.16)
project(fleet_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

add_library(fleet_core
  src/grid_map.cpp
  src/registry.cpp
  src/order_generator.cpp
  src/fault_injector.cpp
  src/metrics_logger.cpp
  src/traffic_manager.cpp
  src/fleet_manager.cpp
)
target_include_directories(fleet_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(fleet_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(run_sim src/run_sim_main.cpp)
target_link_libraries(run_sim PRIVATE fleet_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_fleet_core tests/test_grid_map.cpp)
  target_compile_options(test_fleet_core PRIVATE -UNDEBUG)
  target_link_libraries(test_fleet_core PRIVATE fleet_core)
  add_test(NAME test_fleet_core COMMAND test_fleet_core)
endif()

install(TARGETS fleet_core run_sim EXPORT fleet_coreTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT fleet_coreTargets NAMESPACE fleet_core:: DESTINATION lib/cmake/fleet_core)
