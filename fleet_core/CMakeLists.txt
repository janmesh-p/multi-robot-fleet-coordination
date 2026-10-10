cmake_minimum_required(VERSION 3.16)
project(fleet_core VERSION 0.1.0 LANGUAGES CXX)

# Standalone build: library, apps, tests. Inside fleet_ros (add_subdirectory): library only.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(FLEET_CORE_TOP_LEVEL ON)
else()
  set(FLEET_CORE_TOP_LEVEL OFF)
endif()

option(FLEET_CORE_BUILD_TESTS "Build fleet_core unit tests" ${FLEET_CORE_TOP_LEVEL})
option(FLEET_CORE_BUILD_APPS "Build fleet_core apps (run_sim)" ${FLEET_CORE_TOP_LEVEL})
option(FLEET_WERROR "Treat compiler warnings as errors" ON)
option(FLEET_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

if(FLEET_CORE_TOP_LEVEL)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FleetTargets)

# yaml-cpp: apt libyaml-cpp-dev, rosdep key yaml-cpp. Version 0.7 (Ubuntu 22.04) names the
# target yaml-cpp; 0.8 (Ubuntu 24.04) names it yaml-cpp::yaml-cpp.
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(FLEET_YAML_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(FLEET_YAML_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp was found but defines no CMake target; install libyaml-cpp-dev >= 0.7")
endif()

add_library(fleet_core
  src/common/config.cpp
  src/common/rng.cpp
  src/common/types.cpp
  src/world/grid_map.cpp
  src/world/map_generator.cpp
  src/world/distance_oracle.cpp
  src/world/world_model.cpp
  src/tasks/order_generator.cpp
  src/tasks/task_manager.cpp
  src/tasks/station_manager.cpp
  src/tasks/parking.cpp
  src/allocation/task_allocator.cpp
  src/allocation/nearest_allocator.cpp
  src/planning/path_planner.cpp
  src/planning/independent_astar.cpp
  src/traffic/traffic_manager.cpp
  src/faults/fault_injector.cpp
  src/faults/fault_manager.cpp
  src/execution/grid_executor.cpp
  src/verification/invariant_checker.cpp
  src/metrics/run_stats.cpp
  src/metrics/metrics_collector.cpp
  src/fleet_manager.cpp
  src/simulation.cpp
)
add_library(fleet_core::fleet_core ALIAS fleet_core)
target_include_directories(fleet_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(fleet_core PUBLIC cxx_std_17)
target_link_libraries(fleet_core PRIVATE ${FLEET_YAML_TARGET})
set_target_properties(fleet_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
fleet_configure_target(fleet_core)

if(FLEET_CORE_BUILD_APPS)
  add_executable(run_sim apps/run_sim.cpp)
  target_link_libraries(run_sim PRIVATE fleet_core)
  fleet_configure_target(run_sim)
  install(TARGETS run_sim RUNTIME DESTINATION bin)
endif()

if(FLEET_CORE_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_subdirectory(tests)
  fleet_add_header_check(fleet_core "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()
