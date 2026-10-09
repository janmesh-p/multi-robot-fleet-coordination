cmake_minimum_required(VERSION 3.16)
project(fleet_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fleet_msgs REQUIRED)

# fleet_core builds in-tree (library only; its tests and apps are off in this context).
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../fleet_core ${CMAKE_BINARY_DIR}/fleet_core_build)

add_executable(fleet_manager_node src/fleet_manager_node.cpp)
target_link_libraries(fleet_manager_node fleet_core)
ament_target_dependencies(fleet_manager_node rclcpp fleet_msgs)

install(TARGETS fleet_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Same relative layout as the repo, so map paths inside the configs keep resolving.
install(DIRECTORY ../maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../benchmarks/configs DESTINATION share/${PROJECT_NAME}/benchmarks)

ament_package()
