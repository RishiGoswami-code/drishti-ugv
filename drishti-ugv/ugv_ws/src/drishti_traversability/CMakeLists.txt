cmake_minimum_required(VERSION 3.8)
project(drishti_traversability)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Cost function: no ROS dependency, on purpose. SPEC.md 6.1 lives here and is
# tested standalone (test_traversability_core).
# ---------------------------------------------------------------------------
add_library(traversability_core src/traversability_core.cpp)
target_include_directories(traversability_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---------------------------------------------------------------------------
# Fusion node: elevation map in, traversability map out. Holds no policy.
# ---------------------------------------------------------------------------
add_executable(traversability_fusion src/traversability_fusion_node.cpp)
target_link_libraries(traversability_fusion traversability_core)
ament_target_dependencies(traversability_fusion
  rclcpp grid_map_core grid_map_ros grid_map_msgs)

# ---------------------------------------------------------------------------
# Nav2 costmap layer.
# ---------------------------------------------------------------------------
add_library(drishti_traversability_layer SHARED src/traversability_layer.cpp)
target_link_libraries(drishti_traversability_layer traversability_core)
ament_target_dependencies(drishti_traversability_layer
  rclcpp pluginlib nav2_costmap_2d grid_map_core grid_map_ros grid_map_msgs)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS traversability_core drishti_traversability_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS traversability_fusion DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Plain CTest executable: no gtest, no ROS, so it also runs standalone on a
  # machine that cannot install ROS 2.
  add_executable(test_traversability_core test/test_traversability_core.cpp)
  target_link_libraries(test_traversability_core traversability_core)
  add_test(NAME traversability_core COMMAND test_traversability_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(traversability_core drishti_traversability_layer)
ament_export_dependencies(nav2_costmap_2d grid_map_core grid_map_ros grid_map_msgs)
ament_package()
