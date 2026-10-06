cmake_minimum_required(VERSION 3.8)
project(drishti_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drishti_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Decision core: no ROS dependency, on purpose. SPEC.md invariant 9.4.5.
# ---------------------------------------------------------------------------
add_library(supervisor_core src/supervisor_core.cpp)
target_include_directories(supervisor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---------------------------------------------------------------------------
# ROS node: a thin wrapper that holds no policy.
# ---------------------------------------------------------------------------
add_executable(safety_supervisor src/safety_supervisor_node.cpp)
target_link_libraries(safety_supervisor supervisor_core)
ament_target_dependencies(safety_supervisor
  rclcpp geometry_msgs nav_msgs std_msgs drishti_msgs)

install(TARGETS supervisor_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS safety_supervisor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Plain CTest executable -- no gtest, no ROS, so it also runs standalone on a
  # machine that cannot install ROS 2.
  add_executable(test_supervisor_core test/test_supervisor_core.cpp)
  target_link_libraries(test_supervisor_core supervisor_core)
  add_test(NAME supervisor_core COMMAND test_supervisor_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(supervisor_core)
ament_package()
