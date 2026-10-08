cmake_minimum_required(VERSION 3.8)
project(drishti_sim_isaac)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
