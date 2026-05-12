cmake_minimum_required(VERSION 3.8)
project(mrg_modimoop_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

#
# Libraries
#

# New pure planning library
add_library(path_planner_lib
  src/path_planner.cpp
)

target_include_directories(path_planner_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(path_planner_lib PUBLIC cxx_std_17)

#
# Executables
#

add_executable(localization
  src/localization.cpp
)

target_include_directories(localization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  localization
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

target_compile_features(localization PUBLIC cxx_std_17)

# New planner node
add_executable(planner
  src/planner.cpp
)

target_include_directories(planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  planner
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(planner
  path_planner_lib
)

target_compile_features(planner PUBLIC cxx_std_17)

add_executable(lqr_heading_controller src/lqr_heading_controller.cpp)

ament_target_dependencies(
  lqr_heading_controller
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

#
# Install
#

install(
  TARGETS
    lqr_heading_controller
    path_planner_lib
    localization
    planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY src/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY src/params/
  DESTINATION share/${PROJECT_NAME}/params
)

#
# Testing / lint
#

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()