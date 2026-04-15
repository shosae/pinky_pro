cmake_minimum_required(VERSION 3.8)
project(pinky_docking_plugins)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(dependencies
  geometry_msgs
  nav2_util
  opennav_docking_core
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_library(pinky_april_dock SHARED
  src/april_dock.cpp
  src/pose_filter.cpp
)

ament_target_dependencies(pinky_april_dock
  ${dependencies}
)

pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

install(TARGETS pinky_april_dock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pinky_april_dock)
ament_export_dependencies(${dependencies})
ament_package()
