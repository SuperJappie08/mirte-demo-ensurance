cmake_minimum_required(VERSION 3.8)
project(proj_pts_location)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find_package(rclcpp REQUIRED) find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED) find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED) find_package(tf2_ros REQUIRED)
# find_package(tf2_sensor_msgs REQUIRED) find_package(tf2_geometry_msgs
# REQUIRED) find_package(cv_bridge REQUIRED) find_package(image_transport
# REQUIRED) find_package(image_geometry REQUIRED)
# find_package(visualization_msgs REQUIRED) find_package(yaml-cpp REQUIRED)
# find_package(ament_index_cpp REQUIRED)

ament_auto_add_executable(point_locator src/point_locator.cpp)
# add_executable(point_locator src/point_locator.cpp)
# ament_target_dependencies(point_locator rclcpp sensor_msgs std_msgs
# geometry_msgs tf2 tf2_ros tf2_geometry_msgs image_geometry cv_bridge
# image_transport visualization_msgs ament_index_cpp )

target_link_libraries(point_locator yaml-cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED src/point_locator2.cpp)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "proj_pts_location::PointLocator2"
  EXECUTABLE point_locator2)

# install(FILES camera_intrinsics.yaml DESTINATION share/${PROJECT_NAME} )

# install(TARGETS point_locator DESTINATION lib/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config)
