cmake_minimum_required(VERSION 3.8)
project(arm_controller_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(fsm REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(orocos_kdl_vendor REQUIRED)
# find_package(kdl_parser REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(trajectory_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(backward_ros REQUIRED)


# Add your executable
ament_auto_add_executable(arm_controller_node src/arm_controller.cpp src/ik_solver.cpp)

# # Link dependencies
# ament_target_dependencies(arm_controller_node
#   rclcpp
#   rclcpp_action
#   fsm
#   std_msgs
#   orocos_kdl_vendor
#   kdl_parser
#   ament_index_cpp
#   control_msgs
#   trajectory_msgs
#   tf2_geometry_msgs
#   tf2_ros
#   tf2
#   backward_ros
# )

# # Install the node
# install(TARGETS
#   arm_controller_node
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
