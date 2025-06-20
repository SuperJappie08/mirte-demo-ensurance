cmake_minimum_required(VERSION 3.8)
project(gripper_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mirte_msgs REQUIRED)

# Executable
add_executable(gripper_controller_node src/gripper_controller.cpp)

ament_target_dependencies(gripper_controller_node
  rclcpp
  rclcpp_action
  control_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
  mirte_msgs
)

# Installation
install(TARGETS
  gripper_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: install header files if needed
# install(DIRECTORY include/
#   DESTINATION include/
# )

# Linting / tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
