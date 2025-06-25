cmake_minimum_required(VERSION 3.8)
project(mirte_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies We import external dependencies for both Cpp and Python at the same time.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(behaviortree_ros2 REQUIRED)
# find_package(behaviortree_cpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(slam_toolbox REQUIRED)
# find_package(diagnostic_msgs REQUIRED)
# find_package(lifecycle_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(fsm REQUIRED)
# find_package(control_msgs REQUIRED)


# set(THIS_PACKAGE_DEPS
#     rclcpp
#     rclcpp_lifecycle 
#     behaviortree_ros2
#     behaviortree_cpp
#     lifecycle_msgs
#     diagnostic_msgs 
#     geometry_msgs
#     nav_msgs
#     nav2_msgs
#     sensor_msgs
#     std_srvs
#     slam_toolbox
#     fsm
#     control_msgs)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
ament_auto_add_executable(mirte_arborist src/mirte_arborist.cpp)

ament_auto_add_library(three_d_point_plugin SHARED src/three_d_point_sub.cpp)
target_compile_definitions(three_d_point_plugin PRIVATE  BT_PLUGIN_EXPORT )

ament_auto_add_library(get_point_plugin SHARED src/get_point_service.cpp)
target_compile_definitions(get_point_plugin PRIVATE  BT_PLUGIN_EXPORT )

ament_auto_add_library(fine_navigation_plugin SHARED src/fine_navigation_action.cpp)
target_compile_definitions(fine_navigation_plugin PRIVATE  BT_PLUGIN_EXPORT )

ament_auto_add_library(move_arm_plugin SHARED src/move_arm_action.cpp)
target_compile_definitions(move_arm_plugin PRIVATE  BT_PLUGIN_EXPORT )

ament_auto_add_library(toggle_gripper_plugin SHARED src/toggle_gripper_service.cpp)
target_compile_definitions(toggle_gripper_plugin PRIVATE  BT_PLUGIN_EXPORT )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/tree_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  three_d_point_plugin
  get_point_plugin
  fine_navigation_plugin
  move_arm_plugin
  toggle_gripper_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    trees
)
