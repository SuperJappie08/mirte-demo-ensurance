cmake_minimum_required(VERSION 3.8)
project(fsm)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArm.action"
  "srv/GetPoint.srv"
  "srv/ExchangePoint.srv"
  "srv/SendState.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
)

add_executable(fsm_node src/fsm.cpp)

ament_target_dependencies(fsm_node
  rclcpp
  rclcpp_action
  action_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(fsm_node
  ${cpp_typesupport_target})

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install
install(TARGETS
  fsm_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
