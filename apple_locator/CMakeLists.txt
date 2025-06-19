cmake_minimum_required(VERSION 3.8)
project(apple_locator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find_package(image_transport REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(rclcpp_components REQUIRED)

# add_executable(apple_detector src/apple_detector.cpp)
# target_include_directories(apple_detector PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(
#   apple_detector
#   "image_transport"
#   "rclcpp"
#   "cv_bridge"
#   "sensor_msgs"
#   "std_srvs"
#   "OpenCV"
#   "rclcpp_components"
# )

## Declare a ROS 2 composible node as a library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/apple_detector.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# To fix `DSO missing from command line` error with clang
target_link_libraries(${PROJECT_NAME}_node stdc++)

## Use node to generate an executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "apple_locator::AppleDetector"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)


# install(TARGETS apple_detector
#   DESTINATION lib/${PROJECT_NAME})

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

install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)

