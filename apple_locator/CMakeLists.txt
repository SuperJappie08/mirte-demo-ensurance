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
## apple_locator::AppleDetector
ament_auto_add_library(apple_detector SHARED
  src/apple_detector.cpp
)
target_compile_features(apple_detector PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

## Use node to generate an executable
rclcpp_components_register_node(apple_detector
  PLUGIN "apple_locator::AppleDetector"
  EXECUTABLE apple_detector_node
)

## Declare a ROS 2 composible node as a library
## apple_locator::AppleDetectorIPC
ament_auto_add_library(apple_detector_ipc SHARED
  src/apple_detector_ipc.cpp
)
target_compile_features(apple_detector_ipc PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

## Use node to generate an executable
rclcpp_components_register_node(apple_detector_ipc
  PLUGIN "apple_locator::AppleDetectorIPC"
  EXECUTABLE apple_detector_ipc_node
)

## Declare a ROS 2 composible node as a library
## apple_locator::AppleDetectorIPC
ament_auto_add_library(apple_projector SHARED
  src/apple_projector.cpp
)
target_compile_features(apple_projector PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

## Use node to generate an executable
rclcpp_components_register_node(apple_projector
  PLUGIN "apple_locator::AppleProjector"
  EXECUTABLE apple_projector_node
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
)

