cmake_minimum_required(VERSION 3.8)
project(buttoning_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(buttoning_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Define a compile-time macro when the ROS_DISTRO environment variable is 'humble'.
# Some distributions ship a different cv_bridge header name (.h vs .hpp); allow
# source files to select the correct header at compile time.
if(DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    message(STATUS "ROS_DISTRO is humble -> defining ROS_DISTRO_HUMBLE")
    add_definitions(-DROS_DISTRO_HUMBLE)
  else()
    message(STATUS "ROS_DISTRO is $ENV{ROS_DISTRO} -> not defining ROS_DISTRO_HUMBLE")
  endif()
endif()

# Optional dependencies (comment out if not available)
find_package(realsense2 REQUIRED)
# find_package(onnxruntime REQUIRED)

# Optional: LibTorch for alternative SORT tracker implementation
find_package(Torch QUIET)
find_package(TorchVision QUIET)
if(Torch_FOUND AND TorchVision_FOUND)
  message(STATUS "LibTorch and TorchVision found - building LibTorch SORT tracker variant")
  set(BUILD_LIBTORCH_SORT ON)
elseif(Torch_FOUND AND NOT TorchVision_FOUND)
  message(STATUS "LibTorch found but TorchVision missing - skipping LibTorch SORT tracker")
  message(STATUS "  Install TorchVision from: https://github.com/pytorch/vision")
  set(BUILD_LIBTORCH_SORT OFF)
else()
  message(STATUS "LibTorch not found - skipping LibTorch SORT tracker (Eigen version still available)")
  set(BUILD_LIBTORCH_SORT OFF)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  buttoning_msgs
)

# Detection node executable
add_executable(detection_node
  src/detection_node.cpp
  src/sort_tracker.cpp
)
target_link_libraries(detection_node
  ${buttoning_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# Hand detection node executable
add_executable(hand_detection_node
  src/hand_detection_node.cpp
)
target_link_libraries(hand_detection_node
  ${buttoning_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# Arm controller node executable
add_executable(arm_controller_node
  src/arm_controller_node.cpp
)
target_link_libraries(arm_controller_node
  ${buttoning_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${OpenCV_LIBS}
)

# RealSense camera node executable
add_executable(realsense_node
  src/realsense_node.cpp
)
target_link_libraries(realsense_node
  ${buttoning_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)

# LibTorch SORT tracker test executable (optional, only if LibTorch is found)
if(BUILD_LIBTORCH_SORT)
  add_executable(test_libtorch_sort
    src/test_libtorch_sort.cpp
    src/sort_tracker_libtorch.cpp
  )
  target_include_directories(test_libtorch_sort PRIVATE
    ${TORCH_INCLUDE_DIRS}
  )
  target_link_libraries(test_libtorch_sort
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
    TorchVision::TorchVision
  )
  
  # Install LibTorch test executable
  install(TARGETS test_libtorch_sort
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install executables
install(TARGETS
  detection_node
  hand_detection_node
  arm_controller_node
  realsense_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
