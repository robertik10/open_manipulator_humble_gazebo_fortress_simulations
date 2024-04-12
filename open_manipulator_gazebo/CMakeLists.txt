################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(open_manipulator_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
ament_export_dependencies(
    rclcpp
    std_msgs
    gazebo_ros_pkgs
    gazebo_ros2_control
    controller_manager
    xacro
    urdf
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(omx_gripper_sub_publisher src/omx_gripper_sub_publisher.cpp)
ament_target_dependencies(omx_gripper_sub_publisher rclcpp std_msgs gazebo_ros_pkgs gazebo_ros2_control controller_manager xacro urdf)
target_link_libraries(omx_gripper_sub_publisher ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS omx_gripper_sub_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
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

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
