cmake_minimum_required(VERSION 3.8)
project(pinocchio_)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(gravity_compensation
  src/gravity_compensation.cpp
)

add_executable(gravity_compensation_2
  src/gravity_compensation_2.cpp
)

add_executable(kinematics
  src/kinematics.cpp
)

add_executable(tf2_trial
  src/tf2_trial.cpp
)


ament_target_dependencies(gravity_compensation
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(gravity_compensation_2
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
)

ament_target_dependencies(kinematics
  rclcpp
  sensor_msgs
  pinocchio
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(tf2_trial
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)



install(TARGETS
  gravity_compensation
  gravity_compensation_2
  kinematics
  tf2_trial
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  include/
  src/
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
