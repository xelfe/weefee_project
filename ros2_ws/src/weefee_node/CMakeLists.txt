cmake_minimum_required(VERSION 3.8)
project(weefee_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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

# Add executables
add_executable(servo_commander src/servo_commander.cpp)
add_executable(quadruped_kinematics_controller src/quadruped_kinematics_controller.cpp)
add_executable(quadruped_visualizer src/quadruped_visualizer.cpp)

# Specify dependencies
ament_target_dependencies(servo_commander 
  rclcpp 
  std_msgs
)

ament_target_dependencies(quadruped_kinematics_controller 
  rclcpp 
  std_msgs
  geometry_msgs
)

ament_target_dependencies(quadruped_visualizer 
  rclcpp 
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Install executables
install(TARGETS
  servo_commander
  quadruped_kinematics_controller
  quadruped_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  FILES src/quadruped_inverse_kinematics.h src/quadruped_common.h
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
