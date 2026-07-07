cmake_minimum_required(VERSION 3.8)
project(vtol_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
# Compile the Aerodynamics Node
add_executable(aerodynamics_node src/aerodynamics_node.cpp)
ament_target_dependencies(aerodynamics_node rclcpp sensor_msgs geometry_msgs)

# Install the executable so ROS 2 can run it
install(TARGETS
  aerodynamics_node
  DESTINATION lib/${PROJECT_NAME}
)
# Compile the Hover PID Controller
add_executable(hover_pid_node src/hover_pid_node.cpp)
ament_target_dependencies(hover_pid_node rclcpp sensor_msgs std_msgs nav_msgs )

# Install the executable
install(TARGETS
  aerodynamics_node
  hover_pid_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install Config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
