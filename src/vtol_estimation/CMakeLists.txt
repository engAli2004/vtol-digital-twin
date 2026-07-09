cmake_minimum_required(VERSION 3.8)
project(vtol_estimation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(eskf_node src/eskf_node.cpp)
ament_target_dependencies(eskf_node rclcpp sensor_msgs nav_msgs geometry_msgs)
target_include_directories(eskf_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

install(TARGETS eskf_node DESTINATION lib/${PROJECT_NAME})
ament_package()
