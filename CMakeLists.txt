cmake_minimum_required(VERSION 3.5)
project(snake_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)

# Build the C++ node
add_executable(snake_robot_node src/snake_robot_node.cpp)
ament_target_dependencies(snake_robot_node rclcpp std_msgs sensor_msgs)

# Install C++ executable
install(TARGETS snake_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install launch, urdf, config, meshes
install(DIRECTORY launch urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch urdf config meshes resource DESTINATION share/${PROJECT_NAME})
install(PROGRAMS src/locomotion.py DESTINATION lib/${PROJECT_NAME})

