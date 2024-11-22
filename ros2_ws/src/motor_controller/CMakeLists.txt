cmake_minimum_required(VERSION 3.5)
project(motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Motors.msg"
  "msg/Motor.msg"
)

# Install Python modules
install(DIRECTORY
  motor_controller
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/
)

# Install Python scripts
install(PROGRAMS
  motor_controller/motor_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_controller_node
)

# Install config directory
install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()