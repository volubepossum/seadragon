cmake_minimum_required(VERSION 3.5)
project(ss_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
install(DIRECTORY
  ss_controller
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/
)

# Install Python scripts
install(PROGRAMS
    ss_controller/ss_controller_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ss_controller_node
)

ament_package()