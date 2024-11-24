cmake_minimum_required(VERSION 3.5)
project(teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
install(DIRECTORY
  teleop
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/
)

# Install Python scripts
install(PROGRAMS
    teleop/teleop_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME teleop_node
)

ament_package()

# Include directories
# include_directories(include)

# # Teleop node executable
# add_executable(teleop_node src/teleop_node.py)

# # Link libraries
# ament_target_dependencies(teleop_node rclcpp sensor_msgs geometry_msgs)

# # Install targets
# install(TARGETS
#     teleop_node
#     DESTINATION lib/${PROJECT_NAME}
# )

# # Ament package
# ament_package()
# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

