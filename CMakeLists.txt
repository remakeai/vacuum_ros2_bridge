cmake_minimum_required(VERSION 3.8)
project(vacuum_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bumper.msg"
  "msg/Cliff.msg"
  "msg/VacuumStatus.msg"
  "msg/ActuatorStatus.msg"
  "msg/ActuatorCommand.msg"
  "msg/LedCommand.msg"
  "srv/SetActuator.srv"
  "srv/SetLed.srv"
  "srv/SetLidar.srv"
  DEPENDENCIES std_msgs
)

# Install the hand-written Python module(s) into the same importable package
# that rosidl_generate_interfaces() above already creates for this project, so
# `from vacuum_ros2_bridge.sangamio_client import ...` resolves alongside
# `from vacuum_ros2_bridge.msg import ...`.
#
# NOTE: we intentionally do NOT call ament_python_install_package(${PROJECT_NAME})
# here. rosidl already registers a Python package install under the project name,
# and a second registration collides on the ament_cmake_python_* custom targets
# (e.g. ament_cmake_python_symlink_vacuum_ros2_bridge), which breaks the build
# under `colcon build --symlink-install`. We install the .py file directly
# instead. (__init__.py is provided by the generated package.)
install(FILES
  ${PROJECT_NAME}/sangamio_client.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
