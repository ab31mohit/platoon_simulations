cmake_minimum_required(VERSION 3.5)
project(platoon_simulations)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)



# Install
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

# # Install
# install(TARGETS ${EXEC_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY 
  launch rviz worlds urdf models meshes params
  DESTINATION share/${PROJECT_NAME}/
)

# Install all Python scripts
install(PROGRAMS
  src/teleop.py
  src/multi_teleop.py
  src/formation_control_node.py
  src/generate_trajectory_node.py
  src/trajectory_control_node.py
  src/performance_matrices.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/
#   DESTINATION include/
# )

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

# Macro for ament package
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()

