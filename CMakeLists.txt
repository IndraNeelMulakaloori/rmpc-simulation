cmake_minimum_required(VERSION 3.27)
project(rmpc_f1tenth LANGUAGES CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# Find ROS 2 packages


find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# This makes the header-only target rmpc::rmpc available
add_subdirectory(helper/randomized-mpc)

# Include custom header files
include_directories(include)

# Ros2 executable
add_executable(rmpc_node 
  src/rmpc_node.cu
  src/perception.cu
  src/controller.cu
  src/visualization.cu
  )

# Include directories for the rmpc_node target
target_include_directories(rmpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/helper/randomized-mpc/include  
)
# Link ROS 2 libraries and the rmpc CUDA library
ament_target_dependencies(
  rmpc_node 
  rclcpp 
  nav_msgs 
  sensor_msgs 
  ackermann_msgs 
  geometry_msgs 
  visualization_msgs 
  tf2 
  tf2_ros)
target_link_libraries(rmpc_node rmpc::rmpc)

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

install(TARGETS
  rmpc_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS 
  scripts/rmpc_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  rviz
  config
  waypoints
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})
ament_package()
