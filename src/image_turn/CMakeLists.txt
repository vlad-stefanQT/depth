cmake_minimum_required(VERSION 3.8)
project(image_turn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(image_turn_node 
  src/image_turn_node.cpp
)

ament_target_dependencies(image_turn_node
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
  OpenCV
  camera_info_manager
)

# Install executables
install(TARGETS
  image_turn_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
  OpenCV
  camera_info_manager
)

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

ament_package()
