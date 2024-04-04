cmake_minimum_required(VERSION 3.8)
project(ros2_virtual_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xviz REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_executable(virtual_camera src/virtual_camera.cpp src/virtual_camera_node.cpp src/virtual_camera_node.hpp)
target_include_directories(virtual_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(virtual_camera PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  virtual_camera
  rclcpp std_msgs sensor_msgs tf2
  tf2_ros tf2_eigen cv_bridge Eigen3
  image_geometry
)
target_link_libraries(virtual_camera xviz ${OpenCV_LIBRARIES})

add_executable(robot_mask src/robot_mask.cpp src/robot_mask_node.cpp src/robot_mask_node.hpp)

target_include_directories(robot_mask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(robot_mask PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_mask
  rclcpp std_msgs sensor_msgs tf2
  tf2_ros tf2_eigen cv_bridge Eigen3
  image_geometry
)
target_link_libraries(robot_mask xviz ${OpenCV_LIBRARIES})


install(TARGETS virtual_camera robot_mask
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
