cmake_minimum_required(VERSION 3.8)
project(mason_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(include)

add_executable(camera_node src/camera_node.cpp src/detection.cpp)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME})

# ADDED FOR NODE 2 (START)
add_executable(camera_node2 src/camera_node2.cpp)
ament_target_dependencies(camera_node2
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS
  camera_node2
  DESTINATION lib/${PROJECT_NAME})
# ADDED FOR NODE 2 (END)

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
