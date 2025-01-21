cmake_minimum_required(VERSION 3.8)
project(autoartisan2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rcpputils
  # realtime_tools
  geometry_msgs
  std_msgs
  sensor_msgs
  rclcpp_components
  # rosidl_default_generators
)

add_library(autoartisan2 SHARED src/navigation_node.cpp)

# Link any dependencies
ament_target_dependencies(autoartisan2
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  DIRECTORY include/
  DESTINATION include/autoartisan2
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

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
