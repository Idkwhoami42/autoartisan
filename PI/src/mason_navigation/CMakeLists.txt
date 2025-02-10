cmake_minimum_required(VERSION 3.8)
project(mason_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mason_hardware REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rcpputils
  geometry_msgs
  std_msgs
  sensor_msgs
  rclcpp_components
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/HomingSequence.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Ensure generated headers are included
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(../mason_hardware/include)

# Create shared library
# add_library(mason_navigation_lib SHARED 
#   ../mason_hardware/src/contact_sensors.cpp
#   src/navigation_node.cpp 
#   src/homing_sequence.cpp
# )

# Set include directories for the library
# target_include_directories(mason_navigation_lib PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
# )

# Link dependencies
# ament_target_dependencies(mason_navigation_lib
#     ${THIS_PACKAGE_INCLUDE_DEPENDS}
#     rosidl_default_runtime
#     mason_hardware
# )

find_library(GPIOD_LIBRARY gpiod)
if(NOT GPIOD_LIBRARY)
    message(FATAL_ERROR "libgpiod not found")
endif()

# Explicitly link against the generated message target
# target_link_libraries(mason_navigation_lib ${cpp_typesupport_target})

# Create executable
add_executable(service_node 
  ../mason_hardware/src/contact_sensors.cpp
  src/homing_sequence.cpp
)

ament_target_dependencies(service_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  rosidl_default_runtime
  mason_hardware
)

target_link_libraries(service_node ${cpp_typesupport_target})
target_link_libraries(service_node ${GPIOD_LIBRARY} gpiodcxx)

add_executable(navigation_node
  src/navigation_node.cpp
)

ament_target_dependencies(navigation_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  rosidl_default_runtime
)

target_link_libraries(navigation_node ${cpp_typesupport_target})

# Install targets
install(TARGETS
  service_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} rosidl_default_runtime)

ament_package()
