cmake_minimum_required(VERSION 3.8)
project(mason_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  std_srvs
  rmw
)

# Add the library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/mason_interface.cpp
  src/motor.cpp
  src/contact_sensors.cpp
)

# Include directories
target_include_directories(mason_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mason_hardware>
)
target_link_directories(mason_hardware PUBLIC /usr/lib)

# Link against WiringPi
# target_link_libraries(mason_hardware PUBLIC wiringPi)

# Add ament dependencies
ament_target_dependencies(mason_hardware PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface mason.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)


# Enable testing if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
