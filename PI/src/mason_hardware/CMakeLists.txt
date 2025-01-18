cmake_minimum_required(VERSION 3.8)
project(mason_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++20 -Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  geometry_msgs
  rmw
  rclcpp_components
  # rosidl_default_generators
)

# set(interface_files
#   # Message files
#   "msg/HomingMsg.msg"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
# ${interface_files}
# )

# find_library(GPIOD_LIBRARY gpiod)
# if(NOT GPIOD_LIBRARY)
#     message(FATAL_ERROR "libgpiod not found")
# endif()

# Add the library
add_library(mason_hardware SHARED src/mason_interface.cpp src/motor.cpp) # src/contact_sensors.cpp)
# target_include_directories(mason_hardware PUBLIC /usr/include)
# target_link_libraries(${PROJECT_NAME} ${GPIOD_LIBRARY})

# Link any dependencies
ament_target_dependencies(mason_hardware
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface mason.xml)

# Install the shared library
install(
    TARGETS mason_hardware
    EXPORT export_mason_hardware
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install plugin XML description
install(
    FILES mason.xml
    DESTINATION share/mason_hardware
)

install(
  DIRECTORY include/
  DESTINATION include/mason_hardware
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

# Finalize the package
ament_package()
# #_____________________________________________________________________________________
# cmake_minimum_required(VERSION 3.8)
# project(mason_hardware)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-std=c++20 -Wall -Wextra -Wpedantic)
# endif()

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# # Declare dependencies
# set(THIS_PACKAGE_INCLUDE_DEPENDS
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   rcpputils
#   realtime_tools
#   std_srvs
#   rmw
#   rosidl_default_generators
# )

# # Declare the message files
# set(interface_files
#   msg/HomingMsg.msg
# )

# # Generate the message interfaces
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${interface_files}
# )

# # Add the library
# add_library(mason_hardware SHARED src/mason_interface.cpp src/motor.cpp)

# # Link any dependencies
# ament_target_dependencies(mason_hardware
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# # Link generated message typesupport to the library
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(mason_hardware ${cpp_typesupport_target})

# # Export the plugin
# pluginlib_export_plugin_description_file(hardware_interface mason.xml)

# # Install the shared library
# install(
#     TARGETS mason_hardware
#     EXPORT export_mason_hardware
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     RUNTIME DESTINATION bin
# )

# # Install plugin XML description
# install(
#     FILES mason.xml
#     DESTINATION share/mason_hardware
# )

# # Install message files
# install(
#   DIRECTORY msg
#   DESTINATION share/${PROJECT_NAME}
# )

# install(
#   DIRECTORY include/
#   DESTINATION include/mason_hardware
# )

# # Enable testing if required
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# # Export package dependencies
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# # Finalize the package
# ament_package()
