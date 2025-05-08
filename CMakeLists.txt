# https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html
# https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

cmake_minimum_required(VERSION 3.8)
project(rosbot)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(rosbot SHARED
  src/rosbot_system.cpp
)

target_include_directories(rosbot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(rosbot PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface rosbot.xml)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS rosbot
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies()

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_load_rosbot test/test_load_rosbot.cpp)
  ament_target_dependencies(test_load_rosbot hardware_interface)

endif()

ament_package()
