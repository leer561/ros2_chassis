cmake_minimum_required(VERSION 3.5)
project(ros2_chassis)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Qt5Core REQUIRED)

add_executable(base_driver src/base_driver.cpp src/serial/serialport.cpp)
ament_target_dependencies(base_driver rclcpp geometry_msgs std_msgs)

qt5_use_modules(base_driver Core SerialPort)

install(TARGETS
  base_driver
  DESTINATION lib/${PROJECT_NAME} 
)

ament_package()