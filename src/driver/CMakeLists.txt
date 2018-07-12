cmake_minimum_required(VERSION 3.5)
project(ros2_talker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(talker src/lambda.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(cam2image
  src/burger.cpp
  src/cam2image.cpp
  src/options.cpp
)
ament_target_dependencies(cam2image
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV")

  add_executable(show_image
  src/options.cpp
  src/show_image.cpp
)
ament_target_dependencies(show_image
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV")  

# add_executable(publisher_not_composable not_composable.cpp)
# ament_target_dependencies(publisher_not_composable rclcpp std_msgs)

install(TARGETS
  talker
  subscriber
  cam2image
  show_image
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()