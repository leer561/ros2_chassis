cmake_minimum_required(VERSION 3.5)
project(ros2_chassis)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_linear_algebra REQUIRED)

find_package(Qt5Core REQUIRED)

include_directories(/home/leer/ecl_core/install/ecl_geometry/include)
include_directories(/home/leer/ecl_core/install/ecl_linear_algebra/include)
include_directories(/home/leer/ecl_core/install/ecl_exceptions/include)
include_directories(/home/leer/ecl_core/install/ecl_formatters/include)
include_directories(/home/leer/ecl_core/install/ecl_converters/include)
include_directories(/home/leer/ecl_core/install/ecl_type_traits/include)
include_directories(/home/leer/ecl_core/install/ecl_eigen/include)
include_directories(/home/leer/ecl_core/install/ecl_mpl/include)
include_directories(/home/leer/ecl_core/install/ecl_concepts/include)
include_directories(/home/leer/ecl_core/install/ecl_math/include)


include_directories(/home/leer/ecl_lite/install/ecl_config/include)
include_directories(/home/leer/ecl_lite/install/ecl_errors/include)

include_directories(/home/leer/sophus/install/sophus/include)


ament_package_xml()
ecl_enable_cxx14_compiler()


add_executable(base_driver src/base_driver.cpp src/serial-port/serial-port.cpp src/driver-node/cmd-driver.cpp src/driver-node/pub-driver.cpp)
ament_target_dependencies(base_driver rclcpp geometry_msgs std_msgs nav_msgs tf2 tf2_ros ecl_geometry ecl_linear_algebra ecl_config)

qt5_use_modules(base_driver Core SerialPort)

install(TARGETS
  base_driver
  DESTINATION lib/${PROJECT_NAME} 
)

ament_package()