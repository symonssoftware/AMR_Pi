cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(radio_link_publisher src/radio_link_publisher.cpp)
ament_target_dependencies(radio_link_publisher rclcpp my_robot_interfaces)

add_executable(arduino_serial src/arduino_serial.cpp)
ament_target_dependencies(arduino_serial rclcpp my_robot_interfaces example_interfaces)

add_executable(ctre src/ctre.cpp)
ament_target_dependencies(ctre rclcpp my_robot_interfaces example_interfaces)

set(PHOENIX_LIBS_DIR ${PROJECT_SOURCE_DIR}/lib)

target_link_libraries(ctre ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(ctre ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)

install(TARGETS
    ctre
    radio_link_publisher
    arduino_serial
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
