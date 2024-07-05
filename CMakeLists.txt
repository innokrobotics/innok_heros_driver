cmake_minimum_required(VERSION 3.5)
project(innok_heros_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostics REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(innok_heros_can_driver src/innok_heros_can_driver.cpp)

ament_target_dependencies(
  innok_heros_can_driver 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  sensor_msgs
  tf2
  tf2_ros
  )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  innok_heros_can_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/battery_watchdog
  scripts/can_bms_node
  scripts/heros_diagnostics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
