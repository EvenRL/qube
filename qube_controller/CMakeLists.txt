cmake_minimum_required(VERSION 3.8)
project(qube_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(qube_controller_node src/qube_controller_node.cpp)
ament_target_dependencies(qube_controller_node
        rclcpp
        sensor_msgs
        std_msgs
)

install(TARGETS qube_controller_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()