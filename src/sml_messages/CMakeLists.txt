cmake_minimum_required(VERSION 3.8)
project(sml_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Add this line

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Order.msg"
  "msg/Station.msg"
  "msg/Task.msg"
  DEPENDENCIES 
    std_msgs
    geometry_msgs  # Add this line
)

ament_package()