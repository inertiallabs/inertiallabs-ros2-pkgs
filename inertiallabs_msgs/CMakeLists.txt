cmake_minimum_required(VERSION 3.5)
project(inertiallabs_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/GnssData.msg
        msg/GpsData.msg
        msg/InsData.msg
        msg/MarineData.msg
        msg/SensorData.msg
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

ament_package()
