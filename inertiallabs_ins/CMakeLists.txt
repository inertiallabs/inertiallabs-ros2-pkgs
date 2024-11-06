cmake_minimum_required(VERSION 3.5)

project(inertiallabs_ins)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(inertiallabs_msgs REQUIRED)

set(DEPRECATION_FLAG "-Wsizeof-array-argument")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra ${DEPRECATION_FLAG}")

include_directories()

include_directories(${Opencv_INCLUDE_DIRS})
include_directories(${system_lib_INCLUDE_DIRS})

include_directories(../inertiallabs_sdk/)

if(WIN32)
    set(IL_PLATFORM "windows")
else()
    set(IL_PLATFORM "linux")
endif()

add_executable(il_ins
               src/il_ins.cpp
               ../inertiallabs_sdk/ILDriver.cpp
               ../inertiallabs_sdk/UDDParser.cpp
               ../inertiallabs_sdk/platforms/${IL_PLATFORM}/SerialPort.cpp
               ../inertiallabs_sdk/platforms/${IL_PLATFORM}/NetClient.cpp
)

ament_target_dependencies(il_ins
                          rclcpp
                          rclpy
                          sensor_msgs
                          geometry_msgs
                          tf2_ros
                          inertiallabs_msgs
)

if(NOT WIN32)
    target_link_libraries(il_ins  pthread)
endif()

install(TARGETS il_ins
    DESTINATION
        lib/${PROJECT_NAME})

ament_package()
