cmake_minimum_required(VERSION 3.0.2)
project(motor_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(

)

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(lib ${catkin_LIB_DIRS})

add_executable(motor_drive src/motor_drive.cpp src/can_drive.cpp)

target_link_libraries(motor_drive
  ${catkin_LIBRARIES}
  libcontrolcan.so
 )

