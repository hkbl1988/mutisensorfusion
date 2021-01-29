cmake_minimum_required(VERSION 2.8.3)
project(sensor)

#set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs	
  sensor_msgs
  message_generation
  geometry_msgs
)

add_message_files(
    FILES
    imu.msg
    gnss.msg
    odom.msg
    lidar.msg
    result.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
   message_runtime
   geometry_msgs
)

include_directories(include 
   ${catkin_INCLUDE_DIRS}
)  
