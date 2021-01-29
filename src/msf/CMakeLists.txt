cmake_minimum_required(VERSION 2.8.3)
project(msfusion)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(catkin REQUIRED COMPONENTS
  sensor
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  geometry_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cmn)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/fusion)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/integ)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/process)


catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
   tf
   sensor_msgs
   sensor
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_executable(msfusion 
  #lib/mutifusion.cc
  lib/tranmerc.cc
  lib/utm.cc
  src/base/appfusion.cc
  src/fusion/fusion.cc
  src/fusion/localfusion.cc
  src/integ/processinteg.cc
  src/integ/threadinteg.cc
  src/process/preprocess.cc
  src/main.cc
)
add_dependencies(msfusion 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#add_library(msf SHARED lib/mutifusion.cc)

target_link_libraries(msfusion  ${catkin_LIBRARIES} 
                                ${PROJECT_SOURCE_DIR}/lib/libmsf.so)

target_link_libraries(msfusion  ${catkin_LIBRARIES} )

