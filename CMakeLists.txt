cmake_minimum_required(VERSION 2.8.3)
project(gtec_uwb_ranging)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gtec_msgs
)


set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

catkin_package(
)


include_directories(src/uwbranging/usb)
include_directories(src/uwbranging/publishers)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


set(SOURCE_FILES_POZYX_RANGING src/uwbranging/publishers/node_pozyx_ranging.cpp
    src/uwbranging/usb/PozyxRangingReader.cpp
    src/uwbranging/usb/PozyxRangingReader.h
    src/uwbranging/usb/microbson.hpp)

add_executable(pozyxranging ${SOURCE_FILES_POZYX_RANGING})
add_dependencies(pozyxranging ${catkin_EXPORTED_TARGETS})
target_link_libraries(pozyxranging
   ${catkin_LIBRARIES}
 )
add_dependencies(pozyxranging gtec_msgs_generate_messages_cpp)

