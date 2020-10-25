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
  geometry_msgs
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
include_directories(src/uwbranging/python)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES_POZYX_RANGING src/uwbranging/publishers/node_pozyx_ranging.cpp
    src/uwbranging/usb/PozyxRangingReader.cpp
    src/uwbranging/usb/PozyxRangingReader.h
    src/uwbranging/usb/microbson.hpp)

set(SOURCE_FILES_POZYX_IMU_RANGING src/uwbranging/publishers/node_pozyx_imu_ranging.cpp
    src/uwbranging/usb/PozyxRangingIMUReader.cpp
    src/uwbranging/usb/PozyxRangingIMUReader.h
    src/uwbranging/usb/microbson.hpp)

set(SOURCE_FILES_POZYX_CIR_RANGING src/uwbranging/publishers/node_pozyx_cir_ranging.cpp
    src/uwbranging/usb/PozyxRangingCIRReader.cpp
    src/uwbranging/usb/PozyxRangingCIRReader.h
    src/uwbranging/usb/microbson.hpp)

set(SOURCE_FILES_POZYX_POSITION src/uwbranging/publishers/node_pozyx_position.cpp
    src/uwbranging/usb/PozyxPositionReader.cpp
    src/uwbranging/usb/PozyxPositionReader.h
    src/uwbranging/usb/microbson.hpp)

add_executable(pozyxranging ${SOURCE_FILES_POZYX_RANGING})
add_dependencies(pozyxranging ${catkin_EXPORTED_TARGETS})
target_link_libraries(pozyxranging
   ${catkin_LIBRARIES}
 )
add_dependencies(pozyxranging gtec_msgs_generate_messages_cpp)



add_executable(pozyxrangingimu ${SOURCE_FILES_POZYX_IMU_RANGING})
add_dependencies(pozyxrangingimu ${catkin_EXPORTED_TARGETS})
target_link_libraries(pozyxrangingimu
   ${catkin_LIBRARIES}
 )
add_dependencies(pozyxrangingimu gtec_msgs_generate_messages_cpp)

add_executable(pozyxrangingcir ${SOURCE_FILES_POZYX_CIR_RANGING})
add_dependencies(pozyxrangingcir ${catkin_EXPORTED_TARGETS})
target_link_libraries(pozyxrangingcir
   ${catkin_LIBRARIES}
 )
add_dependencies(pozyxrangingcir gtec_msgs_generate_messages_cpp)

add_executable(pozyxposition ${SOURCE_FILES_POZYX_POSITION})
add_dependencies(pozyxposition ${catkin_EXPORTED_TARGETS})
target_link_libraries(pozyxposition
   ${catkin_LIBRARIES}
 )
add_dependencies(pozyxposition gtec_msgs_generate_messages_cpp)