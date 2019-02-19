cmake_minimum_required(VERSION 2.6)
project(gymfc_digitaltwin)

find_package(Boost 1.58 REQUIRED COMPONENTS system thread timer)
find_package(gazebo REQUIRED)

#
# Find Protobut
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message("Gazebo version: ${GAZEBO_VERSION}")
endif()
#
# End Find Protobut
#



find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
    include 
  ${CMAKE_CURRENT_BINARY_DIR}
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
    ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS ${GAZEBO_CXX_FLAGS})

#--------------------#
# Begin Message Generation #
#--------------------#

set(control_msgs
    msgs/MotorCommand.proto
  )
set(sensor_msgs
  msgs/Float.proto
  msgs/Imu.proto
  msgs/MotorSpeed.proto
  msgs/EscSensor.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  )

PROTOBUF_GENERATE_CPP(CTRL_PROTO_SRCS CTRL_PROTO_HDRS ${control_msgs})
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

add_library(control_msgs SHARED ${CTRL_PROTO_SRCS})
add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})

#--------------------#
# End Message Generation #
#--------------------#

link_libraries(control_msgs sensor_msgs)

add_library(gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
add_library(gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)

set(plugins
    gazebo_motor_model
    gazebo_imu_plugin
    )
foreach(plugin ${plugins})
  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})
endforeach()


