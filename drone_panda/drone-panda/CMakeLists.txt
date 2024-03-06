cmake_minimum_required(VERSION 2.8.3)
project(drone_panda)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS  "-O3 -fopenmp")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf 
        cv_bridge 
        image_transport 
        visualization_msgs
        image_geometry 
        rviz_camera_stream
        sensor_msgs
        pcl_conversions
        pcl_ros
        drone_ros_msgs
	rviz
        )

## System dependencies
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_library(DHDC libdhd.so.3.8.0)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
if(2.9.0 VERSION_GREATER ${OpenCV_VERSION})
    message(FATAL "OpenCV3 required!")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS 
    tf
    sensor_msgs
  DEPENDS OpenCV
  LIBRARIES 
)



###########
## Build ##
###########

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

message(STATUS test)
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

if(DHDC)
  add_executable(fd src/ForceDimension.cpp)
  target_include_directories(fd PUBLIC adept${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} {CMAKE_CURRENT_SOURCE_DIR}/include /home/senft/catkin_ws/devel/include)
  target_link_libraries(fd
     ${catkin_LIBRARIES}
     ${DHDC}
  )
endif()

set(SOURCE_FILES src/dronepanel.cpp)
add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_executable(cloudplane src/PCLPlanes.cpp)
target_link_libraries(cloudplane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloudfilter src/cloudFilter.cpp)
target_link_libraries(cloudfilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY
    config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY
    rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY
    meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
install(DIRECTORY
    urdfs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
)

catkin_install_python(PROGRAMS
    nodes/simulator
    nodes/static_publisher
    nodes/virtual_drone_manager
    nodes/pc_publish
    nodes/rviz_manager
    nodes/drone_controller
    nodes/tello_driver
    nodes/tello_driver_launcher
    nodes/velocity_controller
    nodes/joy_driver
    nodes/spacemouse_driver
    nodes/map_odometry
    nodes/env_mapping
    nodes/pc_record
    nodes/pcl_plane_to_bounding
    nodes/alternate_view_manager
    nodes/autonomous_mapping
    nodes/torque_filter
    nodes/path_recorder
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(DHDC)
  install(TARGETS fd
    RUNTIME DESTINATION lib/fd
  )
endif()

