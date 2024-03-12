cmake_minimum_required(VERSION 3.0.2)
project(geosacs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  message_generation
  
)

# Generate messages in the 'msg' folder
add_message_files( 
    FILES
    WeightedPose.msg
    Correction.msg
    SurfacePose.msg
)

# Generate messages and services from dependencies 
generate_messages( 
    DEPENDENCIES
    geometry_msgs
    std_msgs

)
catkin_package( 
    CATKIN_DEPENDS 
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
