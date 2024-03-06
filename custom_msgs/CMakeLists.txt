cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

# Find packages for message generation
find_package(catkin REQUIRED
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

# Generate services in the 'srv' folder
add_service_files( 
    FILES
    AppControl.srv 
    LoadData.srv
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
 include ${catkin_INCLUDE_DIRS}
)
