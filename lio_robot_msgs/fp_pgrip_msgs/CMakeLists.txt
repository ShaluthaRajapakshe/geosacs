cmake_minimum_required(VERSION 2.8.3)
project(fp_pgrip_msgs)

# find message_generation library
find_package(catkin REQUIRED
  message_generation
)

# generate services in the 'srv' folder
add_service_files(
  FILES
  ToggleGripper.srv
  ReadGripperAngle.srv
  ReadSensorData.srv
)

# generate added messages and services
generate_messages()

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
