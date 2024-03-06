cmake_minimum_required(VERSION 2.8.3)
project(fp_core_msgs)

# find message_generation library
find_package(catkin REQUIRED
  geometry_msgs
  message_generation
)

# generate messages in the 'msg' folder
add_message_files(
  FILES
  AnalogIO.msg
  DigitalIO.msg
  JointAngles.msg
  JointCurrent.msg
  JointPosition.msg
  JointVelocity.msg
  NauticalAngles.msg
  Pose.msg
  PoseAlternative.msg
  Sensors.msg
)

# generate services in the 'srv' folder
add_service_files(
  FILES
  ForwardKinematics.srv
  GenericDataService.srv
  GetGridPoints.srv
  GetListBool.srv
  GetListFloat.srv
  GetPose.srv
  GetStatus.srv
  InformationPoolGet.srv
  InverseKinematics.srv
  IsMotorMoving.srv
  ListInt.srv
  MoveJoint.srv
  MoveTool.srv
  PowerUp.srv
  PowerUpAndCalibrate.srv
  ReadGripperAngle.srv
  ReadSensorData.srv
  Reconfigure.srv
  RequestData.srv
  SetInt.srv
  SetString.srv
  SimplePath.srv
  TestScript.srv
  TokenLogin.srv
  WriteAnalogIO.srv
  WriteDigitalIO.srv
)

# generate added messages and services
generate_messages(
  DEPENDENCIES
  geometry_msgs
)
catkin_package(
CATKIN_DEPENDS message_runtime geometry_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}	
)
