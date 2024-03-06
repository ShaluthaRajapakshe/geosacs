# Platform Descriptions

Created: 11th February 2018 by PDU

This package contains the URDF models of mobile platforms.

## Configs
As there are several versions of mobile platform, configs files are used to know the characteristics of each of them.
They should contains the all relevant informations and a sensor section for the information about the sensors (transforms, topic names, etc...)

Each sensor should have the following entries:
- name: name of the sensor (ex: terarangerHub1, lidar_front).
- portname: name of the port the sensor is connected to (ex: , /dev/flashlidar_front).
- name_for_costmap: list of sensor entries in the costmap (ex: [terarangerHub01, terarangerHub11, terarangerHub21, terarangerHub31, terarangerHub41, terarangerHub51, terarangerHub61, terarangerHub71] )
- type_for_costmap: list of sensor type in the costmap (ex: [LaserScan, LaserScan, LaserScan, LaserScan, LaserScan, LaserScan, LaserScan, LaserScan])
- topic_for_costmap: list of rostopic the costnamp should subscribe to (ex:['/Sensors/Hub01', '/Sensors/Hub11', '/Sensors/Hub21', '/Sensors/Hub31', '/Sensors/Hub41', '/Sensors/Hub51', '/Sensors/Hub61', '/Sensors/Hub71'])
- marking: list of boolean to determine if each sensor should be added to the costmap or not (ex:[true, true, false, true, true, true, true, true])
- clearing: list of boolean to determine if each sensor should be cleared on higeher readings (ex: [true, true, true, false, true, true, false, true])
- expected_update_rate: list of expected update rate (inf if 0) (ex: [0, 0, 0, 0, 0, 0, 0, 0])
- offset_x: list of x distance from base_footprint located in the middle of the front wheels (ex: [0, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
- offset_y: list of y distance from base_footprint located in the middle of the front wheels (ex: [0.32, 0.32, 0.25, 0.15, 0.05, -0.05, -0.15, -0.25])
- offset_z: z distance from base_footprint located in the middle of the front wheels (ex:  -0.03)
- offset_roll: list of roll angle from base_footprint located in the middle of the front wheels (ex: [0, 0, 0, 0, 0, 0, 0, 0])
- offset_pitch: list of pitch angle from base_footprint located in the middle of the front wheels (ex: [0, 0, 0, 0, 0, 0, 0, 0])
- offset_yaw: list of yaw angle from base_footprint located in the middle of the front wheels (ex: [1.57, 1.57, 0, 0, 0, 0, 0, 0])


All the files should be in the config folder, named ROBOT_NAME.yaml.

## URDF
The URDF file will load the config of the corresponding robots to get the transforms information. The other informations are used by the mobile_platform pakage.
For a new URDF file, name it platform_{ROBOT_NAME}.urdf.xacro and add the base link. Then  load all the xacros (arm, pgrip, sensors, cameras etc...)

###### Camera xacros
They are located in p_camera_description under ${camera_name}.urdf.xacro. So far r200.urdf.xacro and  fisheye.urdf.xacro.
###### Sensors xacro
They are located in platform_description under sensors.urdf.xacro. It is possible to create:
- terarangers: 1 infrared sensor given a name, parent, number, origin (x, y, z, roll, pitch, yaw)
- teraranger chains: chain of 8 infrared sensors given a name, parent, 8 origins
- floor_sensors: 1 floor sensor given a name, parent, number, origin (x, y, z, roll, pitch, yaw)
- ultrasonic_sensors: 1 ultrasonic sensor given a name, parent, number, origin (x, y, z, roll, pitch, yaw)

They create the link ${name}_sensor_${number}
