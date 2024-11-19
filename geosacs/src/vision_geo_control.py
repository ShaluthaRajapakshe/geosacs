#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, PoseStamped
import torch
import numpy as np
import torch.nn as nn
from std_msgs.msg import String
from scipy.spatial.transform import Rotation 
import datetime

from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

import tf
import tf2_ros


    

class VisionGeoControlNode():
    def __init__(self):
        global task
        rospy.init_node("vision_geo_control")

        # Variables
        self.rate = rospy.Rate(12)  # Control frequency
  

        # self.lio_pose = None
        self.correction = False
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


        self.joystick_input = [0.0, 0.0]  # Initialize joystick input
        self.position_scale = 1  # Scale factor for joystick input

        self.physical_robot = rospy.get_param("physical_robot")
        

        self.terminate = False
        self.start = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


        # ROS Variables
        # self.joint_position_pub = rospy.Publisher("/pos_control/output", Float64MultiArray, queue_size=10)
        self.joint_position_pub = rospy.Publisher("/panda_ik/output", Float64MultiArray, queue_size=10)

        
        self.gripper_state_pub =rospy.Publisher("/gripper_state", String, queue_size=10)
        self.myp_app_pub = rospy.Publisher("/myp_manager/app_control", String, queue_size=2)
        self.task_end_pub =rospy.Publisher("/task_end", String, queue_size=10)
        self.commanded_pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=10)

        rospy.Subscriber("/joy", Joy, self.joy_cb)

        if self.physical_robot:
            print("Physical robot active")
            rospy.Subscriber("/lio_1c/joint_states", JointState, self.lio_joint_states_cb)
        # else:
        #     rospy.Subscriber("ik_interface/joint_states_sim", JointState, self.joint_states_sim_cb)
        
        self.initial_joint_positions = [-1.415177, 0.452273, 0.975495, -1.505162, 1.719274, -0.000463]
        
        
        # Init
        rospy.loginfo("vision_geo_control node has been started")

    

 

    def joy_cb(self, msg):
        # Map joystick axes to input
        self.joystick_input[0] = msg.axes[0] * self.position_scale  # Axis 0 controls joint1
        self.joystick_input[1] = msg.axes[1] * self.position_scale  # Axis 1 controls joint2
        # Other joystick inputs can be handled if necessary

       # Filter buttons input
        current_buttons = list(msg.buttons)
        if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
        else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
        self.previous_buttons = current_buttons

        if changed_buttons[0] != 0:
            self.gripper_state_pub.publish("toggle_gripper")
            self.gripper_change_request = True

        if changed_buttons[7] != 0:
            # print("STOP")
            self.terminate = True

        if changed_buttons[6] != 0:
            # print("START")
            self.start = True

        if changed_buttons[2] != 0:
            self.marsh_selected = True
            print("marsh_selected")
            

 

        if self.joystick_input[0] != 0.0 or self.joystick_input[1] != 0.0:
            # print("correction given")
            if not self.correction:
                self.correction_start_time = rospy.Time.now()
            self.correction = True
            return
        else:
            if self.correction and not self.first:
                correction_duration = (rospy.Time.now() - self.correction_start_time).to_sec()
                self.cumulative_correction_time += correction_duration
                self.correction_start_time = None
            self.correction = False

        

    # def joint_states_sim_cb(self, msg):
    #     self.lio_joint_positions = list(msg.position[6:12])


    def lio_joint_states_cb(self, msg):
        if self.physical_robot: 
            rospy.loginfo_once("**** Physical robot active ****")
            # self.joint_positions = list(msg.position[6:12])
            self.lio_joint_positions = list(msg.position)[0:6]
            # print("In here", self.joint_positions)



    def change_frame(self, position, orientation):

        # Define transform matrix
        angle =  1.570
        R_ab = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle) , 0],
                         [0            , 0             , 1]])
        p_ab = np.array([[0, 0, 0.266]]).reshape(3,1)
        _ = np.array([[0, 0, 0, 1]])
        T_ab = np.column_stack((R_ab, p_ab))
        T_ab = np.vstack((T_ab, _))

        # Get and check data
        positions_b = np.array(position)
        orientations_b = np.array(orientation)

        # Transform data into new frame
        
        positions_a = np.zeros(positions_b.shape)
        orientations_a = np.zeros(orientations_b.shape)

        
        pos_b = np.append(positions_b,1)
        pos_b = np.array([pos_b]).reshape(4,1)
        pos_a = np.dot(T_ab, pos_b)
        pos_a = np.ravel(pos_a)
        pos_a = pos_a[:-1]
        
        # Orientation
        q_b = orientations_b
        R_b = Rotation.from_quat([q_b[1], q_b[2], q_b[3], q_b[0]]).as_matrix()
        R_a = np.dot(R_ab, R_b)
        q_a = Rotation.from_matrix(R_a).as_quat()
        q_a = np.array([q_a[3],q_a[0], q_a[1], q_a[2]])
        orientations_a = q_a

        
        return pos_a, orientations_a   
    

    def pub_cmd_pose(self, PcurrG, PoriG):
        msg1 = PoseStamped()
        msg1.header.frame_id = "LIO_base_link"
        msg1.pose.position.x = PcurrG[0]
        msg1.pose.position.y = PcurrG[1]
        msg1.pose.position.z = PcurrG[2]

        msg1.pose.orientation.w= PoriG[0]
        msg1.pose.orientation.x= PoriG[1]
        msg1.pose.orientation.y= PoriG[2]
        msg1.pose.orientation.z= PoriG[3]
        msg1.header.stamp = rospy.Time.now()
        self.commanded_pose_pub.publish(msg1)


    def generate_spline_trajectory(self, start_pos, end_pos, start_orientation, end_orientation, num_waypoints=100):
        # Create a time parameterization for the spline (0 to 1)
        t = np.linspace(0, 1, num_waypoints)
        
        # Define waypoints for position
        waypoints = np.vstack((start_pos, end_pos)).T  # Transpose to get shape (3, 2)

        # Create cubic spline for each dimension
        cs_x = CubicSpline([0, 1], waypoints[0])  # Spline for x-axis
        cs_y = CubicSpline([0, 1], waypoints[1])  # Spline for y-axis
        cs_z = CubicSpline([0, 1], waypoints[2])  # Spline for z-axis

        # Generate the interpolated positions
        interpolated_positions = np.vstack((cs_x(t), cs_y(t), cs_z(t))).T

        # Create rotation objects
        start_rot = R.from_quat(start_orientation)
        end_rot = R.from_quat(end_orientation)


        # Create a rotation array for SLERP
        rotations = R.from_quat([start_orientation, end_orientation])

        # Create Slerp object
        slerp = Slerp([0, 1], rotations)

        # # Create Slerp object
        # slerp = Slerp([0, 1], [start_rot, end_rot])

        # Interpolate orientations
        interpolated_orientations = slerp(t).as_quat()

        return interpolated_positions, interpolated_orientations




    def run(self):

        if self.physical_robot: self.myp_app_pub.publish("start")
        # self.task_end_pub.publish("True")

        self.start = True # this is required as a single published value will not move the robot. Just to publish the same goal 
        ## iteratively we use this. Otherthan that we can remove this
        
        while not rospy.is_shutdown():




            ''' 1) Function to set the current pose and goal poses -> for current pose we can utilize the TF tree: current pose and goal 
            pose should be in the same frame '''

            transform = self.tfBuffer.lookup_transform('LIO_robot_base_link', 'lio_tcp_link', rospy.Time(0), rospy.Duration(1.0))

            # print(transform)
                         
            self.current_position_lio = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            self.current_orientation_lio = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])


            if self.start:
                self.goal_position_lio = np.array([transform.transform.translation.x + 0.3, transform.transform.translation.y, transform.transform.translation.z - 0.3])
                self.goal_orientation_lio = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
                self.start = False
          
            

            ''' 2) Function to take those start and goal poses and generate the trajectory using a spline based method '''

            self.waypoints, self.orientations = self.generate_spline_trajectory(
                self.current_position_lio, 
                self.goal_position_lio,
                self.current_orientation_lio,
                self.goal_orientation_lio, 
                num_waypoints=100
            )

            


            ''' 3) Function to visualize the trajectory '''




            ''' 4) Function to send the trajectory points to the IK engine move the robot '''

            # self.pub_cmd_pose(self.goal_position_lio, self.goal_orientation_lio)
            # print("Goal position",  self.goal_position_lio)

            # Publish each waypoint and corresponding orientation in the trajectory
            for position, orientation in zip(self.waypoints, self.orientations):
                self.pub_cmd_pose(position, orientation)  # Publish the interpolated position and orientation
                self.rate.sleep()  # Sleep to control the frequency of command publishing


            self.rate.sleep()
            return

            
            





if __name__ == "__main__":
    my_node = VisionGeoControlNode()
    my_node.run()
