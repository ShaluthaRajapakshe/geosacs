#!/usr/bin/env python3
import rospy
import math 
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray, String

class IKinterface(): 
    def __init__(self):
        rospy.init_node("ik_interface") 

        # Variables
        self.rate = rospy.Rate(12)
        self.ik_js = None
        self.gripper_angle = 0
        self.physical_robot = rospy.get_param("physical_robot")
        self.robot_js = None
        self.gripper_closed = True
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.count = 1

        # ROS Variables
        self.joint_states_lio_pub = rospy.Publisher("ik_interface/joint_states_lio", JointState, queue_size=10)
        self.joint_states_sim_pub = rospy.Publisher("ik_interface/joint_states_sim", JointState, queue_size=10)


        rospy.Subscriber("/gripper_state", String, self.gripper_state_cb)
        
        rospy.Subscriber("/panda_ik/output", Float64MultiArray, self.ik_cb)
        # rospy.Subscriber("/pos_control/output", Float64MultiArray, self.ik_cb)  #use this for LLA work

        rospy.Subscriber("/lio_1c/joint_states", JointState, self.lio_joint_states_cb)

        #Init Message
        rospy.loginfo("ik_interface has been started") 



    def gripper_state_cb(self, msg):
        if msg.data == "toggle_gripper":
            if self.gripper_angle == 0 : self.gripper_angle = 0.523594
            else : self.gripper_angle = 0

    def lio_joint_states_cb(self, msg):
        if self.physical_robot: 
            rospy.loginfo_once("**** Physical robot active ****")
            self.robot_js = msg.position
    
    def run(self):
        while not rospy.is_shutdown():

            if self.ik_js == None : continue # Wait for data
            ik_joint_states = self.ik_js
        
            if self.physical_robot:
                if self.robot_js == None : continue # Wait for data
                robot_joint_states = self.robot_js

                self.publish_simulation(robot_joint_states)
                self.publish_robot(ik_joint_states)
            else:
                self.publish_simulation(ik_joint_states)
            
            self.rate.sleep()
    

    # def joy_cb(self, msg):
    #     changed_buttons = self.filter_joy_buttons(msg)
    #     if changed_buttons[0]==1 : 
    #         if self.gripper_angle == 0 : self.gripper_angle = 0.523594
    #         else : self.gripper_angle = 0


    # def filter_joy_buttons(self, msg):
    #     current_buttons = list(msg.buttons)
    #     if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
    #     else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]

    #     self.previous_buttons = current_buttons

    #     return changed_buttons

    
    def ik_cb(self, msg):
        self.ik_js = msg.data
        # print("received ik_js: ", self.ik_js)



    def publish_simulation(self, joint_states): 
        # Extract information
        arm = list(joint_states)

        # Prepare simulation message
        msg_sim = JointState()
        msg_sim.name = ["wheel_actuated_left_joint","wheel_actuated_right_joint","caster_left_wheel_joint","caster_left_base_joint",
                        "caster_right_wheel_joint","caster_right_base_joint","lio_1c_joint1", "lio_1c_joint2", "lio_1c_joint3", 
                        "lio_1c_joint4", "lio_1c_joint5", "lio_1c_joint6", "lio_1c_gripper_joint", "lio_1c_passive_joint"]
        msg_sim.position = [0 for _ in range(14)]
        
        if len(arm) == 6:
            # Joint states from ik
            msg_sim.position[12] = self.gripper_angle
            msg_sim.position[13] = -self.gripper_angle
            msg_sim.position[6:12] = arm
        else:
            # Joint states from physical robot
            msg_sim.position[12] = arm[6]
            msg_sim.position[13] = -arm[6]
            msg_sim.position[6:12] = arm[:6]

        # Publish 
        msg_sim.header.stamp = rospy.Time.now()
        self.joint_states_sim_pub.publish(msg_sim)
    
    def publish_robot(self, joint_states): 
        # Extract information
        arm = joint_states[:6]

        # Prepare lio message
        msg_lio = JointState()
        msg_lio.name = ["lio_1c_joint1", "lio_1c_joint2", "lio_1c_joint3", "lio_1c_joint4", 
                        "lio_1c_joint5", "lio_1c_joint6", "gripper"]
        msg_lio.position = [0 for _ in range(6)]
        # Convert to degrees for lio
        msg_lio.position = [math.degrees(radian) for radian in arm]
        msg_lio.position.append(math.degrees(self.gripper_angle))

        # Publish 
        msg_lio.header.stamp = rospy.Time.now()
        self.joint_states_lio_pub.publish(msg_lio)

if __name__ == "__main__":
    my_node = IKinterface()  
    my_node.run()