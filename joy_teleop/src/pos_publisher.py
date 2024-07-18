#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class PoseControllerNodePure(): 
    def __init__(self):
        rospy.init_node("joy_controller") 

        # Variables
        self.rate = rospy.Rate(20)  # Control frequency
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initialize positions for six joints
        self.position_scale = 0.005  # Scale factor for joystick input

        # ROS Variables
        self.joint_position_pub = rospy.Publisher("/pos_control/output", Float64MultiArray, queue_size=10)
        rospy.Subscriber("/joy", Joy, self.joy_cb)

        # Init 
        rospy.loginfo("joy_controller has been started") 

    def joy_cb(self, msg):
        # Map joystick axes to joint positions
        self.joint_positions[0] += msg.axes[0] * self.position_scale  # Axis 0 controls joint1
        self.joint_positions[1] += msg.axes[1] * self.position_scale  # Axis 1 controls joint2
        # Other joint positions remain zero or can be controlled by other joystick axes

    def run(self):
        while not rospy.is_shutdown():
            self.publish_joint_positions()
            self.rate.sleep()
            
    def publish_joint_positions(self):
        # Prepare Float64MultiArray message
        msg = Float64MultiArray()
        msg.data = self.joint_positions

        # Publish the positions
        self.joint_position_pub.publish(msg)
        rospy.loginfo("Published joint positions: %s", msg.data)

if __name__ == "__main__":
    my_node = PoseControllerNode()  
    my_node.run()
