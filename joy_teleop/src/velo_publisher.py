#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class PoseControllerNode(): 
    def __init__(self):
        rospy.init_node("joy_controller") 

        # Variables
        self.rate = rospy.Rate(20)  # Control frequency
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initialize velocities for six joints
        self.velocity_scale = 10.0  # Scale factor for joystick input

        # ROS Variables
        self.joint_velocity_pub = rospy.Publisher("/velocities", Float64MultiArray, queue_size=10)
        rospy.Subscriber("/joy", Joy, self.joy_cb)

        # Init 
        rospy.loginfo("joy_controller has been started") 

    def joy_cb(self, msg):
        # Map joystick axes to joint velocities
        self.joint_velocities[0] = msg.axes[0] * self.velocity_scale  # Axis 0 controls joint1
        self.joint_velocities[1] = msg.axes[1] * self.velocity_scale  # Axis 1 controls joint2
        # Other joint velocities remain zero

    def run(self):
        while not rospy.is_shutdown():
            self.publish_joint_velocities()
            self.rate.sleep()
            
    def publish_joint_velocities(self):
        # Prepare Float64MultiArray message
        msg = Float64MultiArray()
        msg.data = self.joint_velocities

        # Publish the velocities
        self.joint_velocity_pub.publish(msg)
        rospy.loginfo("Published joint velocities: %s", msg.data)

if __name__ == "__main__":
    my_node = PoseControllerNode()  
    my_node.run()
