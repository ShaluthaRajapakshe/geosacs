#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from fp_core_msgs.msg import JointVelocity
import time
import numpy as np



class JointVelocityPublisher:
    def __init__(self):
        rospy.init_node('joint_velocity_publisher', anonymous=True)
        self.pub = rospy.Publisher('lio_1c/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Initial joint velocities
        self.joint_velocities = np.zeros(7)

        # Joint names
        self.joint_names = [
            'lio_1c_joint1',
            'lio_1c_joint2',
            'lio_1c_joint3',
            'lio_1c_joint4',
            'lio_1c_joint5',
            'lio_1c_joint6',
            'lio_1c_joint7'
        ]

    def update_joint_velocities(self, velocities):
        self.joint_velocities = velocities

    def publish_joint_states(self):
        while not rospy.is_shutdown():
            joint_state_msg = JointState()

            # Fill in the header
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.header.frame_id = "base"

            # Fill in the name and velocity arrays
            joint_state_msg.name = self.joint_names
            joint_state_msg.velocity = self.joint_velocities #.tolist()  # Convert to list if necessary

            # Leave position and effort fields empty
            joint_state_msg.position = []
            joint_state_msg.effort = []

            # Publish the message
            self.pub.publish(joint_state_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        joint_velocity_publisher = JointVelocityPublisher()

        # Example: Updating joint velocities dynamically
        # This can be replaced with actual logic to compute desired velocities
        while not rospy.is_shutdown():
            # Example velocities, update this logic based on your control algorithm
            example_velocities = [0,0,0,0,5,4,0]
            joint_velocity_publisher.update_joint_velocities(example_velocities)
            joint_velocity_publisher.publish_joint_states()
    except rospy.ROSInterruptException:
        pass