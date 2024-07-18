#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class IKinterface(): 
    def __init__(self):
        rospy.init_node("ik_interface") 

        # Variables
        self.rate = rospy.Rate(12)
        self.velocities = [0, 0, 0, 0, 0, 0]

        # ROS Variables
        self.joint_states_sim_pub = rospy.Publisher("ik_interface/joint_states_sim", JointState, queue_size=10)
        rospy.Subscriber("/velocities", Float64MultiArray, self.velocities_cb)  # Subscriber for velocities

        #Init Message
        rospy.loginfo("ik_interface has been started") 

    def velocities_cb(self, msg):
        self.velocities = msg.data

    def run(self):
        while not rospy.is_shutdown():
            self.publish_simulation()
            self.rate.sleep()

    def publish_simulation(self): 
        # Prepare simulation message
        msg_sim = JointState()
        msg_sim.name = ["wheel_actuated_left_joint","wheel_actuated_right_joint","caster_left_wheel_joint","caster_left_base_joint",
                        "caster_right_wheel_joint","caster_right_base_joint","lio_1c_joint1", "lio_1c_joint2", "lio_1c_joint3", 
                        "lio_1c_joint4", "lio_1c_joint5", "lio_1c_joint6", "lio_1c_gripper_joint", "lio_1c_passive_joint"]
        msg_sim.position = [0 for _ in range(14)]  # Keeping positions zero
        msg_sim.velocity = [0 for _ in range(14)]

        # Use received velocities
        msg_sim.velocity[6:12] = self.velocities

        # Publish 
        msg_sim.header.stamp = rospy.Time.now()
        self.joint_states_sim_pub.publish(msg_sim)
        # rospy.loginfo("Published simulation joint states: %s", msg_sim)

if __name__ == "__main__":
    my_node = IKinterface()  
    my_node.run()
