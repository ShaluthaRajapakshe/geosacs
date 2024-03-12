#!/usr/bin/env python
import rospy
from fp_core_msgs.srv import GetStatus
from std_msgs.msg import String
from std_msgs.msg import Float64

class ServiceToTopicNode(object): 
    def __init__(self):
        rospy.init_node("service_to_topic") 

        # Variables
        self.rate = rospy.Rate(10)
        self.released = None # Added to counter myP bug after 1.7.1 update
        self.get_status_client = rospy.ServiceProxy("/lio_1c/core/get_status", GetStatus)
        self.status_pub = rospy.Publisher("/service2topic/status", String, queue_size=10)
        #self.gripper_angle_pub = rospy.Publisher("/service2topic/gripper_angle", Float64, queue_size=10)

        #Init Message
        rospy.loginfo("service_to_topic node has been started") 

    def run(self):
        while not rospy.is_shutdown():
            status = self.call_get_status()
            self.publish_status(status)
    
    def call_get_status(self):
        try:
            response = self.get_status_client()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        if response.status == 9:
            self.released = True
            return "RELEASED"
        elif response.status == 0 and self.released:
            return "RELEASED"
        else:
            self.released = False
            return f"{response.status}"
    
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


if __name__ == "__main__":
    my_node = ServiceToTopicNode() #
    my_node.run()