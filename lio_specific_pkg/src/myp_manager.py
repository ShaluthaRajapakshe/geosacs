#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class myPManager(): 
    def __init__(self):
        rospy.init_node("myp_manager") 

        # Variables
        self.rate = rospy.Rate(10)  
        self.start = False
        self.running = False
        self.stop = False
        self.state = ""

        # ROS Variables
        rospy.Subscriber("/myp_manager/app_control", String, self.app_control)
        self.app_state_pub = rospy.Publisher("/myp_manager/app_state", String, queue_size=10)

        #Init Message
        rospy.loginfo("myP_manager has been started") 

    def app_control(self, msg):
        if msg.data == "start":
            self.state = "start"
            rospy.loginfo("Start myP app.")
        elif msg.data == "stop":
            self.state = "stop"
            rospy.loginfo("Stop myP app.")
    
    def run(self):
        while not rospy.is_shutdown():
            
            if self.state == "start":
                self.app_state_pub.publish("start")
                self.state = "running"
            elif self.state == "running":
                self.app_state_pub.publish("running")
            elif self.state == "stop":
                self.app_state_pub.publish("stop")
                self.state = ""

            self.rate.sleep()

if __name__ == "__main__":
    my_node = myPManager()  
    my_node.run()