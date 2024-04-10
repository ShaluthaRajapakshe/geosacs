#!/usr/bin/env python
import rospy
from fp_core_msgs.srv import TestScript, TestScriptRequest
from std_msgs.msg import String


class myPApplication(): 
    def __init__(self):
        rospy.init_node("myp_application") 

        # Variables
        self.rate = rospy.Rate(10)  
        # Specify the script you want to run here
        self.script_path = "/home/shalutha/geosacs_ws/src/geosacs/lio_specific_pkg/scripts/move_joint.py"
        self.start = False

        # ROS Variables
        self.myP_script_client = rospy.ServiceProxy("/lio_1c/core/test_script", TestScript)
        rospy.Subscriber("/myp_manager/app_state", String, self.app_state_cb )

        #Init Message
        rospy.loginfo("myp_application has been started") 



    def app_state_cb(self, msg):
        if msg.data == "start":
            self.start = True
    
    def run(self):
      
      while not rospy.is_shutdown():
        
        if self.start:
            rospy.loginfo("Running myP application ...")
            request = TestScriptRequest()
            request.type = "main"
            with open( self.script_path,"r") as file:
                request.code = file.read()
            
            rospy.wait_for_service("/lio_1c/core/test_script")
            try:
                response = self.myP_script_client(request)
                rospy.loginfo(f"{response}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

            self.start = False
            rospy.loginfo("myP application stopped")

        
        self.rate.sleep()










        self.rate.sleep()
                  

if __name__ == "__main__":
    my_node = myPApplication()  
    my_node.run()