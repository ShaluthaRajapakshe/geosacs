#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist, TwistStamped
import transforms3d.quaternions as quat
import transforms3d.euler as euler
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState

class PoseControllerNode(): 
    def __init__(self):
        rospy.init_node("joy_controller") 

        # Variables
        self.freq = 20  #10 seems okay
        self.freq_rate = 5
        self.rate = rospy.Rate(self.freq_rate)  
        self.physical_robot = rospy.get_param("physical_robot")
        self.frame = "LIO_base_link"
        self.commandedPose = PoseStamped()
        self.commandedVel = Twist()
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.previous_ax = [0, 0, 0, 0, 0, 0, 0, 0]
        self.terminate = False
        self.start = False

        self.control_front = True

        self.cumulative_correction_distance = 0
        self.lio_cumulative_correction_distance = 0

        self.cumulative_correction_time = 0
        # self.current_time = 0

        self.correction = False

        # ROS Variables
        rospy.Subscriber("/joy_driver/commanded_vel", TwistStamped, self.commanded_vel_cb)
        rospy.Subscriber("/joy", Joy, self.joy_cb)

        if self.physical_robot: 
            rospy.Subscriber("/lio_1c/pose", PoseStamped, self.lio_pose_cb)

        self.pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=5)  #15
        self.myp_app_pub = rospy.Publisher("/myp_manager/app_control", String, queue_size=2)

        #Init 
        self.init_pose()
        rospy.sleep(1)
        rospy.loginfo("joy_controller has been started") 


    def lio_pose_cb(self,msg):
        # print("in pose cb")
        self.lio_pose = msg

    def joy_cb(self,msg):
        # Filter buttons input
        current_buttons = list(msg.buttons)

        # start_correction_time  = rospy.Time.now()

        ax = list(msg.axes) 

        if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
        else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
        self.previous_buttons = current_buttons

        if ax != [0, 0, 1, 0, 0, 1, 0, 0] or changed_buttons[4] !=0 or changed_buttons[5] !=0:
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


        if changed_buttons[1] != 0:
            if self.control_front:
                self.control_front = False
                rospy.loginfo("###### Mapping done considering the user is in rear side of the robot")
            else:
                self.control_front = True
                rospy.loginfo("###### Mapping done considering the user is in front side of the robot")
      
    
    
        if changed_buttons[7] != 0:
            self.terminate = True

        if changed_buttons[6] != 0:
            self.start = True
    
    def commanded_vel_cb(self, msg):
        self.commandedVel = msg.twist
    
    def init_pose(self):
        self.commandedPose.header.frame_id = self.frame

        # Coordinates in LIO_base_link frame
        self.commandedPose.pose.position.x=0.20695484883032408
        self.commandedPose.pose.position.y=0.0005657323485566697
        self.commandedPose.pose.position.z=1.061217401594137

        self.commandedPose.pose.orientation.x= 0.9982331952531064
        self.commandedPose.pose.orientation.y= -0.05193699424944531
        self.commandedPose.pose.orientation.z= -0.02800310197296562
        self.commandedPose.pose.orientation.w= 0.00699019334487028
    
    def run(self):
        self.first = True
        if self.physical_robot: self.myp_app_pub.publish("start")
        while not rospy.is_shutdown():

            

            if self.first:
                self.publish_commanded_pose()
                rospy.loginfo("Press START button on joystick ...")
                ####IN THE VERY FIRST ATTEMPT WE HAVE TO PRESS THE +z and -Z buttons

                if self.physical_robot: 
                    self.start_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])

                

                while not self.start:
                    rospy.sleep(0.1)
                rospy.loginfo("##### started ####")

                # print("im out")
                
                start_task_time = rospy.Time.now()
                rospy.sleep(0.1)
                self.first = False

                self.update_joy_pose()
                self.publish_commanded_pose()
                self.rate.sleep()

            else:
                # print ("here")
                while self.correction:
                    if self.physical_robot:
                        self.current_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])
                        lio_correction_distance = np.linalg.norm(self.current_position_lio - self.start_position_lio)
                        self.lio_cumulative_correction_distance += lio_correction_distance

                        self.start_position_lio = self.current_position_lio

                    correction_duration = (rospy.Time.now() - self.correction_start_time).to_sec()
                    self.cumulative_correction_time += correction_duration
                    self.correction_start_time = rospy.Time.now()
                    rospy.sleep(0.2)

                    self.update_joy_pose()
                    self.publish_commanded_pose()

                self.update_joy_pose()
                self.publish_commanded_pose()
                    


            if self.terminate: 
                if self.physical_robot: self.myp_app_pub.publish("stop")
                task_duration = (rospy.Time.now() - start_task_time).to_sec()
                rospy.loginfo(f"Total task time: {task_duration} seconds")
                rospy.loginfo(f"Total correction time: {self.cumulative_correction_time} seconds")

                rospy.loginfo(f"Total sim correction distance: {self.cumulative_correction_distance} meters")
                rospy.loginfo(f"Total Lio correction distance: {self.lio_cumulative_correction_distance} meters")

                rospy.loginfo(f"Correction time as a percentage from total: {(self.cumulative_correction_time/task_duration)*100}%)")

                

                ## Total correction time can be neasured by measrung the time when the uer presses any axis and z button on the joy, test it with the othe rjoystick

                # if self.physical_robot: rospy.loginfo(f"Total LIO correction distance: {self.lio_cumulative_correction_distance} meters")

                rospy.sleep(1)
                return
            
            self.rate.sleep()
            

    def update_joy_pose(self):

        PstartG = np.array([self.commandedPose.pose.position.x, self.commandedPose.pose.position.y, self.commandedPose.pose.position.z])

        if self.control_front:
            self.commandedPose.pose.position.x-=self.commandedVel.linear.x/self.freq
            self.commandedPose.pose.position.y-=self.commandedVel.linear.y/self.freq
            self.commandedPose.pose.position.z+=self.commandedVel.linear.z/self.freq
        
        else:

            self.commandedPose.pose.position.x+=self.commandedVel.linear.x/self.freq
            self.commandedPose.pose.position.y+=self.commandedVel.linear.y/self.freq
            self.commandedPose.pose.position.z+=self.commandedVel.linear.z/self.freq

        PnextG = np.array([self.commandedPose.pose.position.x, self.commandedPose.pose.position.y, self.commandedPose.pose.position.z])

        
        correction_distance = np.linalg.norm(PnextG - PstartG)

        

        self.cumulative_correction_distance += correction_distance

        # print("correction distance",  self.cumulative_correction_distance)

        # Quaternion initialization
        robot_rot = quat.quat2mat([self.commandedPose.pose.orientation.x, self.commandedPose.pose.orientation.y,
                                self.commandedPose.pose.orientation.z, self.commandedPose.pose.orientation.w])
        

        # print ("z value", self.commandedVel.angular.z)
        # 
        # Mapping fix to obtain same joystick control as drone_panda
        # Probably due to transforms3d library differences with C++ KDL library

        if self.control_front:
            ang_vel_x = -self.commandedVel.angular.z
            ang_vel_y = +self.commandedVel.angular.x
            ang_vel_z = self.commandedVel.angular.y

        else:
            ang_vel_x = self.commandedVel.angular.z
            ang_vel_y = -self.commandedVel.angular.x
            ang_vel_z = -self.commandedVel.angular.y

        # Rotation initialization using Roll-Pitch-Yaw (RPY)
        motion = euler.euler2mat(ang_vel_x/self.freq, ang_vel_y/self.freq, ang_vel_z/self.freq)

        # Quaternion multiplication and update
        result_quat = quat.mat2quat(np.dot(motion, robot_rot))
        self.commandedPose.pose.orientation.x, self.commandedPose.pose.orientation.y, self.commandedPose.pose.orientation.z, self.commandedPose.pose.orientation.w = result_quat

        # Quaternion normalization
        norm = np.linalg.norm([self.commandedPose.pose.orientation.x, self.commandedPose.pose.orientation.y,
                            self.commandedPose.pose.orientation.z, self.commandedPose.pose.orientation.w])
        if norm != 1:
            self.commandedPose.pose.orientation.x /= norm
            self.commandedPose.pose.orientation.y /= norm
            self.commandedPose.pose.orientation.z /= norm
            self.commandedPose.pose.orientation.w /= norm
    
    def publish_commanded_pose(self):
        self.commandedPose.header.stamp = rospy.Time.now()
        self.pose_pub.publish(self.commandedPose)




if __name__ == "__main__":
    my_node = PoseControllerNode()  
    my_node.run()