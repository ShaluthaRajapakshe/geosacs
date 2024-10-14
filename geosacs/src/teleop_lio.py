#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist, TwistStamped
import transforms3d.quaternions as quat
import transforms3d.euler as euler
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class PoseControllerNode(): 
    def __init__(self):
        rospy.init_node("lio_teleoperation_node") 

        # Variables
        self.freq = 100
        self.rate = rospy.Rate(self.freq)  
        self.physical_robot = rospy.get_param("physical_robot")
        self.frame = "LIO_base_link"
        self.commandedPose = PoseStamped()
        self.commandedVel = Twist()
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.terminate = False

        self._a_pressed = False
        self._b_pressed = False
        self._x_pressed = False
        self._grasped = False
        self._ax2_init = False
        self._ax5_init = False
        self._y_pressed = False
        self._right_pressed = False

        # ROS Variables
        rospy.Subscriber("/commanded_vel", Twist, self.commanded_vel_cb)  ##adapt using the prev teleop mcode
        rospy.Subscriber("/joy", Joy, self.joy_cb)

    
        self._twist_pub = rospy.Publisher("/commanded_vel", Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=15)
        self.myp_app_pub = rospy.Publisher("/myp_manager/app_control", String, queue_size=2)

        #Init 
        self.init_pose()
        rospy.sleep(1)
        rospy.loginfo("joy_controller has been started") 

    def mapping(self,x,low=0.005,high=.25):
        a = (np.log(high)-np.log(low))/0.9
        b = np.exp(np.log(low)-.1*a)
        return np.sign(x)*b*((np.exp(a*np.abs(x))-1))
    

    #adjust the below code
    def on_twist(self, msg):
        control_t = self._tfBuffer.lookup_transform(REFERENCE_FRAME,self._control_frame,rospy.Time(0))
        t=PyKDL.Twist()
        t[0]=msg.linear.x
        t[1]=msg.linear.y
        t[2]=msg.linear.z
        t[3]=msg.angular.x
        t[4]=msg.angular.y
        t[5]=msg.angular.z
        rpy=PyKDL.Rotation.Quaternion(control_t.transform.rotation.x,control_t.transform.rotation.y,control_t.transform.rotation.z,control_t.transform.rotation.w).GetRPY()
        R=PyKDL.Rotation.RPY(rpy[0],rpy[1],rpy[2])
        
        t=R*t

        twist = TwistStamped()
        twist.header.frame_id = "panda_gripper_joint"
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.x = t[0]
        twist.twist.linear.y = t[1]
        twist.twist.linear.z = t[2]
        twist.twist.angular.x = t[3]
        twist.twist.angular.y = t[4]
        twist.twist.angular.z = t[5]

        self._goal_pub.publish(twist)



    def joy_cb(self,msg):
        # Filter buttons input
        current_buttons = list(msg.buttons)
        if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
        else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
        self.previous_buttons = current_buttons
    
        if changed_buttons[7] != 0:
            self.terminate = True


        ax = np.array(msg.axes)
        if ax[2] != 0:
            self._ax2_init = True
        if not self._ax2_init:
            ax[2]=1
        if ax[5] != 0:
            self._ax5_init = True
        if not self._ax5_init:
            ax[5]=1

        t = Twist()
        t.linear.x = self.mapping(ax[1])
        t.linear.y = self.mapping(ax[0])
        t.linear.z = self.mapping((ax[2]-ax[5])/2)
        t.angular.x = self.mapping(-ax[3],low=.01,high=1)
        t.angular.y = self.mapping(ax[4],low=.01,high=1)
        if msg.buttons[4]:
            t.angular.z = -np.pi/4
        if msg.buttons[5]:
            t.angular.z = np.pi/4
        self._twist_pub.publish(t)

        # if msg.buttons[0] and not self._a_pressed:
        #     self._a_pressed = True
        #     self._command_pub.publish("toggle_gripper")
        
        # if not msg.buttons[0] and self._a_pressed:
        #     self._a_pressed = False

        # if msg.buttons[1] and not self._b_pressed:
        #     self._b_pressed = True
        #     self._drone_command_pub.publish("takeoff")
        # if not msg.buttons[1] and self._b_pressed:
        #     self._b_pressed = False

        # if msg.buttons[2] and not self._x_pressed:
        #     self._x_pressed = True
        #     self._drone_command_pub.publish("land")
        # if not msg.buttons[2] and self._x_pressed:
        #     self._x_pressed = False

        # if msg.buttons[3] and not self._y_pressed:
        #     self._y_pressed = True
        #     self._view_command_pub.publish("go")
        # if not msg.buttons[3] and self._y_pressed:
        #     self._y_pressed = False

        # if msg.axes[6] and not self._right_pressed:
        #     self._right_pressed = True
        #     self._view_command_pub.publish("new")
        # if not msg.axes[6] and self._right_pressed:
        #     self._right_pressed = False

    
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
        if self.physical_robot: self.myp_app_pub.publish("start")
        while not rospy.is_shutdown():
            if self.terminate: 
                if self.physical_robot: self.myp_app_pub.publish("stop")
                rospy.sleep(1)
                return
            self.update_joy_pose()
            self.publish_commanded_pose()
            self.rate.sleep()

    def update_joy_pose(self):
        self.commandedPose.pose.position.x+=self.commandedVel.linear.x/self.freq
        self.commandedPose.pose.position.y+=self.commandedVel.linear.y/self.freq
        self.commandedPose.pose.position.z+=self.commandedVel.linear.z/self.freq


        # Quaternion initialization
        robot_rot = quat.quat2mat([self.commandedPose.pose.orientation.x, self.commandedPose.pose.orientation.y,
                                self.commandedPose.pose.orientation.z, self.commandedPose.pose.orientation.w])
        
        # Mapping fix to obtain same joystick control as drone_panda
        # Probably due to transforms3d library differences with C++ KDL library
        ang_vel_x = self.commandedVel.angular.z
        ang_vel_y = -self.commandedVel.angular.x
        ang_vel_z = self.commandedVel.angular.y

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