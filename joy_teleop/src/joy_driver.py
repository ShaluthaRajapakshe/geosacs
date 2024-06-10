#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf.transformations import *
import signal
import sys
import numpy as np

class JoyDriver(object):
    def __init__(self):
        rospy.init_node('joy_driver')

        self._twist_pub = rospy.Publisher("/joy_driver/commanded_vel", TwistStamped, queue_size=1)
        self._command_pub = rospy.Publisher("/gripper_state", String, queue_size=1)
        self._joy_sub = rospy.Subscriber('/joy', Joy, self.on_joy)

        self._ax2_init = False
        self._ax5_init = False      
        self._a_pressed = False


        self.start_time = rospy.Time.now()

        self.cumulative_time = 0

        rospy.loginfo("joy_driver has been started") 
      

    def mapping(self,x,low=0.005,high=.25):
        a = (np.log(high)-np.log(low))/0.9
        b = np.exp(np.log(low)-.1*a)
        return np.sign(x)*b*((np.exp(a*np.abs(x))-1))
        
    def on_joy(self, msg):
        ax = np.array(msg.axes) #check this one tomorrow. Ideally this should be all zeros. if there is at least one non-zero value, then collect the time here and 
        # at the end of this function again collect the time and add the diffrence to the cumulative time.
        # print("ax vals", ax,"time on joy", (rospy.Time.now() - self.start_time).to_sec())

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
        new_msg = TwistStamped()
        new_msg.twist=t
        self._twist_pub.publish(new_msg)

        if msg.buttons[0] and not self._a_pressed:
            self._a_pressed = True
            self._command_pub.publish("toggle_gripper")
        
        if not msg.buttons[0] and self._a_pressed:
            self._a_pressed = False
    
    def run(self):
        rospy.spin()

    def signal_handler(self, signal, frame):
        sys.exit()

if __name__ == "__main__":
    mouse_driver = JoyDriver()
    signal.signal(signal.SIGINT, mouse_driver.signal_handler)
    mouse_driver.run()
