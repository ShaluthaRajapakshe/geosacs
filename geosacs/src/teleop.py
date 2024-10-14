#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from sensor_msgs.msg import Joy
from geosacs.msg import SurfacePose
from std_msgs.msg import String
import numpy as np
from scipy.spatial.transform import Rotation 

class Teleoperation(): 
    def __init__(self):
        rospy.init_node("teleoperation") 

        # Variables
        self.rate = rospy.Rate(20) 
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.rot_stride = 0.020
        self.pos_stride = 0.005

        self.position_x = 0.0018523891472076854
        self.position_y = -0.17025735074220158
        self.position_z = 0.7825139929025751

        
        self.orientation_x = -0.697770234523002
        self.orientation_y = 0.7157399411033368
        self.orientation_z = 0.01581835965885365
        self.orientation_w = -0.024141582814929168

        self.physical_robot = rospy.get_param("physical_robot")
      

        rospy.Subscriber("/joy", Joy, self.joy_cb)
        
        self.commanded_pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=10)
        self.myp_app_pub = rospy.Publisher("/myp_manager/app_control", String, queue_size=2)

        #Init Message
        rospy.sleep(1)
        rospy.loginfo("teleoperation node has been started")  

    from scipy.spatial.transform import Rotation

    def quaternion_to_euler(self, quaternion):

        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        
        Parameters:
            quaternion (list or np.array): A 4-element list or numpy array representing the quaternion
                                            in the order [w, x, y, z].
        
        Returns:
            list: Euler angles (roll, pitch, yaw) in radians.
        """
        
        # Create a Rotation object from the quaternion
        r = Rotation.from_quat(quaternion)
        
        # Convert Rotation object to Euler angles (roll, pitch, yaw)
        euler_angles = r.as_euler('xyz')  # Specify 'xyz' order for yaw, pitch, roll
        
        return euler_angles
    
    def euler_to_quaternion(self, euler_angles):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        
        Parameters:
            euler_angles (list or np.array): A 3-element list or numpy array representing the Euler angles
                                            in radians in the order [roll, pitch, yaw].
        
        Returns:
            list: A 4-element list representing the quaternion in the order [w, x, y, z].
        """
        # Create a Rotation object from the Euler angles
        r = Rotation.from_euler('xyz', euler_angles)  # Specify 'xyz' order for yaw, pitch, roll
        
        # Convert Rotation object to quaternion
        quaternion = r.as_quat()
        
        return quaternion


    def joy_cb(self, data):

        angle =  1.570
        R_ab = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle) , 0],
                         [0            , 0             , 1]])
        p_ab = np.array([[0, 0, 0.266]]).reshape(3,1)
        _ = np.array([[0, 0, 0, 1]])
        T_ab = np.column_stack((R_ab, p_ab))
        T_ab = np.vstack((T_ab, _))

        quat = [self.orientation_w, self.orientation_x, self.orientation_y, self.orientation_z]

        euler_angles = self.quaternion_to_euler(quat)


        # Moving along X axis
        if data.axes[0] != 0:
            # as the initial x and y axis will be rotated to align with the LIO_base_link, now the y axis will act as the x axis (rotation of +pi/2 along z axis)
            self.position_y += data.axes[0]/350
            print ("Move along X")
        
        elif data.axes[1] != 0:
            # as the initial x and y axis will be rotated to align with the LIO_base_link, now the x axis will act as the y axis (rotation of +pi/2 along z axis)
            self.position_x += data.axes[1]/350
            print ("Move along Y")
        
        elif data.axes[3] != 0:
            euler_angles[2] += data.axes[3]/100
            print ("Rotate roll", data.axes[3]/1)
            # print ("Print axes[3]", data.axes[3])

        elif data.axes[4] != 0:
            euler_angles[1] += data.axes[4]/100
            print ("Rotate pitch", data.axes[4]/1)

        elif data.buttons[0] != 0: #in the joystic it always start at 1 and reduce to -1 when we press it: Might be an issue with the joystick
            self.position_z -= self.pos_stride
            print ("Move along +Z", data.buttons[0])

        elif data.buttons[3] != 0: #in the joystic it always start at 1 and reduce to -1 when we press it: Might be an issue with the joystick
            self.position_z += self.pos_stride
            print ("Move along -Z", data.buttons[3])


        # elif data.axes[2] < 0.9: #in the joystic it always start at 1 and reduce to -1 when we press it: Might be an issue with the joystick
        #     self.position_z -= data.axes[2]/180
        #     print ("Move along +Z", data.axes[2])

        # elif data.axes[5] < 0.9: #in the joystic it always start at 1 and reduce to -1 when we press it: Might be an issue with the joystick
        #     self.position_z += data.axes[5]/180
        #     print ("Move along -Z", data.axes[5])


        elif data.buttons[4] != 0:
            euler_angles[0] -= self.rot_stride
            print ("Rotate yaw -")


        elif data.buttons[5] != 0:
            euler_angles[0] += self.rot_stride
            print ("Rotate yaw +")



        quat_vals = self.euler_to_quaternion(euler_angles)
        self.orientation_w = quat_vals[0]
        self.orientation_x = quat_vals[1]
        self.orientation_y = quat_vals[2]
        self.orientation_z = quat_vals[3]

        
        #position correction according to LIO_base_link (mapping from LIO_robot_base_link to LIO_base_link)
        pos_b = np.array([self.position_x, self.position_y, self.position_z, 1])
        pos_b = np.array([pos_b]).reshape(4,1)
        pos_a = np.dot(T_ab, pos_b)
        pos_a = np.ravel(pos_a)

        # Orientation correction according to LIO_base_link (mapping from LIO_robot_base_link to LIO_base_link)
    
        R_b = Rotation.from_quat([self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w]).as_matrix()
        R_a = np.dot(R_ab, R_b)
        q_a = Rotation.from_matrix(R_a).as_quat()
        q_a = np.array([q_a[3],q_a[0], q_a[1], q_a[2]])

        # print("pos vals", pos_a)
        # print("orientation vals", q_a)


        #If we want to move the robot w.r.t the LIO_robot_base_link change the frame id to that and remove the multiplication by rotation matrix part above.
        ## Also in the launch file the urdf to the ik engine should be the lio_arm and not the lio_arm_reframed

        msg = PoseStamped()
        msg.header.frame_id = "LIO_base_link"
        msg.pose.position.x = pos_a[0]
        msg.pose.position.y = pos_a[1]
        msg.pose.position.z = pos_a[2]
        msg.pose.orientation.w= q_a[0]
        msg.pose.orientation.x= q_a[1]
        msg.pose.orientation.y= q_a[2]
        msg.pose.orientation.z= q_a[3]


        msg.header.stamp = rospy.Time.now()
        self.commanded_pose_pub.publish(msg)
    
    def run(self):

        if self.physical_robot : 
            self.myp_app_pub.publish("start")
            print("Started")
        rospy.spin()

    

if __name__ == "__main__":
    my_node = Teleoperation()  
    my_node.run()


        # # Moving along Y axis
        # elif data.axes[6] != 0:
        #     if data.axes[6] == 1:
        #         position_r[1] += pos_stride
        #     else:
        #         position_r[1] -= pos_stride
        #     print ("Move along Y")

        # # Moving along Z axis
        # elif data.buttons[4] != 0:
        #     position_r[2] += pos_stride
        
        # elif data.buttons[5] != 0:
        #     position_r[2] -= pos_stride
        #     # print ("Move along Z")

        # # Rotating around +X
        # elif data.buttons[3] != 0:
        #     euler = list(T.euler_from_quaternion(rotation_r))
        #     euler[0] += rot_stride
        #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        #     print ("Rotate along +X")

        # # Rotating around -X
        # elif data.buttons[0] != 0:
        #     euler = list(T.euler_from_quaternion(rotation_r))
        #     euler[0] -= rot_stride
        #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        #     print ("Rotate along -X")

        # # Rotating around +Y                                                                                                                                                            
        # elif data.buttons[2] != 0:
        #     euler = list(T.euler_from_quaternion(rotation_r))
        #     euler[1] += rot_stride
        #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        #     print ("Rotate along +Y")

        # # Rotating around -Y
        # elif data.buttons[1] != 0:
        #     euler = list(T.euler_from_quaternion(rotation_r))
        #     euler[1] -= rot_stride
        #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        #     print ("Rotate along -Y")

        # # Rotating around Z 
        # elif data.axes[4] != 0:
        #     if data.axes[4] > 0:
        #         euler = list(T.euler_from_quaternion(rotation_r))
        #         euler[2] += movement_scale * rot_stride
        #         rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        #     else:
        #         euler = list(T.euler_from_quaternion(rotation_r))
        #         euler[2] -= movement_scale * rot_stride
        #         rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        #     print ("Rotate along Z")