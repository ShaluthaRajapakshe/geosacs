#!/usr/bin/env python3
import rospy
import sys
import os
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState
from geosacs.msg import WeightedPose, Correction, SurfacePose
from randomInitialPoints import randomInitialPoints
from reproduce import reproduce

import tf
import tf2_ros



class MainNode(): 
    def __init__(self):
        rospy.init_node("main_node") 

        # if len(sys.argv) < 2:
        #     rospy.loginfo("Usage: rosrun geosacs main.py <control_front:bool>")
        #     return

        # Get the arguments from the command line
        # self.control_front = sys.argv[1]

        
        

        # self.control_front = True
        # Variables
        self.ratio = None
        self.x_corr = None
        self.y_corr = None
        self.rate = rospy.Rate(15)  # (4;25) ()Prev: 17 #10 IS GOOD WITH REAL ROBOT
        self.frame = "LIO_base_link"
        self.correction = False
        self.physical_robot = rospy.get_param("physical_robot")
        # self.control_front = rospy.get_param("control_front")
        self.weighted_pose =  True
        self.change_direction_request = False
        self.task_done_and_reverse = False
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.terminate = False
        self.gripper_state = ""
        self.gripper_change_direction_request = False
        self.prev_gripper_angle = None
        self.gripper_states = []
        self.start = False

        self.task = "other" ##marshmellow, other
        self.marsh_selected = False

        self.out_of_canal = False


        self.prev_circle_type = None
        self.prev_xy_flag = None

        self.gripper_change_request = False ##added newly

        # ROS Variables
        # Visualisaation
        self.directrix_pub = rospy.Publisher("/tlgc/directrix", PoseArray, queue_size=10)
        self.reproduced_traj_pub = rospy.Publisher("/tlgc/reproduced_trajectory", PoseArray, queue_size=10)
        self.raw_traj_pub = rospy.Publisher("/tlgc/raw_trajectory", PoseArray, queue_size=10)
        self.processed_traj_pub = rospy.Publisher("/tlgc/processed_trajectory", PoseArray, queue_size=10)
        self.gc_circle_pub = rospy.Publisher("/tlgc/gc_circle", PoseArray, queue_size=10)
        self.x_corr_axis_pub = rospy.Publisher("/tlgc/x_correction_axis", PoseArray, queue_size=10)
        self.y_corr_axis_pub = rospy.Publisher("/tlgc/y_correction_axis", PoseArray, queue_size=10)
        self.eT_axis_pub = rospy.Publisher("/tlgc/eT_axis", PoseArray, queue_size=10)
        self.clear_rviz_pub = rospy.Publisher("/tlgc/viz_clear", String, queue_size=2)
        self.raw_pose_traj_pub = rospy.Publisher("/tlgc/raw_pose_trajectory", PoseArray, queue_size=10)
        self.processed_pose_traj_pub = rospy.Publisher("/tlgc/processed_pose_trajectory", PoseArray, queue_size=10)
        self.directrix_pose_pub = rospy.Publisher("/tlgc/directrix_pose", PoseArray, queue_size=10)
        self.reproduced_pose_traj_pub = rospy.Publisher("/tlgc/reproduced_pose_trajectory", PoseArray, queue_size=10)
        self.eN_axis_pub = rospy.Publisher("/tlgc/eN_axis", PoseArray, queue_size=10)
        self.eB_axis_pub = rospy.Publisher("/tlgc/eB_axis", PoseArray, queue_size=10)
        self.events_pub = rospy.Publisher("/tlgc/events", String, queue_size=10)
        self.corrections_pub = rospy.Publisher("/tlgc/corrections", Correction, queue_size=10)
        self.surface_pose_pub = rospy.Publisher("/tlgc/surface_pose", SurfacePose, queue_size=10)
        
        # Other
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        if self.physical_robot: 
            rospy.Subscriber("/lio_1c/joint_states", JointState, self.lio_joint_states_cb)
            rospy.Subscriber("/lio_1c/pose", PoseStamped, self.lio_pose_cb)
            # self.lio_pose = None

        #standard front controlling axes will be x = [0,1,0] and y = [-1,0,0]  
        #Assuming the global x axis will be towards the front of therobot and global Y will be twards the left of the robot
        # Need to decide the x_dir_mul w.r.t the these two axes" Note: the values will change based on whether it is front or rear of the robot
        

        # ## For front control  #object relocation task
        # self.control_front = True
        # self.control_rear = False
        # self.control_left = False

        ## For rear control  #painting task
        self.control_front = False
        self.control_rear = True
        self.control_left = False

        # ### For left control  #laundry task
        # self.control_front = False
        # self.control_rear = False
        # self.control_left = True



        self.joy_x_front = np.array([0, 1, 0])  # Joystick X-axis (global frame)  
        self.joy_y_front = np.array([-1, 0, 0]) # Joystick Y-axis (global frame)
        self.front_config = [self.joy_x_front, self.joy_y_front]

        #standard rear controlling axes will be x = [0,-1,0] and y = [1,0,0]  
        #Assuming the global x axis will be towards the front of therobot and global Y will be twards the left of the robot
        #Need to decide the x_dir_mul w.r.t the these two axes" Note: the values will change based on whether it is front or rear of the robot

        self.joy_x_rear = np.array([0, -1, 0]) # [0, 1, 0]
        self.joy_y_rear = np.array([1, 0, 0])
        self.rear_config = [self.joy_x_rear, self.joy_y_rear]


        self.joy_x_left = np.array([-1, 0, 0]) # [0, 1, 0] #robot's left (as the robot is in front facing to the user)and user's right
        self.joy_y_left = np.array([0, -1, 0])
        self.left_config = [self.joy_x_left, self.joy_y_left]



        ## For XBOX joystick
        self.x_dir_mul = -1 ## This need to be change based on how the joy will give values w.r.t the above two axes (eg: if when we press right, and the joy x will gibe negative values, then this should be -1, else 1)
        self.y_dir_mul = 1 #1 because when w epress up (according the above define axes, the values will give positive values)

        # ## For Sony PS% Access controler
        # self.x_dir_mul = -1 ## This need to be change based on how the joy will give values w.r.t the above two axes (eg: if when we press right, and the joy x will gibe negative values, then this should be -1, else 1)
        # self.y_dir_mul = -1 #1 because when w epress up (according the above define axes, the values will give positive values)
        

        self.commanded_pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=10)
        self.weighted_pose_pub = rospy.Publisher("/weighted_pose", WeightedPose, queue_size=10)
        self.myp_app_pub = rospy.Publisher("/myp_manager/app_control", String, queue_size=2)
        self.gripper_state_pub =rospy.Publisher("/gripper_state", String, queue_size=10)


        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #Init Message
        rospy.sleep(1)
        rospy.loginfo("main_node has been started") 
    
        
    
    def lio_pose_cb(self,msg):
        # print("in pose cb")
        self.lio_pose = msg
    
    def lio_joint_states_cb(self, msg):
        gripper_angle = msg.position[6]
        if self.prev_gripper_angle == None:
            self.prev_gripper_angle = gripper_angle
            return            
        
        # print("gripper_angle", gripper_angle)
        if np.isclose(gripper_angle, 0.000, atol=0.001):
            # print("CLOSED")
            self.gripper_states.append("close")
            
        elif np.isclose(gripper_angle, 0.523, atol=0.001):
            # print("OPEN")
            self.gripper_states.append("open")
        else:
            diff = abs(gripper_angle-self.prev_gripper_angle)
            # print("diff", diff)      
            if diff < 0.001:
                self.gripper_states.append("grasping")
            else:
                self.gripper_states.append("moving")
        
        if len(self.gripper_states)<=3:
            return
        
        self.gripper_states.pop(0)
        # print(self.gripper_states)
        
        if all(state == "close" for state in self.gripper_states) and self.gripper_state != "close":
            self.gripper_state = "close"
            print(f"                                                    **{self.gripper_state}*")
            self.events_pub.publish(self.gripper_state)
        elif all(state == "open" for state in self.gripper_states) and self.gripper_state != "open":
            self.gripper_state = "open"
            print(f"                                                    **{self.gripper_state}*")
            self.events_pub.publish(self.gripper_state)
        elif all(state == "moving" for state in self.gripper_states) and self.gripper_state != "moving":
            self.gripper_state = "moving"
            print(f"                                                    **{self.gripper_state}*")
            self.events_pub.publish(self.gripper_state)
        elif all(state == "grasping" for state in self.gripper_states) and self.gripper_state != "grasping":
            self.gripper_state = "grasping"
            self.gripper_change_direction_request = True
            print(f"                                                    **{self.gripper_state}*")
            self.events_pub.publish(self.gripper_state)


        
        self.prev_gripper_angle = gripper_angle


    def pub_correction(self, idx, current_idx, strt, end, q, distance, duration):
        
        start_pose = Pose()
        start_pose.position.x = strt[0]
        start_pose.position.y = strt[1]
        start_pose.position.z = strt[2]
        start_pose.orientation.w = q[0]
        start_pose.orientation.x = q[1]
        start_pose.orientation.y = q[2]
        start_pose.orientation.z = q[3]
        end_pose = Pose()
        end_pose.position.x = end[0]
        end_pose.position.y = end[1]
        end_pose.position.z = end[2]
        end_pose.orientation.w = q[0]
        end_pose.orientation.x = q[1]
        end_pose.orientation.y = q[2]
        end_pose.orientation.z = q[3]
        msg = Correction()
        msg.idx = idx
        msg.current_idx = current_idx
        msg.distance = distance
        msg.duration = duration
        msg.start_pose = start_pose
        msg.end_pose = end_pose
        self.corrections_pub.publish(msg)
    
    
    def pub_surface_pose(self, idx, current_idx, PcurrG, AcurrG, q, q_weight, ratio, state):
        
        surface_pose = Pose()
        surface_pose.position.x = AcurrG[0]
        surface_pose.position.y = AcurrG[1]
        surface_pose.position.z = AcurrG[2]
        surface_pose.orientation.w = q[0]
        surface_pose.orientation.x = q[0]
        surface_pose.orientation.y = q[1]
        surface_pose.orientation.z = q[2]

        global_pose = Pose()
        global_pose.position.x = PcurrG[0]
        global_pose.position.y = PcurrG[1]
        global_pose.position.z = PcurrG[2]
        global_pose.orientation.w = q[0]
        global_pose.orientation.x = q[0]
        global_pose.orientation.y = q[1]
        global_pose.orientation.z = q[2]

        msg = SurfacePose()
        msg.idx = idx
        msg.current_idx = current_idx
        msg.pose_surface = surface_pose
        msg.ratio = ratio
        msg.q_weight = q_weight
        msg.pose_global = global_pose
        msg.state = state
        self.surface_pose_pub.publish(msg)


        
    ### with logitech joystick controller
    def joy_cb(self, msg):
        
        if msg.axes[0] != 0 or msg.axes[1] != 0:
            if self.control_front:
                self.joy_x = self.front_config[0]
                self.joy_y = self.front_config[1]

            elif self.control_rear:
                self.joy_x = self.rear_config[0]
                self.joy_y = self.rear_config[1]

            elif self.control_left:
                # print("#############IN LEFT")
                self.joy_x = self.left_config[0]
                self.joy_y = self.left_config[1]
                
            ## xbox controller   
            self.y_corr = self.y_dir_mul * msg.axes[1]/150
            self.x_corr = self.x_dir_mul * msg.axes[0]/150

            self.correction = True
            return # cannot do corrections & change direction at the same time
        
    

        # Filter buttons input
        current_buttons = list(msg.buttons)
        if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
        else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
        self.previous_buttons = current_buttons

        # if changed_buttons[1] != 0:
            
        #     if self.control_front:
        #         # self.control_front = False
        #         rospy.loginfo("###### Mapping done considering the user is in front side of the robot")
        #         # self.joy_x = self.front_config[0]
        #         # self.joy_y = self.front_config[1]

        #         self.joy_x = self.rear_config[0]
        #         self.joy_y = self.rear_config[1]
        #     else:
        #         self.control_front = True
        #         rospy.loginfo("###### Mapping done considering the user is in front side of the robot")
        #         ## self.joy_x = self.rear_config[0]
        #         ## self.joy_y = self.rear_config[1]

        #         # self.joy_x = self.front_config[0]
        #         # self.joy_y = self.front_config[1]

        #         self.joy_x = self.left_config[0]
        #         self.joy_y = self.left_config[1]


        if changed_buttons[3] != 0:
            # print("CHANGE DIRECTION")
            self.change_direction_request = True
            return
        
        if changed_buttons[0] != 0:
            self.gripper_state_pub.publish("toggle_gripper")
            self.gripper_change_request = True

            # rospy.sleep(3)

        if changed_buttons[7] != 0:
            # print("STOP")
            self.terminate = True
        if changed_buttons[6] != 0:
            # print("START")
            self.start = True

        if changed_buttons[2] != 0:
            self.marsh_selected = True



    # ### with sony access controller
    # def joy_cb(self, msg):

    #     if msg.axes[0] != 0 or msg.axes[1] != 0:

    #         if self.control_front:
    #             self.joy_x = self.front_config[0]
    #             self.joy_y = self.front_config[1]

    #         else:
    #             self.joy_x = self.rear_config[0]
    #             self.joy_y = self.rear_config[1]
                
    #         ##sony access controller
    #         self.y_corr = self.y_dir_mul * msg.axes[0]/150
    #         self.x_corr = self.x_dir_mul * msg.axes[1]/150

    #         self.correction = True
    #         return # cannot do corrections & change direction at the same time

    #     # Filter buttons input
    #     current_buttons = list(msg.buttons)
    #     if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
    #     else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
    #     self.previous_buttons = current_buttons


    #     if changed_buttons[1] != 0:
    #         # print("CHANGE DIRECTION")
    #         self.change_direction_request = True
    #         return
    #     if changed_buttons[9] != 0:
    #         # print("STOP")
    #         self.terminate = True
    #     if changed_buttons[12] != 0:
    #         # print("START")
    #         self.start = True
    #     if changed_buttons[2] != 0:
    #         self.gripper_state_pub.publish("toggle_gripper")





        
            
    def get_model(self, model_dir):

        rospy.loginfo(f"Model directory {model_dir}")

        items = os.listdir(model_dir)
        rospy.loginfo(f"  Files: {items}")

        filename = model_dir + f"/{items[0]}"
        model_npz = np.load(filename)
        rospy.loginfo(f"  File data: {model_npz.files}")
        rospy.loginfo(f"  Model loaded with {len(model_npz['directrix'])} directrix points.")
        rospy.loginfo(f"  Model idx {model_npz['idx']}.")

        return model_npz

    def visualise_trajectory(self, traj_xyz, traj_q):
        
        if traj_xyz.shape[0] != traj_q.shape[0]:
            rospy.logerr("visualise_trajectory : traj_xyz.shape[0] != traj_q.shape[0]")
        
        msg = PoseArray()
        nb_points = traj_xyz.shape[0]

        for i in range(nb_points):
            p = Pose()
            p.position.x = traj_xyz[i,0]
            p.position.y = traj_xyz[i,1]
            p.position.z = traj_xyz[i,2]
            p.orientation.w = traj_q[i,0]
            p.orientation.x = traj_q[i,1]
            p.orientation.y = traj_q[i,2]
            p.orientation.z = traj_q[i,3]
            msg.poses.append(p)
        
        self.reproduced_traj_pub.publish(msg)
        self.reproduced_pose_traj_pub.publish(msg)

    def visualise_model(self, model):
        
        gc_circles = model["GC"]
        for circle in gc_circles:
            msg = PoseArray()
            for point in circle:
                p = Pose()
                p.position.x = point[0]
                p.position.y = point[1]
                p.position.z = point[2]
                msg.poses.append(p)

            self.gc_circle_pub.publish(msg)
        
        directrix = model["directrix"]
        msg = PoseArray()
        for point in directrix:
            p = Pose()
            p.position.x = point[0]
            p.position.y = point[1]
            p.position.z = point[2]
            msg.poses.append(p)

        self.directrix_pub.publish(msg)

    def init_pose(self):
        msg1 = PoseStamped()
        msg1.header.frame_id = "LIO_base_link"
        msg1.pose.position.x = 0.20695484883032408
        msg1.pose.position.y = 0.0005657323485566697
        msg1.pose.position.z = 1.061217401594137

        msg1.pose.orientation.w= 0.00699019334487028
        msg1.pose.orientation.x= 0.9982331952531064
        msg1.pose.orientation.y= -0.05193699424944531
        msg1.pose.orientation.z= -0.02800310197296562
        msg1.header.stamp = rospy.Time.now()
        self.commanded_pose_pub.publish(msg1)

        
    def pub_cmd_pose(self, PcurrG, QcurrG, q_weight):
        msg1 = PoseStamped()
        msg1.header.frame_id = "LIO_base_link"
        msg1.pose.position.x = PcurrG[0]
        msg1.pose.position.y = PcurrG[1]
        msg1.pose.position.z = PcurrG[2]
        msg1.pose.orientation.w= QcurrG[0]
        msg1.pose.orientation.x= QcurrG[1]
        msg1.pose.orientation.y= QcurrG[2]
        msg1.pose.orientation.z= QcurrG[3]
        msg1.header.stamp = rospy.Time.now()
        self.commanded_pose_pub.publish(msg1)
        
        msg2 = WeightedPose()
        msg2.pose = msg1.pose
        msg2.weight = q_weight
        # print("WEIGHT:", q_weight)
        self.weighted_pose_pub.publish(msg2)
    
    def get_raw_demos(self, raw_dir):

        rospy.loginfo(f"Raw directory {raw_dir}")
        items = os.listdir(raw_dir)
        rospy.loginfo(f"  Files found {items}")

        demos_xyz = []
        demos_q = []
        i=1
        for file in items:
            
            demo_file = raw_dir + f"/{file}"
            raw_dict = np.load(demo_file)
            xyz = raw_dict["position"]
            q = raw_dict["orientation"]
            demos_xyz.append(xyz)
            demos_q.append(q)
            rospy.loginfo(f"  Demo {i} with {len(xyz)} xyz and {len(q)} q")
            i+=1

        return demos_xyz, demos_q

    def get_raw_demos(self, raw_dir):

        rospy.loginfo(f"Raw directory {raw_dir}")
        items = os.listdir(raw_dir)
        rospy.loginfo(f"  Files found {items}")

        demos_xyz = []
        demos_q = []
        i=1
        for file in items:
            
            demo_file = raw_dir + f"/{file}"
            raw_dict = np.load(demo_file)
            xyz = raw_dict["position"]
            q = raw_dict["orientation"]
            demos_xyz.append(xyz)
            demos_q.append(q)
            rospy.loginfo(f"  Demo {i} with {len(xyz)} xyz and {len(q)} q")
            i+=1

        return demos_xyz, demos_q

    def get_processed_demos(self, processed_dir):
        
        rospy.loginfo(f"Processed directory {processed_dir}")
        items = os.listdir(processed_dir)
        rospy.loginfo(f"  Files found {items}")

        filename = processed_dir + f"/{items[0]}"
        dict = np.load(filename)
        rospy.loginfo(f"  Data available: {dict.files}")
        demos_xyz = dict["position"]
        demos_q = dict["orientation"]
        rospy.loginfo(f"  Processed demos shape {demos_xyz.shape} xyz and {demos_q.shape} q")

        return demos_xyz, demos_q

    def visualise_demos(self, demos_xyz, demos_q, type):

        if len(demos_xyz) != len(demos_q):
            rospy.logerr("visualise_demos: len(demos_xyz) != len(demos_q)")
            return

        nb_demos = len(demos_xyz)

        for i in range(nb_demos):

            if len(demos_xyz[i]) != len(demos_q[i]):
                rospy.logerr("visualise_demos: len(demos_xyz[i]) != len(demos_q[i])")
                return
            
            positions = demos_xyz[i]
            orientations = demos_q[i]
            nb_points = len(positions)
            msg = PoseArray()
            for j in range(nb_points):
                pose = Pose()
                pose.position.x = positions[j,0]
                pose.position.y = positions[j,1]
                pose.position.z = positions[j,2]

                pose.orientation.w = orientations[j,0]
                pose.orientation.x = orientations[j,1]
                pose.orientation.y = orientations[j,2]
                pose.orientation.z = orientations[j,3]

                msg.poses.append(pose)
            
            if type == "processed":
                self.processed_traj_pub.publish(msg)
                self.processed_pose_traj_pub.publish(msg)
            elif type == "raw": 
                self.raw_traj_pub.publish(msg)
                self.raw_pose_traj_pub.publish(msg)
            else :
                rospy.logerr("Type of trajectory not detected.")

    
    def visualise_model(self, model):
        gc_circles = model["GC"]
        
        for circle in gc_circles:
            msg = PoseArray()
            for point in circle:
                p = Pose()
                p.position.x = point[0]
                p.position.y = point[1]
                p.position.z = point[2]
                msg.poses.append(p)

            self.gc_circle_pub.publish(msg)
            rospy.sleep(0.05)
        

        directrix = model["directrix"]
        q_mean = model["q"]
        if directrix.shape[0] != q_mean.shape[0]:
            rospy.logerr("visualise_model function: directrix.shape[0] != q_mean.shape[0]")
            return
        
        nb_points = directrix.shape[0]
        msg = PoseArray()
        for i in range(nb_points):
            p = Pose()
            p.position.x = directrix[i,0]
            p.position.y = directrix[i,1]
            p.position.z = directrix[i,2]

            p.orientation.w = q_mean[i,0] # We have the quaternions in w, x, y, z format
            p.orientation.x = q_mean[i,1]
            p.orientation.y = q_mean[i,2]
            p.orientation.z = q_mean[i,3]

            msg.poses.append(p)

        self.directrix_pub.publish(msg)
        self.directrix_pose_pub.publish(msg)
    
    
    def visualise_correction_axes(self, x_axes, y_axes, directrix):
        
        nb_points = directrix.shape[0]
        for i in range(nb_points):
            strt_arrow = Pose()
            strt_arrow.position.x = directrix[i,:][0]
            strt_arrow.position.y = directrix[i,:][1]
            strt_arrow.position.z = directrix[i,:][2]
            x_arrow_end = Pose()
            x_arrow_end.position.x = directrix[i,:][0] + 0.02*x_axes[i,:][0]
            x_arrow_end.position.y = directrix[i,:][1] + 0.02*x_axes[i,:][1]
            x_arrow_end.position.z = directrix[i,:][2] + 0.02*x_axes[i,:][2]
            y_arrow_end = Pose()
            y_arrow_end.position.x = directrix[i,:][0] + 0.02*y_axes[i,:][0]
            y_arrow_end.position.y = directrix[i,:][1] + 0.02*y_axes[i,:][1]
            y_arrow_end.position.z = directrix[i,:][2] + 0.02*y_axes[i,:][2]
            
            x_arrow = PoseArray()
            x_arrow.poses.append(strt_arrow)
            x_arrow.poses.append(x_arrow_end)

            y_arrow = PoseArray()
            y_arrow.poses.append(strt_arrow)
            y_arrow.poses.append(y_arrow_end)

            self.x_corr_axis_pub.publish(x_arrow)
            self.y_corr_axis_pub.publish(y_arrow)
            rospy.sleep(0.05)
        
        return
    
    
    
    def visualise_gc(self, gc_circles):  

        for circle in gc_circles:
            msg = PoseArray()
            for point in circle:
                p = Pose()
                p.position.x = point[0]
                p.position.y = point[1]
                p.position.z = point[2]
                msg.poses.append(p)

            self.gc_circle_pub.publish(msg)
            rospy.sleep(0.1)


    def visualise_directrix(self, directrix, q_mean):

        if directrix.shape[0] != q_mean.shape[0]:
            rospy.logerr("visualise_model function: directrix.shape[0] != q_mean.shape[0]")
            return
        
        nb_points = directrix.shape[0]
        msg = PoseArray()
        for i in range(nb_points):
            p = Pose()
            p.position.x = directrix[i,0]
            p.position.y = directrix[i,1]
            p.position.z = directrix[i,2]

            p.orientation.w = q_mean[i,0] # We have the quaternions in w, x, y, z format
            p.orientation.x = q_mean[i,1]
            p.orientation.y = q_mean[i,2]
            p.orientation.z = q_mean[i,3]

            msg.poses.append(p)

        self.directrix_pub.publish(msg)
        self.directrix_pose_pub.publish(msg)
    

    def visualise_TNB_axes(self, axes, directrix, type):

        
        nb_points = directrix.shape[0]

        for i in range(nb_points):
            strt_arrow = Pose()
            strt_arrow.position.x = directrix[i,:][0]
            strt_arrow.position.y = directrix[i,:][1]
            strt_arrow.position.z = directrix[i,:][2]
            arrow_end = Pose()
            arrow_end.position.x = directrix[i,:][0] + 0.02*axes[i,:][0]
            arrow_end.position.y = directrix[i,:][1] + 0.02*axes[i,:][1]
            arrow_end.position.z = directrix[i,:][2] + 0.02*axes[i,:][2]

            arrow = PoseArray()
            arrow.poses.append(strt_arrow)
            arrow.poses.append(arrow_end)

            if type == "eT" : self.eT_axis_pub.publish(arrow)
            if type == "eN" : self.eN_axis_pub.publish(arrow)
            if type == "eB" : self.eB_axis_pub.publish(arrow)


            rospy.sleep(0.05)
        
        return
    
    
    def slice_model(self, model, start_idx, end_idx):
        
        # rospy.loginfo(f"Slice model:")
        if start_idx>end_idx: 
            direction = -1
        elif end_idx > start_idx: 
            direction = 1
        else : rospy.logerr("slice_model: edge case not addressed")

        # rospy.loginfo(f"  Slicing model from {start_idx} to {end_idx} in {direction} direction.")
        
        keep_idxs = np.arange(start_idx, end_idx+1*direction, direction)
        # rospy.loginfo(f"  Slice model to {keep_idxs.shape[0]} points")
        sliced_model = {}
        sliced_model["directrix"] = model["directrix"][keep_idxs, :] 
        sliced_model["eT"] = model["eT"][keep_idxs, :] 
        sliced_model["eN"] = model["eN"][keep_idxs, :] 
        sliced_model["eB"] = model["eB"][keep_idxs, :] 
        sliced_model["x_corr_axes"] = model["x_corr_axes"][keep_idxs, :] 
        sliced_model["y_corr_axes"] = model["y_corr_axes"][keep_idxs, :] 

        sliced_model["Rc"] = model["Rc"][keep_idxs] 
        sliced_model["q"] = model["q"][keep_idxs, :] 
        sliced_model["GC"] = model["GC"][keep_idxs, :] 

        sliced_model["vertical_start"] = model["vertical_start"]
        sliced_model["vertical_end"] = model["vertical_end"]

        sliced_model["xyz_corr_y"] = model["xyz_corr_y"]

        # rospy.loginfo(f"  Sliced model directrix shape {sliced_model['directrix'].shape}")

        return sliced_model, direction
        
    # Function to project a vector onto a plane defined by a normal vector
    def project_onto_plane(self, v, normal):
        normal_normalized = normal / np.linalg.norm(normal)
        projection = v - np.dot(v, normal_normalized) * normal_normalized
        return projection

    # Function to compute the dot product projections of joystick axes onto correction axes
    def compute_axis_projections(self, correction_axes, joy_x, joy_y):
        projections_x = [np.dot(joy_x, axis) for axis in correction_axes]
        projections_y = [np.dot(joy_y, axis) for axis in correction_axes]
        return projections_x, projections_y

    # Function to select the most aligned correction axis based on projections
    def select_aligned_axis(self, projections):
        max_index = np.argmax(np.abs(projections))
        direction = np.sign(projections[max_index])
        return max_index, direction

    # Function to compute the normal of a plane defined by two vectors
    def compute_normal(self, axis1, axis2):
        return np.cross(axis1, axis2)

    # Function to compute a rotation matrix given an axis and angle
    # def rotation_matrix(self, axis, angle):
    #     axis = axis / np.linalg.norm(axis)
    #     a = np.cos(angle / 2.0)
    #     b, c, d = -axis * np.sin(angle / 2.0)
    #     return np.array([[a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d + a*c)],
    #                     [2*(b*c + a*d), a*a + c*c - b*b - d*d, 2*(c*d - a*b)],
    #                     [2*(b*d - a*c), 2*(c*d + a*b), a*a + d*d - b*b - c*c]])

    # # Function to rotate a vector by a given axis and angle
    # def rotate_vector(self, vector, axis, angle):
    #     R = self.rotation_matrix(axis, angle)
    #     return np.dot(R, vector)


    def calculate_angle_between_vectors(self, v1, v2):
        # Normalize the vectors to ensure they are unit vectors
        v1_normalized = v1 / np.linalg.norm(v1)
        v2_normalized = v2 / np.linalg.norm(v2)
        # Calculate the cosine of the angle using the dot product
        cos_angle = np.clip(np.dot(v1_normalized, v2_normalized), -1.0, 1.0)
        # Calculate the amgle in radians and then convert to degrees
        angle = np.arccos(cos_angle)
        angle_degrees = np.degrees(angle)
        return angle_degrees



    def vertical_check(self, eT):

        # print("######eT", eT)
    
        global_z = np.array([0, 0, 1])
        z_diff = self.calculate_angle_between_vectors(eT, global_z)

        if z_diff < 90:
            if z_diff > 60:
                return True
            else:
                return False
        else:
            if z_diff < 120:
                return True
            else:
                return False

    # Function to determine the direction for joystick X-axis
    def determine_x_direction(self, aligned_y_axis, correction_axes, joy_x):
        # Project the joystick X-axis onto the plane formed by the correction axes
        normal = np.cross(correction_axes[0], correction_axes[1])
        projected_joy_x = self.project_onto_plane(joy_x, normal)
        projected_joy_x /= np.linalg.norm(projected_joy_x)  # Normalize

        # Determine which correction axis aligns with the projected joystick X-axis
        projections = [np.dot(projected_joy_x, axis) for axis in correction_axes]
        aligned_axis_index_x, direction_x = self.select_aligned_axis(projections)

        return correction_axes[aligned_axis_index_x], direction_x


    # Function to determine the direction for joystick X-axis
    def determine_x_direction_new(self, aligned_x_axis, correction_axes, joy_x):
        # Project the joystick X-axis onto the plane formed by the correction axes
        normal = np.cross(correction_axes[0], correction_axes[1])
        projected_joy_x = self.project_onto_plane(joy_x, normal)
        projected_joy_x /= np.linalg.norm(projected_joy_x)  # Normalize

        # Determine which correction axis aligns with the projected joystick X-axis
        projection = np.dot(projected_joy_x, aligned_x_axis) 
        direction_x = np.sign(projection)

        return direction_x



    def check_position_in_joystick_frame(self, R_T, p_global):
        # Transform the point to the joystick frame
        p_joystick = np.dot(R_T, p_global)
        
        # Check the sign of the X-coordinate in the joystick frame
        if p_joystick[0] > 0:
            position = "right"
        else:
            position = "left"

        return position, p_joystick
    

    def vertical_type(self, horizontal_axis, global_z, joy_y):
        projected_horizontal_axis = self.project_onto_plane(horizontal_axis, global_z)

        angle_with_joy_y = self.calculate_angle_between_vectors(projected_horizontal_axis, joy_y)

        if angle_with_joy_y <= 30 or angle_with_joy_y >= 150:  ## TODO: this can be reduced to 20 and 160 
            type = "typeA"
        else:
            type = "typeB"

        print(f"########## Type", type)
            
        return type
        

    def direction_x_new(self, type, position, correction_x, joy_y, joy_x, global_z):
        projected_horizontal_axis = self.project_onto_plane(correction_x, global_z)

        if type == "typeA":
            alignment = np.dot(joy_y, projected_horizontal_axis)
            sign = np.sign(alignment)

            print("######",  type, position, sign)

            if position == "left":
                return sign
            else:
                return -sign
        
        else:
            alignment = np.dot(joy_x, projected_horizontal_axis)
            sign = np.sign(alignment)

            ## No need to check whether left or right for typeB vertical planes

            return sign
        


    # Main function to map input
    def map_input(self, correction_x, correction_y, joy_x, joy_y, eT, directrix):
        correction_axes = [
            np.array(correction_x),
            np.array(correction_y)
        ]

        

        z = np.array([0, 0, 1])

        vertical_plane = self.vertical_check(eT)


        if vertical_plane:


            # Define global axes
            g_x = np.array([1, 0, 0])
            g_y = np.array([0, 1, 0])
            g_z = np.array([0, 0, 1])

            # Define joystick axes w.r.t global frame
            j_x = joy_x
            j_y = joy_y
            j_z = np.cross(j_x, j_y)  # Should be [0, 0, 1]

            # Rotation matrix from global frame to joystick frame
            R = np.column_stack((j_x, j_y, j_z))

            # Transpose of the rotation matrix
            R_T = R.T

            position, p_joystick = self.check_position_in_joystick_frame(R_T, directrix)

            print(f"########## directrix", directrix, " and I'm on,", position,"  w.r.t joyx")
            

            # angle_corr_x = np.arccos(np.dot(correction_axes[0], z) / np.linalg.norm(correction_axes[0]))
            # angle_corr_y = np.arccos(np.dot(correction_axes[1], z) / np.linalg.norm(correction_axes[1]))

            # if (self.control_front and (joy_y == np.array([-1, 0, 0])).all()) or (not self.control_front and (joy_y == np.array([1, 0, 0])).all()):
            if (self.control_front and (joy_y == np.array([-1, 0, 0])).all()) or (self.control_rear and (joy_y == np.array([1, 0, 0])).all()) or (self.control_left and (joy_y == np.array([0, -1, 0])).all()): #config for left side cotrol
                y_dir_mult = 1

            else:
                y_dir_mult = -1

                print("########### Y dir mult", y_dir_mult)

            def angle_with_z(axis):
                angle = np.arccos(np.dot(axis, z) / np.linalg.norm(axis))
                # if angle > np.pi / 2:
                #     direction = -1.0
                # else:
                #     direction = 1.

                
                # return min(angle, np.pi - angle), direction
                return min(angle, np.pi - angle)

            angle_corr_x = angle_with_z(correction_axes[0])
            angle_corr_y = angle_with_z(correction_axes[1])

            if angle_corr_x < angle_corr_y:
                aligned_axis_y = correction_axes[0]
                aligned_axis_x = correction_axes[1]


                type = self.vertical_type(aligned_axis_x, g_z, joy_y)

                direction_x = self.direction_x_new(type, position, aligned_axis_x, joy_y, joy_x, g_z)

                direction_y = np.sign(np.dot(aligned_axis_y, z))
                direction_y = y_dir_mult * direction_y
                # self.prev_xy_flag = False

            elif angle_corr_y < angle_corr_x:
                
                aligned_axis_y = correction_axes[1]
                aligned_axis_x = correction_axes[0]
                
                type = self.vertical_type(aligned_axis_x, g_z, joy_y)

                direction_x = self.direction_x_new(type, position, aligned_axis_x, joy_y, joy_x, g_z)

                direction_y = np.sign(np.dot(aligned_axis_y, z))
                direction_y = y_dir_mult * direction_y
                # self.prev_xy_flag = True

            else:
                aligned_axis_y = correction_axes[0]
                aligned_axis_x = correction_axes[1]
                
                type = self.vertical_type(aligned_axis_x, g_z, joy_y)

                direction_x = self.direction_x_new(type, position, aligned_axis_x, joy_y, joy_x, g_z)
                
                direction_y = np.sign(np.dot(aligned_axis_y, z))
                direction_y = y_dir_mult * direction_y
            
                
                
            
        else:

            # print("############     Horizontal Plane")

            projected_corr_x = self.project_onto_plane(correction_axes[0], z ) 
            projected_corr_y = self.project_onto_plane(correction_axes[1], z ) 


            projected_corr_axes = np.array([projected_corr_x, projected_corr_y])


            # Compute the projections of the joystick axes onto the correction axes
            projections_x, projections_y = self.compute_axis_projections(projected_corr_axes, joy_x, joy_y)

            # Select the most aligned correction axis for joystick X-axis
            aligned_axis_index_x, direction_x = self.select_aligned_axis(projections_x) 
            


            # Select the most aligned correction axis for joystick Y-axis
            aligned_axis_index_y, direction_y = self.select_aligned_axis(projections_y)
            

            # print("Aligned Axis Index for Joystick Y-axis:", aligned_axis_index_y)
            # print("Direction for Joystick Y-axis:", direction_y)


            # The most aligned correction axes, invert if direction is negative
            aligned_axis_x = correction_axes[aligned_axis_index_x] 
            aligned_axis_y = correction_axes[aligned_axis_index_y]

            # print("aligned x axis for Joyx", )

            # if 45 degree case, we will assign correctionX to joyx and correctionY to joyy
            if (aligned_axis_x == aligned_axis_y).all(): 
                aligned_axis_x = correction_axes[0]
                aligned_axis_y = correction_axes[1]


            # ## Taking the actual correction axeses (untill now we were dealing with projected vectors on the horizontal plane)
            # ## Also this will handle cases like 45 degrees where joy axes can be mapped to one same correction axis, but the following step ensures at last it will be applied to the two axes again

            # if (aligned_axis_x == projected_corr_x).all():
            #     print("########### here in if condition")
            #     aligned_axis_x = correction_axes[0]
            #     aligned_axis_y = correction_axes[1]
            # else:
            #     print("########### here in else")
            #     aligned_axis_x = correction_axes[1]
            #     aligned_axis_y = correction_axes[0]

            self.prev_circle_type = "horizontal"

        # print("original correction axes", correction_axes[0], correction_axes[1])
        # print("rotated correction axes", correction_axes_rotated)
        # print("Aligned Axis for Joystick X-axis:", aligned_axis_x, "Direction identifier", direction_x)
        # print("Aligned Axis for Joystick Y-axis:", aligned_axis_y, "Direction identifier", direction_y)

        return aligned_axis_x, aligned_axis_y, direction_x, direction_y
    




    # # Main function to map input
    # def map_input(self, correction_x, correction_y, joy_x, joy_y, eT, directrix):
    #     correction_axes = [
    #         np.array(correction_x),
    #         np.array(correction_y)
    #     ]

        

    #     z = np.array([0, 0, 1])

    #     vertical_plane = self.vertical_check(eT)


    #     if vertical_plane:


    #         # Define global axes
    #         g_x = np.array([1, 0, 0])
    #         g_y = np.array([0, 1, 0])
    #         g_z = np.array([0, 0, 1])

    #         # Define joystick axes w.r.t global frame
    #         j_x = joy_x
    #         j_y = joy_y
    #         j_z = np.cross(j_x, j_y)  # Should be [0, 0, 1]

    #         # Rotation matrix from global frame to joystick frame
    #         R = np.column_stack((j_x, j_y, j_z))

    #         # Transpose of the rotation matrix
    #         R_T = R.T

    #         position, p_joystick = self.check_position_in_joystick_frame(R_T, directrix)

    #         print(f"########## directrix", directrix, " and I'm on,", position,"  w.r.t joyx")
            

    #         # angle_corr_x = np.arccos(np.dot(correction_axes[0], z) / np.linalg.norm(correction_axes[0]))
    #         # angle_corr_y = np.arccos(np.dot(correction_axes[1], z) / np.linalg.norm(correction_axes[1]))

    #         if (self.control_front and (joy_y == np.array([-1, 0, 0])).all()) or (not self.control_front and (joy_y == np.array([1, 0, 0])).all()):
    #             y_dir_mult = 1

    #         else:
    #             y_dir_mult = -1

    #         def angle_with_z(axis):
    #             angle = np.arccos(np.dot(axis, z) / np.linalg.norm(axis))
    #             # if angle > np.pi / 2:
    #             #     direction = -1.0
    #             # else:
    #             #     direction = 1.

                
    #             # return min(angle, np.pi - angle), direction
    #             return min(angle, np.pi - angle)

    #         angle_corr_x = angle_with_z(correction_axes[0])
    #         angle_corr_y = angle_with_z(correction_axes[1])

    #         if angle_corr_x < angle_corr_y:
    #             aligned_axis_y = correction_axes[0]
    #             aligned_axis_x = correction_axes[1]


    #             type = self.vertical_type(aligned_axis_x, z, joy_y)

                

    #             # aligned_axis_x, direction_x = self.determine_x_direction(aligned_axis_y, correction_axes, joy_x)
    #             if self.prev_circle_type == "vertical" and self.prev_xy_flag == False:   ### here we have taken the assumptoin that the planes will be gradual so that axes will not change
                    
    #                 if not self.control_front:
    #                     direction_x = -self.prev_x_dir
    #                 else:
    #                     direction_x = self.prev_x_dir
                
    #             else: #here just the xy_flag might not be enough, in order to keep it consistent, we neet to check the alignment with previous aligned_axis_x (but these are edge cases)
    #                 direction_x = self.determine_x_direction_new(aligned_axis_x, correction_axes, joy_x)
    #                 self.prev_x_dir =  direction_x

    #             direction_y = np.sign(np.dot(aligned_axis_y, z))
    #             direction_y = y_dir_mult * direction_y
    #             self.prev_xy_flag = False

    #         elif angle_corr_y < angle_corr_x:
                
    #             aligned_axis_y = correction_axes[1]
    #             aligned_axis_x = correction_axes[0]
    #             # aligned_axis_x, direction_x = self.determine_x_direction(aligned_axis_y, correction_axes, joy_x)
    #             if self.prev_circle_type == "vertical" and self.prev_xy_flag == True:
    #                 if not self.control_front:
    #                     direction_x = -self.prev_x_dir
    #                 else:
    #                     direction_x = self.prev_x_dir
    #             else:
    #                 direction_x = self.determine_x_direction_new(aligned_axis_x, correction_axes, joy_x)
    #                 self.prev_x_dir =  direction_x

    #             direction_y = np.sign(np.dot(aligned_axis_y, z))
    #             direction_y = y_dir_mult * direction_y
    #             self.prev_xy_flag = True

    #         else:
    #             aligned_axis_y = correction_axes[0]
    #             aligned_axis_x = correction_axes[1]
    #             # aligned_axis_x, direction_x = self.determine_x_direction(aligned_axis_y, correction_axes, joy_x)
    #             if self.prev_circle_type == "vertical":
    #                 if not self.control_front:
    #                     direction_x = -self.prev_x_dir
    #                 else:
    #                     direction_x = self.prev_x_dir  #this because we conisder front control as default
    #             else:
    #                 direction_x = self.determine_x_direction_new(aligned_axis_x, correction_axes, joy_x)
    #                 self.prev_x_dir =  direction_x
                
    #             direction_y = np.sign(np.dot(aligned_axis_y, z))
    #             direction_y = y_dir_mult * direction_y

    #         self.prev_circle_type = "vertical"
            
                
                
            
    #     else:

    #         projected_corr_x = self.project_onto_plane(correction_axes[0], z ) 
    #         projected_corr_y = self.project_onto_plane(correction_axes[1], z ) 


    #         projected_corr_axes = np.array([projected_corr_x, projected_corr_y])


    #         # Compute the projections of the joystick axes onto the correction axes
    #         projections_x, projections_y = self.compute_axis_projections(projected_corr_axes, joy_x, joy_y)

    #         # Select the most aligned correction axis for joystick X-axis
    #         aligned_axis_index_x, direction_x = self.select_aligned_axis(projections_x) 
            


    #         # Select the most aligned correction axis for joystick Y-axis
    #         aligned_axis_index_y, direction_y = self.select_aligned_axis(projections_y)
            

    #         # print("Aligned Axis Index for Joystick Y-axis:", aligned_axis_index_y)
    #         # print("Direction for Joystick Y-axis:", direction_y)


    #         # The most aligned correction axes, invert if direction is negative
    #         aligned_axis_x = correction_axes[aligned_axis_index_x] 
    #         aligned_axis_y = correction_axes[aligned_axis_index_y]


    #         ## Taking the actual correction axeses (untill now we were dealing with projected vectors on the horizontal plane)
    #         ## Also this will handle cases like 45 degrees where joy axes can be mapped to one same correction axis, but the following step ensures at last it will be applied to the two axes again

    #         if (aligned_axis_x == projected_corr_x).all():
    #             aligned_axis_x = correction_axes[0]
    #             aligned_axis_y = correction_axes[1]
    #         else:
    #             aligned_axis_x = correction_axes[1]
    #             aligned_axis_y = correction_axes[0]

    #         self.prev_circle_type = "horizontal"

    #     # print("original correction axes", correction_axes[0], correction_axes[1])
    #     # print("rotated correction axes", correction_axes_rotated)
    #     # print("Aligned Axis for Joystick X-axis:", aligned_axis_x, "Direction identifier", direction_x)
    #     # print("Aligned Axis for Joystick Y-axis:", aligned_axis_y, "Direction identifier", direction_y)

    #     return aligned_axis_x, aligned_axis_y, direction_x, direction_y
    

    def correct_traj(self, s_model, i, PcurrG, QcurrG, 
                     q_weight, current_idx, numRepro, starting, crossSectionType, strategy, direction, trajectory_xyz, trajectory_q, 
                     cumulative_correction_time, cumulative_correction_distance, lio_cumulative_correction_distance):
        rospy.loginfo(f"  *Correction request at i={i}*")
        self.events_pub.publish(f"Correction request at i={i}")
        # Truncate model
        eN = s_model["eN"][i:]
        eB = s_model["eB"][i:]
        eT = s_model["eT"][i:]
        directrix = s_model["directrix"][i:]

        directrix_point = s_model["directrix"][i,]

        Rc = s_model["Rc"][i:]
        # print("Rc", Rc)
        # Get correction axes
        eX = s_model["x_corr_axes"][i:]
        eY = s_model["y_corr_axes"][i:]
        x_corr_axes = s_model["x_corr_axes"]
        y_corr_axes = s_model["y_corr_axes"]

        xyz_corr_y = s_model["xyz_corr_y"]
        vertical_start = s_model["vertical_start"]
        vertical_end = s_model["vertical_end"]

        from_front = True

        start_correction_time = rospy.Time.now()
        start_position = PcurrG
        if self.physical_robot: 
            # print("############################## here in")
            transform = self.tfBuffer.lookup_transform('LIO_robot_base_link', 'lio_tcp_link', rospy.Time(0), rospy.Duration(1.0))

            ##changed
            start_position_lio = np.array([[transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]])
            # start_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])

        
        rospy.loginfo("    correcting trajectory...")
        rospy.loginfo(f"      Before correction: {PcurrG}")


        # Define the fixed joystick axes in the global frame ASSUMIN WE ARE IN FRONT

        joy_x = self.joy_x # Joystick X-axis (global frame)
        joy_y = self.joy_y # Joystick Y-axis (global frame)


        while self.correction:
            # Integrate correction
            # raw_joystickDisplacement = self.x_corr*x_corr_axes[i,:] + self.y_corr*y_corr_axes[i,:]
            # print("input format", self.x_corr, self.y_corr)
            # print("x and y axes", x_corr_axes[i,:], y_corr_axes[i,:])
            # print("######### x and y after mult ", self.x_corr*x_corr_axes[i,:], self.y_corr*y_corr_axes[i,:])



            # ##### Test code #####

            aligned_axis_x, aligned_axis_y, direction_x, direction_y = self.map_input(x_corr_axes[i,:], y_corr_axes[i,:], joy_x, joy_y, s_model["eT"][i,:], directrix_point)

            aligned_axis_x = np.array(aligned_axis_x)
            aligned_axis_y = np.array(aligned_axis_y)

            # raw_joystickDisplacement = self.x_corr*x_corr_axes[i,:] + self.y_corr*y_corr_axes[i,:]
            # raw_joystickDisplacement = self.x_corr*aligned_axis_x* -direction_x  + self.y_corr*aligned_axis_y* direction_y   #works for the normal joystick
            raw_joystickDisplacement = self.x_corr*aligned_axis_x* direction_x  + self.y_corr*aligned_axis_y* direction_y


            ##### Test code ends here #####

            # raw_joystickDisplacement = self.x_corr*x_corr_axes[i,:] + self.y_corr*y_corr_axes[i,:]

            # if from_front:
            #     raw_joystickDisplacement = -self.x_corr*y_corr_axes[i,:] + self.y_corr*-x_corr_axes[i,:]
            # print ("raw_joystickDisplacement: ", raw_joystickDisplacement)
            ## Bring point to origin
            AcurrG = PcurrG - directrix[0] 
            # print("Directrix[0]: ", directrix[0])
            # print("AcurrG: ", AcurrG)
            ## Apply correction
            AcurrG_corr = AcurrG + raw_joystickDisplacement
            # print("AcurrG_corr: ", AcurrG_corr, "of norm: ", np.linalg.norm(AcurrG_corr))



            # ###########  Saturate correction and compute ratio #############
            # if np.linalg.norm(AcurrG_corr) > Rc[0] :   
            #     AcurrG_corr = Rc[0] * AcurrG_corr / np.linalg.norm(AcurrG_corr)
            #     Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
            #     # print("#### ratio", Ratio)
            #     # print(f"Rc[0]={Rc[0]} and (saturated) ratio={Ratio}")
            # else:
            #     Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
            #     # print(f"Rc[0]={Rc[0]} and (corr): ratio={Ratio}")

            # ################################################################


            ############ TO use the ealstic model of going outside the canal ########

            # ##Mofidy the below code so that only if a user put too much of an effort only they will be given the chance to go beyond the circle
            distance_from_center = np.linalg.norm(AcurrG_corr)

            elastic_constant = 0.8
            if distance_from_center > Rc[0]:
                self.out_of_canal = True
                elastic_extension = distance_from_center - Rc[0]
                Rc_adjusted = Rc[0] + elastic_constant * elastic_extension
                AcurrG_corr = Rc_adjusted * AcurrG_corr / distance_from_center  #vectorize
                AcurrG_corr = AcurrG_corr * Rc_adjusted / distance_from_center  #vectorize
                Ratio = 1.0
                print("Outside", Rc_adjusted)
            else:
                self.out_of_canal = False
                Ratio = distance_from_center / Rc[0]

            ################################################################


            ## Translate back to directrix point
            PcurrG_corr = AcurrG_corr + directrix[0]
            ## Change current point 
            PcurrG = PcurrG_corr
            self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
            self.pub_surface_pose(i, current_idx, PcurrG_corr, AcurrG_corr, QcurrG, q_weight, Ratio, "correction")
            self.correction = False
            rospy.sleep(0.2)
        rospy.loginfo(f"      After correction: {PcurrG}")
        correction_duration = rospy.Time.now() - start_correction_time
        cumulative_correction_time += correction_duration.to_sec()
        rospy.loginfo(f"    Correction duration: {correction_duration.to_sec()}")
        correction_distance = np.linalg.norm(PcurrG - start_position)
        if self.physical_robot:

            transform = self.tfBuffer.lookup_transform('LIO_robot_base_link', 'lio_tcp_link', rospy.Time(0), rospy.Duration(1.0))

            ##changed
            end_position_lio = np.array([[transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]])
            # end_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])
            
            lio_correction_distance = np.linalg.norm(start_position_lio-end_position_lio)
            lio_cumulative_correction_distance += lio_correction_distance
            rospy.loginfo(f"    Lio correction distance: {lio_correction_distance}")
            self.events_pub.publish(f"Lio correction distance: {lio_correction_distance}")
        cumulative_correction_distance += correction_distance
        rospy.loginfo(f"    Correction distance: {correction_distance}")
        self.events_pub.publish(f"Correction of {correction_distance} during {correction_duration.to_sec()}")
        self.pub_correction(i,current_idx, start_position, PcurrG, QcurrG, correction_distance, correction_duration.to_sec())
        # Get the right shape for 'reproduce' function
        PcurrG_corr = np.array([[PcurrG_corr[0], PcurrG_corr[1], PcurrG_corr[2]]])
        Ratio = np.array([Ratio])
        # Create new trajectory for remaining points
        small_model = {"eN":eN, "eB":eB, "eT":eT, "directrix":directrix, "Rc":Rc, "x_corr_axes":eX, "y_corr_axes":eY, "xyz_corr_y": xyz_corr_y, "vertical_start": vertical_start, "vertical_end": vertical_start }
        newTrajectory = reproduce(small_model, numRepro, starting, PcurrG_corr, Ratio, crossSectionType, strategy, direction, self.out_of_canal)
        self.out_of_canal = False
        
        trajectory_xyz[i:]= newTrajectory[0]                    
        rospy.loginfo(f"  -> New trajectory_xyz of length {len(newTrajectory[0])}  with first point: {newTrajectory[0][0]}")
        # Update visualisation
        self.clear_rviz_pub.publish("reproduced")
        self.visualise_trajectory(trajectory_xyz, trajectory_q )
        # return
        self.correction = False

        return trajectory_xyz, trajectory_q, PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance



    def correction_at_ends(self, s_model, PcurrG, QcurrG, t,
                     q_weight, current_idx, 
                     cumulative_correction_time, cumulative_correction_distance, lio_cumulative_correction_distance):
        
        ## Because of this function, the robot might stop in the middle, untill the user presses the Y button
 
        if current_idx == t["-1"]: #pick location
            rospy.loginfo(f"  *Correction request at start*")
            self.events_pub.publish(f"Correction request at start")

        if current_idx == t["1"]: #place location
            rospy.loginfo(f"  *Correction request at end*")
            self.events_pub.publish(f"Correction request at end")

        from_front = True
        

        directrix = s_model["directrix"][-1:]
        Rc = s_model["Rc"][-1:]
        
        directrix_point = s_model["directrix"][-1,]

        x_corr_axes = s_model["x_corr_axes"]
        y_corr_axes = s_model["y_corr_axes"]


        start_correction_time = rospy.Time.now()
        start_position = PcurrG

        if self.physical_robot: 

            transform = self.tfBuffer.lookup_transform('LIO_robot_base_link', 'lio_tcp_link', rospy.Time(0), rospy.Duration(1.0))

            ##changed
            start_position_lio = np.array([[transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]])
            # start_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])

        rospy.loginfo("    correcting trajectory...")
        rospy.loginfo(f"      Before correction: {PcurrG}")

        joy_x = self.joy_x # Joystick X-axis (global frame)
        joy_y = self.joy_y # Joystick Y-axis (global frame)

        while self.correction:
            # Integrate correction
            # raw_joystickDisplacement = self.x_corr*x_corr_axes[-1,:] + self.y_corr*y_corr_axes[-1,:]

            # if from_front:
            #     raw_joystickDisplacement = -self.x_corr*y_corr_axes[-1,:] + self.y_corr*-x_corr_axes[-1,:]

            aligned_axis_x, aligned_axis_y, direction_x, direction_y = self.map_input(x_corr_axes[-1,:], y_corr_axes[-1,:], joy_x, joy_y, s_model["eT"][-1,:], directrix_point)

            aligned_axis_x = np.array(aligned_axis_x)
            aligned_axis_y = np.array(aligned_axis_y)

            # raw_joystickDisplacement = self.x_corr*x_corr_axes[i,:] + self.y_corr*y_corr_axes[i,:]
            raw_joystickDisplacement = self.x_corr*aligned_axis_x* direction_x  + self.y_corr*aligned_axis_y* direction_y



            # print ("raw_joystickDisplacement: ", raw_joystickDisplacement)
            ## Bring point to origin
            AcurrG = PcurrG - directrix[0]
            # print("Directrix[0]: ", directrix[0])
            # print("AcurrG: ", AcurrG)
            ## Apply correction
            AcurrG_corr = AcurrG + raw_joystickDisplacement
            # print("AcurrG_corr: ", AcurrG_corr, "of norm: ", np.linalg.norm(AcurrG_corr))


            ## Saturate correction and compute ratio
            # if np.linalg.norm(AcurrG_corr) > Rc[0] : 
            #     AcurrG_corr = Rc[0] * AcurrG_corr / np.linalg.norm(AcurrG_corr)
            #     Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
            #     # print(f"Rc[0]={Rc[0]} and (saturated) ratio={Ratio}")
            # else:
            #     Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
            #     # print(f"Rc[0]={Rc[0]} and (corr): ratio={Ratio}")


            ############ TO use the ealstic model of going outside the canal ########

            # ##Mofidy the below code so that only if a user put too much of an effort only they will be given the chance to go beyond the circle
            distance_from_center = np.linalg.norm(AcurrG_corr)

            elastic_constant = 0.8
            if distance_from_center > Rc[0]:
                self.out_of_canal = True
                elastic_extension = distance_from_center - Rc[0]
                Rc_adjusted = Rc[0] + elastic_constant * elastic_extension
                AcurrG_corr = Rc_adjusted * AcurrG_corr / distance_from_center  #vectorize
                Ratio = 1.0
                print("Outside", Rc_adjusted)
            else:
                self.out_of_canal = False
                Ratio = distance_from_center / Rc[0]

            ################################################################


    




            ## Translate back to directrix point
            PcurrG_corr = AcurrG_corr + directrix[0]
            ## Change current point 
            PcurrG = PcurrG_corr
            self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
            self.pub_surface_pose(-1, current_idx, PcurrG_corr, AcurrG_corr, QcurrG, q_weight, Ratio, "correction")
            self.correction = False
            rospy.sleep(0.2)
            
        rospy.loginfo(f"      After correction: {PcurrG}")
        correction_duration = rospy.Time.now() - start_correction_time
        cumulative_correction_time += correction_duration.to_sec()
        rospy.loginfo(f"    Correction duration: {correction_duration.to_sec()}")
        correction_distance = np.linalg.norm(PcurrG - start_position)
        if self.physical_robot:
                        
            transform = self.tfBuffer.lookup_transform('LIO_robot_base_link', 'lio_tcp_link', rospy.Time(0), rospy.Duration(1.0))

            ##changed
            end_position_lio = np.array([[transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]])
            # end_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])
            
            lio_correction_distance = np.linalg.norm(start_position_lio-end_position_lio)
            lio_cumulative_correction_distance += lio_correction_distance
            rospy.loginfo(f"    Lio correction distance: {lio_correction_distance}")
            self.events_pub.publish(f"Lio correction distance: {lio_correction_distance}")
        cumulative_correction_distance += correction_distance
        rospy.loginfo(f"    Correction distance: {correction_distance}")
        self.events_pub.publish(f"Correction of {correction_distance} during {correction_duration.to_sec()}")

        self.pub_correction(-1,current_idx, start_position, PcurrG, QcurrG, correction_distance, correction_duration.to_sec())

       
        return PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance, lio_cumulative_correction_distance


    def update_states(self, goal, prev_goal, strategy, current_idx, t):

        if self.change_direction_request and current_idx != t["1"] and current_idx!=t["-1"]: #if we are at the start or end, we don't need to do any strategy changes or goal changes
                rospy.loginfo("   JOYSTICK change direction request")
                _ = goal
                goal = prev_goal
                prev_goal = _
                strategy = "fixed"
                self.change_direction_request = False
        # elif self.gripper_change_direction_request:
        #     rospy.loginfo("   GRIPPER change direction request")
        #     if goal == "HOME":
        #         pass
        #     else:
        #         prev_goal = goal
        #         goal = "HOME"
        #         strategy = "convergent"
        #     self.gripper_change_direction_request = False
        elif goal == "PICK": 
            prev_goal = goal
            goal = "HOME"
            strategy = "convergent"
        elif goal == "PLACE":
            prev_goal = goal
            goal = "HOME"
            strategy = "convergent"
        elif goal == "HOME" and prev_goal == "PICK":
            prev_goal = goal
            goal = "PLACE"
            strategy = "fixed"
        elif goal == "HOME" and prev_goal == "PLACE":
            prev_goal = goal
            goal = "PICK"
            strategy = "fixed"

        return goal, prev_goal, strategy
    

    def update_states_at_the_end(self, goal, prev_goal, strategy, current_idx, t):

      
        # elif self.gripper_change_direction_request:
        #     rospy.loginfo("   GRIPPER change direction request")
        #     if goal == "HOME":
        #         pass
        #     else:
        #         prev_goal = goal
        #         goal = "HOME"
        #         strategy = "convergent"
        #     self.gripper_change_direction_request = False
        if goal == "PICK": 
            prev_goal = goal
            goal = "HOME"
            strategy = "convergent"
        elif goal == "PLACE":
            prev_goal = goal
            goal = "HOME"
            strategy = "convergent"
        elif goal == "HOME" and prev_goal == "PICK":
            prev_goal = goal
            goal = "HOME"
            strategy = "convergent"
        elif goal == "HOME" and prev_goal == "PLACE":
            prev_goal = goal
            goal = "HOME"
            strategy = "convergent"

        return goal, prev_goal, strategy


    # def run(self):
    #     # Specify data directory and task
    #     data_dir = "/home/shalutha/TLGC_data"
    #     task = input("What is the name of the task? ")
    #     model_dir = data_dir +f"/{task}/model"
    #     raw_dir = data_dir + f"/{task}/record-raw"
    #     processed_dir = data_dir + f"/{task}/record-processed"


    #     # Get model & demos
    #     raw_demos_xyz, raw_demos_q = self.get_raw_demos(raw_dir)
    #     processed_demos_xyz, processed_demos_q = self.get_processed_demos(processed_dir)
    #     model = self.get_model(model_dir)
    #     if self.weighted_pose: 
    #         q_weights = model["q_weights"]
    #     else:
    #         q_weights = np.ones((model["q"].shape[0], 1))

    #     self.clear_rviz_pub.publish("all")

    #     self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
    #     self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
    #     # rospy.loginfo("Loading demonstration data visualisation...")
    #     # self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
    #     # self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
    #     rospy.loginfo("Loading GC visualisation...")
    #     # self.visualise_gc(model["GC"])
    #     self.visualise_gc(model["GC"][-50:,:,:])
    #     self.visualise_gc(model["GC"][:50,:,:])
        
    #     rospy.loginfo("Loading directrix visualisation...")
    #     self.visualise_directrix(model["directrix"], model["q"])
    #     rospy.loginfo("Loading correction axes visualisation...")
    #     self.visualise_correction_axes(model["x_corr_axes"], model["y_corr_axes"], model["directrix"])

    #     # rospy.loginfo("Loading eT visualisation...")
    #     # self.visualise_TNB_axes(model["eT"], model["directrix"], "eT")
    #     # rospy.loginfo("Loading eN visualisation...")
    #     # self.visualise_TNB_axes(model["eN"], model["directrix"], "eN")
    #     # rospy.loginfo("Loading eB visualisation...")
    #     # self.visualise_TNB_axes(model["eB"], model["directrix"], "eB")
    #     # rospy.loginfo("Visualisation loaded.")

    #     pick_idx = 0
    #     place_idx = model["directrix"].shape[0] - 1 # -1 needed to convert length to idx
    #     mid_idx = int(place_idx/2)
    #     t = {"-1":pick_idx, "0": mid_idx, "1":place_idx}
        
    #     restart = True
    #     prev_goal = "HOME"
    #     goal = "PICK"
    #     start_idx = t["0"]
    #     end_idx = t["-1"]
    #     current_idx = start_idx
    #     numRepro = 1
    #     starting = np.ones((1, numRepro), dtype=int) 
    #     crossSectionType = "circle"
    #     strategy = "fixed"
    #     s_model, direction = self.slice_model(model,start_idx, end_idx)
    #     # print(f"{s_model['x_corr_axes'].shape}, {s_model['y_corr_axes'].shape}, {s_model['eT'].shape}")
    #     # initPoints, Ratio = randomInitialPoints(s_model, numRepro, crossSectionType)
    #     # print(f"Initpoint {initPoints - s_model['directrix'][0,:]}")
    #     initPoints = np.array([[model["directrix"][start_idx,:]]])
    #     Ratio = np.array([0.])
    #     first = True

    #     start_task_time = rospy.Time.now()
    #     cumulative_correction_time = 0
    #     cumulative_correction_distance = 0
    #     lio_cumulative_correction_distance = 0

        
    #     while restart:

    #         # Generate trajectory from a random starting point
    #         print("----------------------------------------------------------------------------")
    #         rospy.loginfo("Reproduce trajectory")
    #         rospy.loginfo(f"  Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-s_model['directrix'][0,:])}")
    #         rospy.loginfo(f"  Rc[0]={s_model['Rc'][0]}. Starting ratio={Ratio}")
            
    #         # print("#######################", s_model["xyz_corr_y"])
    #         repro_trajectories = reproduce(s_model, numRepro, starting, initPoints, Ratio, crossSectionType, strategy, direction)
    #         trajectory_xyz = repro_trajectories[0]
    #         # print(trajectory_xyz)
    #         trajectory_q = s_model["q"]
    #         rospy.loginfo(f"  First trajectory point {trajectory_xyz[0]}")

    #         if trajectory_xyz.shape[0] != trajectory_q.shape[0]:
    #             rospy.logerr("main: trajectory_xyz.shape[0]!= trajectory_q.shape[0]")
    #         self.clear_rviz_pub.publish("reproduced")
    #         self.visualise_trajectory(trajectory_xyz, trajectory_q)
    #         nb_points = trajectory_xyz.shape[0]
            
    #         # Move Lio to starting point
    #         if self.physical_robot : self.myp_app_pub.publish("start") 
    #         PcurrG = trajectory_xyz[0,:]
    #         QcurrG = trajectory_q[0,:]
    #         q_weight = q_weights[0,:]

    #         self.pub_cmd_pose(PcurrG, QcurrG, q_weight)

    #         if first:
    #             rospy.loginfo("Press START button on joystick ...")
    #             while not self.start:
    #                 rospy.sleep(0.1)
    #             self.events_pub.publish("Start.")
    #             self.events_pub.publish(f"  Go from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")
    #             rospy.sleep(0.1)
    #             first = False

            
    #         rospy.loginfo("Running trajectory ...")

    #         self.task_done_and_reverse = False
    #         self.change_direction_request = False

    #         for i in range(nb_points):

    #             if self.terminate:
    #                 self.events_pub.publish("Terminate.")
    #                 break
                

    #             ## Gripper related
    #             while self.gripper_state == "moving":
    #                 rospy.sleep(0.01)

    #             if self.gripper_change_direction_request:
    #                 if current_idx != t["1"] and current_idx!=t["-1"] and current_idx != t["0"]:
    #                     self.events_pub.publish("Gripper change direction request.")
    #                     break
    #                 else:
    #                     self.gripper_change_direction_request = False

                
    #             ## Arm movement related
    #             if current_idx == t["-1"] or current_idx == t["1"]:
    #                 print ("Reached start or end, waiting for user command to go back", i)
    #                 while not self.change_direction_request:
    #                     if self.correction:
    #                         trajectory_xyz, trajectory_q, PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance = self.correct_traj(s_model,i, PcurrG, QcurrG, q_weight, 
    #                                                                                                           current_idx, numRepro, starting, 
    #                                                                                                           crossSectionType, strategy, direction, 
    #                                                                                                           trajectory_xyz, trajectory_q, cumulative_correction_time,
    #                                                                                                           cumulative_correction_distance)
    #                     self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
    #                     AcurrG = PcurrG - model["directrix"][current_idx,:]
    #                     Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
    #                     self.pub_surface_pose(i, current_idx, PcurrG, AcurrG, QcurrG, q_weight, Ratio, "")
    #                     # i designates the NEXT idx of the robot
    #                     if i != 0: current_idx += 1*direction # Designates the idx AT which the robot is located
    #                     self.rate.sleep()

    #             else:
    #                 if self.change_direction_request:
    #                     if current_idx != t["1"] and current_idx!=t["-1"] and current_idx != t["0"]: #if not either of those edge cases we will break the loop
    #                         self.events_pub.publish("User change direction request.")
    #                         break
    #                     else:
    #                         self.change_direction_request = False
                        
    

    #             PcurrG = trajectory_xyz[i,:]
    #             QcurrG = trajectory_q[i,:]
    #             q_weight = q_weights[i,:]

    #             # if self.correction and i != nb_points-1: #need to change this last condition as we may want to give corrections on the last frame
    #             if self.correction:
    #                         trajectory_xyz, trajectory_q, PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance = self.correct_traj(s_model,i, PcurrG, QcurrG, q_weight, 
    #                                                                                                           current_idx, numRepro, starting, 
    #                                                                                                           crossSectionType, strategy, direction, 
    #                                                                                                           trajectory_xyz, trajectory_q, cumulative_correction_time,
    #                                                                                                           cumulative_correction_distance)
    #             self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
    #             AcurrG = PcurrG - model["directrix"][current_idx,:]
    #             Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
    #             self.pub_surface_pose(i, current_idx, PcurrG, AcurrG, QcurrG, q_weight, Ratio, "")
    #             # i designates the NEXT idx of the robot
    #             if i != 0: current_idx += 1*direction # Designates the idx AT which the robot is located
    #             self.rate.sleep()

            
    #         if self.terminate:
    #             if self.physical_robot : self.myp_app_pub.publish("stop")
    #             task_duration = (rospy.Time.now() - start_task_time).to_sec()
    #             rospy.loginfo(f"Total task time: {task_duration} seconds")
    #             rospy.loginfo(f"Total task time*: {task_duration-cumulative_correction_time} seconds")
    #             rospy.loginfo(f"Total correction time: {cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")
    #             rospy.loginfo(f"Total correction time*: {cumulative_correction_time} seconds ({(cumulative_correction_time/(task_duration-cumulative_correction_time))*100}%)")
    #             rospy.loginfo(f"Total correction distance: {cumulative_correction_distance} meters")
    #             rospy.loginfo(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters")
    #             self.events_pub.publish(f"Total task time: {task_duration} seconds")
    #             self.events_pub.publish(f"Total task time*: {task_duration-cumulative_correction_time} seconds")
    #             self.events_pub.publish(f"Total correction time: {cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")
    #             self.events_pub.publish(f"Total correction time*: {cumulative_correction_time} seconds ({(cumulative_correction_time/(task_duration-cumulative_correction_time))*100}%)")
    #             self.events_pub.publish(f"Total correction distance: {cumulative_correction_distance} meters")
    #             self.events_pub.publish(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters")
    #             return
            

    #         rospy.loginfo("End of trajectory.")
    #         rospy.loginfo("Compute next section")
    #         self.events_pub.publish("End of section, compute next")
    #         rospy.loginfo(f"PREVIOUS: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")

    #         goal, prev_goal, strategy = self.update_states(goal, prev_goal, strategy)


    #         start_idx = current_idx
    #         if goal == "PICK" : end_idx = t["-1"]
    #         elif goal == "PLACE" : end_idx = t["1"]
    #         elif goal == "HOME" : end_idx = t["0"]
    #         rospy.loginfo(f"  UPDATED: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")
    #         self.events_pub.publish("--------------------------------------------------------------------")
    #         self.events_pub.publish(f"Go from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")


    #         initPoints = np.array([PcurrG])
    #         AcurrG = PcurrG - model["directrix"][current_idx,:]
    #         Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
    #         rospy.loginfo(f"  Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-model['directrix'][current_idx,:])}")
    #         rospy.loginfo(f"  Rc[current_idx]={model['Rc'][current_idx]}. Starting ratio={Ratio}")
    #         s_model, direction = self.slice_model(model, start_idx, end_idx )




    
    def run(self):
        # Specify data directory and task
        data_dir = "/home/shalutha/TLGC_data"
        task = input("What is the name of the task? ")
        model_dir = data_dir +f"/{task}/model"
        raw_dir = data_dir + f"/{task}/record-raw"
        processed_dir = data_dir + f"/{task}/record-processed"

        # self.init_pose()


        # Get model & demos
        raw_demos_xyz, raw_demos_q = self.get_raw_demos(raw_dir)
        processed_demos_xyz, processed_demos_q = self.get_processed_demos(processed_dir)
        model = self.get_model(model_dir)
        if self.weighted_pose: 
            q_weights = model["q_weights"]
        else:
            q_weights = np.ones((model["q"].shape[0], 1))

        self.clear_rviz_pub.publish("all")

        # self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
        # self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
        # rospy.loginfo("Loading demonstration data visualisation...")
        # self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
        # self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )

        rospy.loginfo("Loading GC visualisation...")
        self.visualise_gc(model["GC"])
        # self.visualise_gc(model["GC"][120:220,:,:])
        # self.visualise_gc(model["GC"][60:80,:,:])
        
        # self.visualise_gc(model["GC"][-50:,:,:])
        # self.visualise_gc(model["GC"][:50,:,:])
        
        rospy.loginfo("Loading directrix visualisation...")
        # self.visualise_directrix(model["directrix"], model["q"])

        rospy.loginfo("Loading correction axes visualisation...")
        # self.visualise_correction_axes(model["x_corr_axes"], model["y_corr_axes"], model["directrix"])

        # rospy.sleep(5)

        # rospy.loginfo("Loading eT visualisation...")
        # self.visualise_TNB_axes(model["eT"], model["directrix"], "eT")
        # rospy.loginfo("Loading eN visualisation...")
        # self.visualise_TNB_axes(model["eN"], model["directrix"], "eN")
        # rospy.loginfo("Loading eB visualisation...")
        # self.visualise_TNB_axes(model["eB"], model["directrix"], "eB")
        # rospy.loginfo("Visualisation loaded.")


        if self.task == "marshmellow":

            ###### ONLY FOR MARSHMELLOW TASK ######
            pick_idx = model["directrix"].shape[0] - 1
            place_idx = 0
            mid_idx = int(pick_idx/2)
            t = {"-1":pick_idx, "0": mid_idx, "1":place_idx}

            restart = True
            prev_goal = "PICK"
            goal = "HOME"
            start_idx = t["-1"]
            end_idx = t["0"]
            current_idx = start_idx
            numRepro = 1
            starting = np.ones((1, numRepro), dtype=int)
            crossSectionType = "circle"
            strategy = "convergent"

            #################################################

        else:
        
            ##### For All Other Tasks Except the Marshmellow Task ######
            pick_idx = 0
            place_idx = model["directrix"].shape[0] - 1 # -1 needed to convert length to idx
            mid_idx = int(place_idx/2)
            t = {"-1":pick_idx, "0": mid_idx, "1":place_idx}
            
            restart = True
            prev_goal = "HOME"
            goal = "PICK"
            start_idx = t["0"]
            end_idx = t["-1"]
            current_idx = start_idx
            numRepro = 1
            starting = np.ones((1, numRepro), dtype=int) 
            crossSectionType = "circle"
            strategy = "fixed"

            ##########################################################





        s_model, direction = self.slice_model(model,start_idx, end_idx)
        # print(f"{s_model['x_corr_axes'].shape}, {s_model['y_corr_axes'].shape}, {s_model['eT'].shape}")
        # initPoints, Ratio = randomInitialPoints(s_model, numRepro, crossSectionType)
        # print(f"Initpoint {initPoints - s_model['directrix'][0,:]}")

        # if self.task == "marshmellow":
        #     initPoints = np.array([])
        # else:
        initPoints = np.array([[model["directrix"][start_idx,:]]])
        Ratio = np.array([0.])
        first = True

        # start_task_time = rospy.Time.now()
        cumulative_correction_time = 0
        cumulative_correction_distance = 0
        lio_cumulative_correction_distance = 0

        
        while restart:

            # Generate trajectory from random a starting point
            print("----------------------------------------------------------------------------")
            rospy.loginfo("Reproduce trajectory")
            rospy.loginfo(f"  Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-s_model['directrix'][0,:])}")
            rospy.loginfo(f"  Rc[0]={s_model['Rc'][0]}. Starting ratio={Ratio}")
            
            repro_trajectories = reproduce(s_model, numRepro, starting, initPoints, Ratio, crossSectionType, strategy, direction, self.out_of_canal)
            trajectory_xyz = repro_trajectories[0]
            self.out_of_canal = False  #we need to do this as after reproduction the points will be inside
    
            trajectory_q = s_model["q"]
            rospy.loginfo(f"  First trajectory point {trajectory_xyz[0]}")

            if trajectory_xyz.shape[0] != trajectory_q.shape[0]:
                rospy.logerr("main: trajectory_xyz.shape[0]!= trajectory_q.shape[0]")
            self.clear_rviz_pub.publish("reproduced")
            self.visualise_trajectory(trajectory_xyz, trajectory_q)
            nb_points = trajectory_xyz.shape[0]
            
            # Move Lio to starting point
            if self.physical_robot : self.myp_app_pub.publish("start")
            PcurrG = trajectory_xyz[0,:]
            QcurrG = trajectory_q[0,:]
            q_weight = q_weights[0,:]

            # start_PcurrG = PcurrG
            # start_QcurrG = QcurrG
            # start_q_weight = q_weight

            self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
            if first:
                rospy.loginfo("Press START button on joystick ...")
                while not self.start:
                    rospy.sleep(0.1)

                rospy.loginfo("##### started ####")
                start_task_time = rospy.Time.now()
                self.events_pub.publish("Start.")
                self.events_pub.publish(f"  Go from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")
                rospy.sleep(0.1)
                first = False

            
            rospy.loginfo("Running trajectory ...")

            self.task_done_and_reverse = False
            self.change_direction_request = False


            for i in range(nb_points):

                # directrix_point = s_model["directrix"][i,:]
                # print("############ current directrix_point", directrix_point)
                

                if self.terminate:
                    self.events_pub.publish("Terminate.")
                    break
                        
                while self.gripper_state == "moving":
                    rospy.sleep(0.01)

                if self.gripper_change_request:
                    rospy.sleep(1.2)
                    self.gripper_change_request = False


                #TODO CHECK THE BELOW COMMENTED PART WILL TAKE CARE THE AUTOMATIC DIRECTION REQUEST
                # if self.gripper_change_direction_request:
                #     rospy.loginfo("Im here")
                #     if current_idx != t["1"] and current_idx!=t["-1"] and current_idx != t["0"]:
                #         self.events_pub.publish("Gripper change direction request.")
                #         break
                #     else:
                #         self.gripper_change_direction_request = False
                    
    
                
                # if current_idx == t["-1"] or current_idx == t["1"]: ## If we reached here, that means the canal has been shifted the other way
                #     #therefore we will start from this location and NOT ENDING

                #     # print ("Reached start or end, waiting for user command to go back", i, "self.task_done_and_reverse", self.task_done_and_reverse, "direction", direction)
                #     print ("Reached start or end, waiting for user command to go back")

                
                    # while not self.change_direction_request:
                    #     # rospy.sleep(0.01)
                    #     if self.correction:
                    
                    #         trajectory_xyz, trajectory_q, PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance = self.correct_traj(s_model,i, PcurrG, QcurrG, q_weight, 
                    #                                                                                           current_idx, numRepro, starting, 
                    #                                                                                           crossSectionType, strategy, direction, 
                    #                                                                                           trajectory_xyz, trajectory_q, cumulative_correction_time,
                    #                                                                                           cumulative_correction_distance, lio_cumulative_correction_distance )
                    #     self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
                    #     AcurrG = PcurrG - model["directrix"][current_idx,:]
                    #     Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
                    #     self.pub_surface_pose(i, current_idx, PcurrG, AcurrG, QcurrG, q_weight, Ratio, "")
                    #     # i designates the NEXT idx of the robot
                        
                    #     self.rate.sleep()

                    # current_idx += 1*direction # Designates the idx AT which the robot is located
                    # self.change_direction_request = False
                
                # else:

                if self.change_direction_request:
                    if current_idx != t["1"] and current_idx!=t["-1"] and current_idx != t["0"]:
                        self.events_pub.publish("User change direction request.")
                        break
                    else:
                        self.change_direction_request = False

                PcurrG = trajectory_xyz[i,:]
                QcurrG = trajectory_q[i,:]
                q_weight = q_weights[i,:]

                if self.correction and i != nb_points-1:
                    # print("########### current idx", current_idx)
                    
                    trajectory_xyz, trajectory_q, PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance = self.correct_traj(s_model,i, PcurrG, QcurrG, q_weight, 
                                                                                                            current_idx, numRepro, starting, 
                                                                                                            crossSectionType, strategy, direction, 
                                                                                                        trajectory_xyz, trajectory_q, cumulative_correction_time, cumulative_correction_distance,
                                                                                                        lio_cumulative_correction_distance)
                self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
                AcurrG = PcurrG - model["directrix"][current_idx,:]
                Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
                self.pub_surface_pose(i, current_idx, PcurrG, AcurrG, QcurrG, q_weight, Ratio, "")
                # i designates the NEXT idx of the robot
                if i != 0: current_idx += 1*direction # Designates the idx AT which the robot is located
                self.rate.sleep()

            # print("current index", current_idx, "t[1] = ", t["1"] ,"t[-1] = " , t["-1"])

            if current_idx == t["-1"] or current_idx == t["1"]:

                while not self.change_direction_request:

                    # q_weight = q_weights[current_idx,:]  #q_weights will not change anywhere
                    # QcurrG = trajectory_q[current_idx,:] #trajector_q will not change anywhere

                    if self.terminate:
                        self.events_pub.publish("Terminate.")
                        break

                    # rospy.sleep(0.01)
                    if self.correction:  ## Need to give the previous QcurrG and q_weight

                        PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance, lio_cumulative_correction_distance = self.correction_at_ends(s_model,PcurrG, QcurrG, t, q_weight, current_idx, cumulative_correction_time,
                                                                                                            cumulative_correction_distance, lio_cumulative_correction_distance)
                
                        # trajectory_xyz, trajectory_q, PcurrG, QcurrG, q_weight, Ratio, cumulative_correction_time, cumulative_correction_distance = self.correct_traj(s_model,i, PcurrG, QcurrG, q_weight, 
                        #                                                                                     current_idx, numRepro, starting, 
                        #                                                                                     crossSectionType, strategy, direction, 
                        #                                                                                     trajectory_xyz, trajectory_q, cumulative_correction_time,
                        #                                                                                     cumulative_correction_distance, lio_cumulative_correction_distance )
                    
                        # print("PcurrG", PcurrG, "QcurrG", QcurrG, "q_weight", q_weight)

                    if self.task == "marshmellow" and self.marsh_selected: #TODO might need to add this even in the middle of the canal rather than waiting until the last cross section
                        # print("PcuurG", PcurrG)
                        PcurrG[2] = PcurrG[2] - 0.03
                        # print("#################### After PcuurG", PcurrG)
                        self.terminate = True

                    self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
                    AcurrG = PcurrG - model["directrix"][current_idx,:]

                    if self.out_of_canal:
                        Ratio = np.array([1.0])
                    else:
                        Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
                    self.pub_surface_pose(current_idx, current_idx, PcurrG, AcurrG, QcurrG, q_weight, Ratio, "")
                    # print("################### PcurrG", PcurrG)
                self.change_direction_request = False

            # PcurrG = trajectory_xyz[i,:]
            # QcurrG = trajectory_q[i,:]
            # q_weight = q_weights[i,:]
                
            
            if self.terminate:

                rospy.loginfo("###################################################")
                if self.physical_robot : self.myp_app_pub.publish("stop")
                task_duration = (rospy.Time.now() - start_task_time).to_sec()
                rospy.loginfo(f"Total task time: {task_duration} seconds")
                rospy.loginfo(f"Total correction time: {cumulative_correction_time} seconds")

                rospy.loginfo(f"Total correction distance: {cumulative_correction_distance} meters")
                rospy.loginfo(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters")

                # rospy.loginfo(f"Total task time*: {task_duration-cumulative_correction_time} seconds")
                # rospy.loginfo(f"Total correction time: {cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")
                rospy.loginfo(f"Correction time as a percentage from total:{cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")

                # rospy.loginfo(f"Total correction time*: {cumulative_correction_time} seconds ({(cumulative_correction_time/(task_duration-cumulative_correction_time))*100}%)")

                self.events_pub.publish(f"Total task time: {task_duration} seconds")
                self.events_pub.publish(f"Total correction time: {cumulative_correction_time} seconds")

                self.events_pub.publish(f"Total correction distance: {cumulative_correction_distance} meters")
                self.events_pub.publish(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters")

                self.events_pub.publish(f"Correction time as a percentage from total:{cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")
                
                rospy.loginfo("###################################################")


                file_path = '/home/shalutha/geosacs_ws/src/geosacs/geosacs/data/experiment_data_geosacsv2.txt'

                # Write the values to the file
                with open(file_path, 'a') as file:
                    file.write(f"Total task time: {task_duration} seconds\n")
                    file.write(f"Total correction time: {cumulative_correction_time} seconds\n")
                    file.write(f"Total correction distance: {cumulative_correction_distance} meters\n")
                    file.write(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters\n")
                    file.write(f"Correction time as a percentage from total:  ({(cumulative_correction_time/task_duration)*100}%)\n")
                    file.write("\n")  # Adding a newline for separation between entries
                
                # self.events_pub.publish(f"Total task time: {task_duration} seconds")
                # self.events_pub.publish(f"Total task time*: {task_duration-cumulative_correction_time} seconds")
                # self.events_pub.publish(f"Total correction time: {cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")
                # self.events_pub.publish(f"Total correction time*: {cumulative_correction_time} seconds ({(cumulative_correction_time/(task_duration-cumulative_correction_time))*100}%)")
                # self.events_pub.publish(f"Total correction distance: {cumulative_correction_distance} meters")
                # self.events_pub.publish(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters")

                
                rospy.loginfo("task ended, moving to the intial task position")
                # self.pub_cmd_pose(start_PcurrG, start_QcurrG, start_q_weight)

                rospy.sleep(3)


                self.events_pub.publish("End of section, compute next")
                # rospy.loginfo(f"PREVIOUS: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")

                goal, prev_goal, strategy = self.update_states_at_the_end(goal, prev_goal, strategy, current_idx, t)

                start_idx = current_idx
                if goal == "PICK" : end_idx = t["-1"]
                elif goal == "PLACE" : end_idx = t["1"]
                elif goal == "HOME" : end_idx = t["0"]
                rospy.loginfo(f"  UPDATED: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}. Strategy : {strategy}")
                self.events_pub.publish("--------------------------------------------------------------------")
                self.events_pub.publish(f"Go from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")


                initPoints = np.array([PcurrG])
                AcurrG = PcurrG - model["directrix"][current_idx,:]
                Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
                # rospy.loginfo(f"  Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-model['directrix'][current_idx,:])}")
                # rospy.loginfo(f"  Rc[current_idx]={model['Rc'][current_idx]}. Starting ratio={Ratio}")
                s_model, direction = self.slice_model(model, start_idx, end_idx )


                repro_trajectories = reproduce(s_model, numRepro, starting, initPoints, Ratio, crossSectionType, strategy, direction, self.out_of_canal)
                trajectory_xyz = repro_trajectories[0]
                self.out_of_canal = False

                trajectory_q = s_model["q"]
                self.clear_rviz_pub.publish("reproduced")
                self.visualise_trajectory(trajectory_xyz, trajectory_q)

                nb_points = trajectory_xyz.shape[0]

                if self.physical_robot : self.myp_app_pub.publish("start")
                PcurrG = trajectory_xyz[0,:]
                QcurrG = trajectory_q[0,:]
                q_weight = q_weights[0,:]

                self.pub_cmd_pose(PcurrG, QcurrG, q_weight)

                for i in range(nb_points):
                    PcurrG = trajectory_xyz[i,:]
                    QcurrG = trajectory_q[i,:]
                    q_weight = q_weights[i,:]
                    
                    self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
                    AcurrG = PcurrG - model["directrix"][current_idx,:]
                    Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
                    self.pub_surface_pose(i, current_idx, PcurrG, AcurrG, QcurrG, q_weight, Ratio, "")
                    # i designates the NEXT idx of the robot
                    if i != 0: current_idx += 1*direction # Designates the idx AT which the robot is located
                    self.rate.sleep()

                return
            

            rospy.loginfo("End of trajectory.")
            rospy.loginfo("Compute next section")
            self.events_pub.publish("End of section, compute next")
            rospy.loginfo(f"PREVIOUS: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")

            goal, prev_goal, strategy = self.update_states(goal, prev_goal, strategy, current_idx, t)

            start_idx = current_idx
            if goal == "PICK" : end_idx = t["-1"]
            elif goal == "PLACE" : end_idx = t["1"]
            elif goal == "HOME" : end_idx = t["0"]
            rospy.loginfo(f"  UPDATED: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}. Strategy : {strategy}")
            self.events_pub.publish("--------------------------------------------------------------------")
            self.events_pub.publish(f"Go from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")


            initPoints = np.array([PcurrG])
            AcurrG = PcurrG - model["directrix"][current_idx,:]

            if self.out_of_canal:
                Ratio = np.array([1.0])
            else:
                Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
            rospy.loginfo(f"  Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-model['directrix'][current_idx,:])}")
            rospy.loginfo(f"  Rc[current_idx]={model['Rc'][current_idx]}. Starting ratio={Ratio}")
            s_model, direction = self.slice_model(model, start_idx, end_idx )




if __name__ == "__main__":
    my_node = MainNode()  
    my_node.run()