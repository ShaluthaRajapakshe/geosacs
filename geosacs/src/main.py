#!/usr/bin/env python
import rospy
import os
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState
from geosacs.msg import WeightedPose, Correction, SurfacePose
from randomInitialPoints import randomInitialPoints
from reproduce import reproduce


class MainNode(): 
    def __init__(self):
        rospy.init_node("main_node") 

        # Variables
        self.ratio = None
        self.x_corr = None
        self.y_corr = None
        self.rate = rospy.Rate(17)  # (4;25) ()
        self.frame = "LIO_base_link"
        self.correction = False
        self.physical_robot = rospy.get_param("physical_robot")
        self.weighted_pose =  True
        self.change_direction_request = False
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.terminate = False
        self.gripper_state = ""
        self.gripper_change_direction_request = False
        self.prev_gripper_angle = None
        self.gripper_states = []
        self.start = False

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

        

        self.commanded_pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=10)
        self.weighted_pose_pub = rospy.Publisher("/weighted_pose", WeightedPose, queue_size=10)
        self.myp_app_pub = rospy.Publisher("/myp_manager/app_control", String, queue_size=2)
        self.gripper_state_pub =rospy.Publisher("/gripper_state", String, queue_size=10)


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


        

    def joy_cb(self, msg):
        
        if msg.axes[0] != 0 or msg.axes[1] != 0:
            self.x_corr = msg.axes[0]/150
            self.y_corr = msg.axes[1]/150
            self.correction = True
            return # cannot do corrections & change direction at the same time

        # Filter buttons input
        current_buttons = list(msg.buttons)
        if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
        else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
        self.previous_buttons = current_buttons
        if changed_buttons[3] != 0:
            # print("CHANGE DIRECTION")
            self.change_direction_request = True
            return
        if changed_buttons[7] != 0:
            # print("STOP")
            self.terminate = True
        if changed_buttons[6] != 0:
            # print("START")
            self.start = True
        if changed_buttons[0] != 0:
            self.gripper_state_pub.publish("toggle_gripper")
        
            
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
        
        rospy.loginfo(f"Slice model:")
        if start_idx>end_idx: 
            direction = -1
        elif end_idx > start_idx: 
            direction = 1
        else : rospy.logerr("slice_model: edge case not addressed")

        rospy.loginfo(f"  Slicing model from {start_idx} to {end_idx} in {direction} direction.")
        
        keep_idxs = np.arange(start_idx, end_idx+1*direction, direction)
        rospy.loginfo(f"  Slice model to {keep_idxs.shape[0]} points")
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

        rospy.loginfo(f"  Sliced model directrix shape {sliced_model['directrix'].shape}")

        return sliced_model, direction

    
    def run(self):
        # Specify data directory and task
        data_dir = "/home/shalutha/TLGC_data"
        task = input("What is the name of the task? ")
        model_dir = data_dir +f"/{task}/model"
        raw_dir = data_dir + f"/{task}/record-raw"
        processed_dir = data_dir + f"/{task}/record-processed"


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
        
        rospy.loginfo("Loading directrix visualisation...")
        self.visualise_directrix(model["directrix"], model["q"])
        rospy.loginfo("Loading correction axes visualisation...")
        self.visualise_correction_axes(model["x_corr_axes"], model["y_corr_axes"], model["directrix"])

        # rospy.loginfo("Loading eT visualisation...")
        # self.visualise_TNB_axes(model["eT"], model["directrix"], "eT")
        # rospy.loginfo("Loading eN visualisation...")
        # self.visualise_TNB_axes(model["eN"], model["directrix"], "eN")
        # rospy.loginfo("Loading eB visualisation...")
        # self.visualise_TNB_axes(model["eB"], model["directrix"], "eB")
        # rospy.loginfo("Visualisation loaded.")

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
        s_model, direction = self.slice_model(model,start_idx, end_idx)
        # print(f"{s_model['x_corr_axes'].shape}, {s_model['y_corr_axes'].shape}, {s_model['eT'].shape}")
        # initPoints, Ratio = randomInitialPoints(s_model, numRepro, crossSectionType)
        # print(f"Initpoint {initPoints - s_model['directrix'][0,:]}")
        initPoints = np.array([[model["directrix"][start_idx,:]]])
        Ratio = np.array([0.])
        first = True

        start_task_time = rospy.Time.now()
        cumulative_correction_time = 0
        cumulative_correction_distance = 0
        lio_cumulative_correction_distance = 0

        
        while restart:

            # Generate trajectory from random a starting point
            print("----------------------------------------------------------------------------")
            rospy.loginfo("Reproduce trajectory")
            rospy.loginfo(f"  Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-s_model['directrix'][0,:])}")
            rospy.loginfo(f"  Rc[0]={s_model['Rc'][0]}. Starting ratio={Ratio}")
            
            print("#######################", s_model["xyz_corr_y"])
            repro_trajectories = reproduce(s_model, numRepro, starting, initPoints, Ratio, crossSectionType, strategy, direction)
            trajectory_xyz = repro_trajectories[0]
            # print(trajectory_xyz)
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

            self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
            if first:
                rospy.loginfo("Press START button on joystick ...")
                while not self.start:
                    rospy.sleep(0.1)
                self.events_pub.publish("Start.")
                self.events_pub.publish(f"  Go from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")
                rospy.sleep(0.1)
                first = False

            
            rospy.loginfo("Running trajectory ...")

            for i in range(nb_points):

                if self.terminate:
                    self.events_pub.publish("Terminate.")
                    break
                
                if self.change_direction_request:
                    if current_idx != t["1"] and current_idx!=t["-1"] and current_idx != t["0"]:
                        self.events_pub.publish("User change direction request.")
                        break
                    else:
                        self.change_direction_request = False
                        
                while self.gripper_state == "moving":
                    rospy.sleep(0.01)

                if self.gripper_change_direction_request:
                    if current_idx != t["1"] and current_idx!=t["-1"] and current_idx != t["0"]:
                        self.events_pub.publish("Gripper change direction request.")
                        break
                    else:
                        self.gripper_change_direction_request = False
                    


                PcurrG = trajectory_xyz[i,:]
                QcurrG = trajectory_q[i,:]
                q_weight = q_weights[i,:]

                if self.correction and i != nb_points-1: #need to change this last condition as we may want to give corrections on the last frame
                    
                    rospy.loginfo(f"  *Correction request at i={i}*")
                    self.events_pub.publish(f"Correction request at i={i}")
                    # Truncate model
                    eN = s_model["eN"][i:]
                    eB = s_model["eB"][i:]
                    eT = s_model["eT"][i:]
                    directrix = s_model["directrix"][i:]
                    Rc = s_model["Rc"][i:]
                    # Get correction axes
                    eX = s_model["x_corr_axes"][i:]
                    eY = s_model["y_corr_axes"][i:]
                    x_corr_axes = s_model["x_corr_axes"]
                    y_corr_axes = s_model["y_corr_axes"]

                    xyz_corr_y = s_model["xyz_corr_y"]
                    vertical_start = s_model["vertical_start"]
                    vertical_end = s_model["vertical_end"]

                    start_correction_time = rospy.Time.now()
                    start_position = PcurrG
                    if self.physical_robot: 
                        start_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])
                    rospy.loginfo("    correcting trajectory...")
                    rospy.loginfo(f"      Before correction: {PcurrG}")
                    while self.correction:
                        # Integrate correction
                        raw_joystickDisplacement = self.x_corr*x_corr_axes[i,:] + self.y_corr*y_corr_axes[i,:]
                        # print ("raw_joystickDisplacement: ", raw_joystickDisplacement)
                        ## Bring point to origin
                        AcurrG = PcurrG - directrix[0]
                        # print("Directrix[0]: ", directrix[0])
                        # print("AcurrG: ", AcurrG)
                        ## Apply correction
                        AcurrG_corr = AcurrG + raw_joystickDisplacement
                        # print("AcurrG_corr: ", AcurrG_corr, "of norm: ", np.linalg.norm(AcurrG_corr))
                        ## Saturate correction and compute ratio
                        if np.linalg.norm(AcurrG_corr) > Rc[0] : 
                            AcurrG_corr = Rc[0] * AcurrG_corr / np.linalg.norm(AcurrG_corr)
                            Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
                            # print(f"Rc[0]={Rc[0]} and (saturated) ratio={Ratio}")
                        else:
                            Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
                            # print(f"Rc[0]={Rc[0]} and (corr): ratio={Ratio}")
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
                        end_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])
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
                    newTrajectory = reproduce(small_model, numRepro, starting, PcurrG_corr, Ratio, crossSectionType, strategy, direction)

                    
                    trajectory_xyz[i:]= newTrajectory[0]                    
                    rospy.loginfo(f"  -> New trajectory_xyz of length {len(newTrajectory[0])}  with first point: {newTrajectory[0][0]}")
                    # Update visualisation
                    self.clear_rviz_pub.publish("reproduced")
                    self.visualise_trajectory(trajectory_xyz, trajectory_q )
                    # return
                    self.correction = False

                self.pub_cmd_pose(PcurrG, QcurrG, q_weight)
                AcurrG = PcurrG - model["directrix"][current_idx,:]
                Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
                self.pub_surface_pose(i, current_idx, PcurrG, AcurrG, QcurrG, q_weight, Ratio, "")
                # i designates the NEXT idx of the robot
                if i != 0: current_idx += 1*direction # Designates the idx AT which the robot is located
                self.rate.sleep()
            
            if self.terminate:
                if self.physical_robot : self.myp_app_pub.publish("stop")
                task_duration = (rospy.Time.now() - start_task_time).to_sec()
                rospy.loginfo(f"Total task time: {task_duration} seconds")
                rospy.loginfo(f"Total task time*: {task_duration-cumulative_correction_time} seconds")
                rospy.loginfo(f"Total correction time: {cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")
                rospy.loginfo(f"Total correction time*: {cumulative_correction_time} seconds ({(cumulative_correction_time/(task_duration-cumulative_correction_time))*100}%)")
                rospy.loginfo(f"Total correction distance: {cumulative_correction_distance} meters")
                rospy.loginfo(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters")
                self.events_pub.publish(f"Total task time: {task_duration} seconds")
                self.events_pub.publish(f"Total task time*: {task_duration-cumulative_correction_time} seconds")
                self.events_pub.publish(f"Total correction time: {cumulative_correction_time} seconds ({(cumulative_correction_time/task_duration)*100}%)")
                self.events_pub.publish(f"Total correction time*: {cumulative_correction_time} seconds ({(cumulative_correction_time/(task_duration-cumulative_correction_time))*100}%)")
                self.events_pub.publish(f"Total correction distance: {cumulative_correction_distance} meters")
                self.events_pub.publish(f"Total Lio correction distance: {lio_cumulative_correction_distance} meters")
                return
            

            rospy.loginfo("End of trajectory.")
            rospy.loginfo("Compute next section")
            self.events_pub.publish("End of section, compute next")
            rospy.loginfo(f"PREVIOUS: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")

            if self.change_direction_request:
                rospy.loginfo("   JOYSTICK change direction request")
                _ = goal
                goal = prev_goal
                prev_goal = _
                strategy = "fixed"
                self.change_direction_request = False
            elif self.gripper_change_direction_request:
                rospy.loginfo("   GRIPPER change direction request")
                if goal == "HOME":
                    pass
                else:
                    prev_goal = goal
                    goal = "HOME"
                    strategy = "convergent"
                self.gripper_change_direction_request = False
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

            start_idx = current_idx
            if goal == "PICK" : end_idx = t["-1"]
            elif goal == "PLACE" : end_idx = t["1"]
            elif goal == "HOME" : end_idx = t["0"]
            rospy.loginfo(f"  UPDATED: from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")
            self.events_pub.publish("--------------------------------------------------------------------")
            self.events_pub.publish(f"Go from {prev_goal} go {goal}, from idx {start_idx} to idx{end_idx}")


            initPoints = np.array([PcurrG])
            AcurrG = PcurrG - model["directrix"][current_idx,:]
            Ratio = np.array([np.sqrt(AcurrG[0]**2 + AcurrG[1]**2 + AcurrG[2]**2) / model["Rc"][current_idx]])
            rospy.loginfo(f"  Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-model['directrix'][current_idx,:])}")
            rospy.loginfo(f"  Rc[current_idx]={model['Rc'][current_idx]}. Starting ratio={Ratio}")
            s_model, direction = self.slice_model(model, start_idx, end_idx )




if __name__ == "__main__":
    my_node = MainNode()  
    my_node.run()