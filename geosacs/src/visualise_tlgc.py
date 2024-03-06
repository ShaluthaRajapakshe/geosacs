#!/usr/bin/env python
import rospy
import os
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Empty, String



class VisualiseTLGC(): 
    def __init__(self):
        rospy.init_node("visualise_tlgc_node") 

        # Variables
        self.rate = rospy.Rate(20)  
        self.frame = "LIO_base_link"


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



        #Init Message
        rospy.sleep(1)
        rospy.loginfo("visualise_tlgc_node has been started") 
    
        
    
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
    

    
    def run(self):
        # Specify data directory and task
        data_dir = "/home/jurassix16/TLGC_data"
        task = input("What is the name of the task? ")
        model_dir = data_dir +f"/{task}/model"
        raw_dir = data_dir + f"/{task}/record-raw"
        processed_dir = data_dir + f"/{task}/record-processed"


        # Get model & demos
        raw_demos_xyz, raw_demos_q = self.get_raw_demos(raw_dir)
        processed_demos_xyz, processed_demos_q = self.get_processed_demos(processed_dir)
        model = self.get_model(model_dir)
        q_weights = model["q_weights"]


        self.clear_rviz_pub.publish("all")
        self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
        self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
        rospy.loginfo("Loading demonstration data visualisation...")
        self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
        self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
        rospy.loginfo("Loading GC visualisation...")
        self.visualise_gc(model["GC"])
        rospy.loginfo("Loading directrix visualisation...")
        self.visualise_directrix(model["directrix"], model["q"])
        rospy.loginfo("Loading correction axes visualisation...")
        self.visualise_correction_axes(model["x_corr_axes"], model["y_corr_axes"], model["directrix"])
        rospy.loginfo("Loading eT visualisation...")
        self.visualise_TNB_axes(model["eT"], model["directrix"], "eT")
        rospy.loginfo("Loading eN visualisation...")
        self.visualise_TNB_axes(model["eN"], model["directrix"], "eN")
        rospy.loginfo("Loading eB visualisation...")
        self.visualise_TNB_axes(model["eB"], model["directrix"], "eB")
        rospy.loginfo("Visualisation loaded.")





if __name__ == "__main__":
    my_node = VisualiseTLGC()  
    my_node.run()