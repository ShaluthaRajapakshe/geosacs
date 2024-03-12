#!/usr/bin/env python
import rospy
import os
import numpy as np
from scipy.interpolate import CubicSpline
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
from splines.quaternion import CatmullRom, UnitQuaternion
from sksurgerycore.algorithms.averagequaternions import average_quaternions
from dtw import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import KMeans

# Resample & Format data to send to GC creating node

class DataProcessingNode(): 
    def __init__(self):
        rospy.init_node("tglc_data_processing_node") 

        # Variables
        self.rate = rospy.Rate(10)  
        self.demos = None
        self.resampled_demos = None
        self.q_delta_pause_tolerance = 0.00001

        # self.process_data = True
        self.process_data = False

        self.task = None
        self.data_dir = None
        self.raw_dir = None

        # ROS Variables
        self.raw_traj_pub = rospy.Publisher("/tlgc/raw_trajectory", PoseArray, queue_size=10)
        self.processed_traj_pub = rospy.Publisher("/tlgc/processed_trajectory", PoseArray, queue_size=10)
        self.clear_viz_pub = rospy.Publisher("/tlgc/viz_clear", String, queue_size=2)
        self.raw_pose_traj_pub = rospy.Publisher("/tlgc/raw_pose_trajectory", PoseArray, queue_size=10)
        self.processed_pose_traj_pub = rospy.Publisher("/tlgc/processed_pose_trajectory", PoseArray, queue_size=10)

        
        #Init Message
        rospy.sleep(1)
        rospy.loginfo("data_processing_node has been started")



    
    
    def get_data(self, raw_dir):
        rospy.loginfo(f"Raw directory {raw_dir}")
        items = os.listdir(raw_dir)
        rospy.loginfo(f"Files found {items}")

        demos_xyz = []
        demos_q = []
        for file in items:
            demo_file = raw_dir + f"/{file}"
            raw_dict = np.load(demo_file)
            xyz = raw_dict["position"]
            q = raw_dict["orientation"]
            demos_xyz.append(xyz)
            demos_q.append(q)

        return demos_xyz, demos_q

    def static_filter(self, demos_xyz, demos_q):

        rospy.loginfo("Static filtering of demos:")
  
        if len(demos_xyz) != len(demos_q):
            rospy.logerr("static_filter : len(demos_xyz) != len(demos_q)")
            return
        
        n = len(demos_xyz)
        filtered_demos_xyz = []
        filtered_demos_q = []

        for i in range(n):
            # Get trajectory points
            positions = demos_xyz[i]
            orientations = demos_q[i]
            if positions.shape[0] != orientations.shape[0]:
                rospy.logerr("static_filter: positions.shape[0] != orientations.shape[0]")
                return
            rospy.loginfo(f"  Trajectory {i}")
            rospy.loginfo(f"    Trajectory size before static filtering: {positions.shape[0]}")
            # Determine static filter
            q_future, q_past = positions[1:,:], positions[:-1,:]
            q_diff_norms = np.linalg.norm(q_future - q_past, axis=1)
            keep_idxs = list((q_diff_norms > self.q_delta_pause_tolerance).nonzero()[0])
            keep_idxs.insert(0,0) # Include first point
            keep_idxs.append(positions.shape[0]-1) # Include last point
            # Filter trajectory
            filtered_positions = positions[keep_idxs,:]
            filtered_orientations = orientations[keep_idxs,:]
            rospy.loginfo(f"    Trajectory size after static filtering: {filtered_positions.shape[0]}")
            # Append
            filtered_demos_xyz.append(filtered_positions)
            filtered_demos_q.append(filtered_orientations)
        
        return filtered_demos_xyz, filtered_demos_q

    def step_filter(self, demos_xyz, demos_q):
        
        rospy.loginfo("Step filtering of demos:")
  
        if len(demos_xyz) != len(demos_q):
            rospy.logerr("step_filter : len(demos_xyz) != len(demos_q)")
            return
        
        n = len(demos_xyz)
        filtered_demos_xyz = []
        filtered_demos_q = []
        stepc = None

        
        for i in range(n):
            positions = demos_xyz[i]
            orientations = demos_q[i]

            if positions.shape[0] != orientations.shape[0]:
                rospy.logerr("step_filter: positions.shape[0] != orientations.shape[0]")
                return
            
            rospy.loginfo(f"Trajectory {i}")
            rospy.loginfo(f"  Trajectory size before step filtering: {positions.shape[0]}")
            # Determine step filter
            num_points = len(positions)
            stepc = int(0.1 * num_points) # int() returns integer part
            if stepc == 0 : stepc = 1
            keep_idxs = list(range(0, num_points, stepc))
            if not (num_points-1)%stepc == 0: 
                keep_idxs.append(num_points-1) # include last point
                rospy.loginfo(f"    step size of {0.1*num_points} -> {stepc}")
                rospy.loginfo(f"    Keep idxs: {keep_idxs} with last point")
            else:
                rospy.loginfo(f"    step size of {0.1*num_points} -> {stepc}")
                rospy.loginfo(f"    Keep idxs: {keep_idxs}")
            # Filter trajectory
            filtered_positions = positions[keep_idxs, :]
            filtered_orientations = orientations[keep_idxs, :]
            rospy.loginfo(f"    Trajectory size after step filtering: {filtered_positions.shape[0]}")

            # Append
            filtered_demos_xyz.append(filtered_positions)
            filtered_demos_q.append(filtered_orientations)
        
        return filtered_demos_xyz, filtered_demos_q, stepc




    def request_save(self, demos_xyz, demos_q):
        processed_dir = self.data_dir + f"/{self.task}/record-processed"
        demo_file = processed_dir + f"/{self.task}-processed.npz"

        if input("Save these demos? (y/[n]) ")=="y": save = True
        else: save= False

        if save:
            processed_dict = {"position":demos_xyz, "orientation":demos_q}
            os.makedirs(processed_dir, exist_ok=True) 
            np.savez(str(demo_file), **processed_dict)
            # print("Demos shape:", new_demos.shape)
            print(f"Processed demos of '{self.task}' task saved in {demo_file}")
            
        else:
            print(f"Demos of '{self.task}' task not saved")


    def resample_interpolate(self, demos_xyz, demos_q, N):

        rospy.loginfo(f"Resample and interpolate demos to {N} points:")
          
        if len(demos_xyz) != len(demos_q):
            rospy.logerr("resample_interpolate: len(demos_xyz) != len(demos_q)")
            return

        resampled_demos_xyz = []
        resampled_demos_q = []

        for positions, orientations in zip(demos_xyz, demos_q):
            
            if positions.shape[0] != orientations.shape[0]:
                rospy.logerr("resample_interpolate: Nb positions != nb orientations")
                return

            # Position splines
            n = positions.shape[0]
            t_original = np.linspace(0,1,n)
            spline_x = CubicSpline(t_original, positions[:, 0])
            spline_y = CubicSpline(t_original, positions[:, 1])
            spline_z = CubicSpline(t_original, positions[:, 2])
            rospy.loginfo(f"  Trajectory size before interpolation: {n}")

            # Position interpolation
            t_resampled = np.linspace(0,1, N)
            stacking = np.vstack((spline_x(t_resampled), spline_y(t_resampled), spline_z(t_resampled)))
            resampled_positions = stacking.T
            rospy.loginfo(f"  Trajectory size after interpolation: {N}")

            # Create UnitQuaternions list 
            unitQuats = []
            for i in range(n):
                wxyz = orientations[i,:]
                xyzw = [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]
                q = UnitQuaternion.from_unit_xyzw(xyzw)
                norm = np.linalg.norm(wxyz)
                unitQuats.append(q)
            
            # Orientation "spline"
            catmull = CatmullRom(unitQuats, t_original)

            # Orientation interpolation
            result = catmull.evaluate(t_resampled)
            resampled_orientations = np.zeros((N,4))
            for i in range(N):
                scalar = result[i].scalar
                vector = result[i].vector
                resampled_orientations[i,:] = np.array([[scalar, vector[0], vector[1], vector[2]]])
            
            resampled_demos_xyz.append(resampled_positions)
            resampled_demos_q.append(resampled_orientations)
        
        return resampled_demos_xyz, resampled_demos_q

        
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
            
            if type == "resampled":
                self.processed_traj_pub.publish(msg)
                self.processed_pose_traj_pub.publish(msg)
            elif type == "raw": 
                self.raw_traj_pub.publish(msg)
                self.raw_pose_traj_pub.publish(msg)
            else :
                rospy.logerr("Type of trajectory not detected.")
    
    
    
    def spatial_filter(self, demos_xyz, demos_q):
        rospy.loginfo("Spatial filtering of demos:")
  
        if len(demos_xyz) != len(demos_q):
            rospy.logerr("spatial_filter : len(demos_xyz) != len(demos_q)")
            return
        
        n = len(demos_xyz)
        filtered_demos_xyz = []
        filtered_demos_q = []
        tolerance = 0.05 # radius of sphere

        for i in range(n):
            positions = demos_xyz[i]
            orientations = demos_q[i]

            if positions.shape[0] != orientations.shape[0]:
                rospy.logerr("spatial_filter : positions.shape[0] != orientations.shape[0])")
                return
            nb_points = positions.shape[0]
            rospy.loginfo(f"  Trajectory {i} of size {nb_points}")

            filtered_positions = []
            filtered_orientations = []
            idx = 0
            increment = True
            finish = False

            while not finish:
                average_idxs = []
                center = positions[idx]
                sum = 0
                increment = True
                rospy.loginfo(f"    Finding points in sphere around idx={idx}")
                while increment:
                    if np.linalg.norm(positions[idx]-center) < tolerance:
                        # Update sum
                        average_idxs.append(idx)
                        sum += positions[idx]
                        # Update idx
                        if idx+1 < nb_points: 
                            idx+=1
                        else: 
                            increment = False
                            finish = True
                            rospy.loginfo(f"      End of trajectory with {len(average_idxs)} points")
                    else: 
                        increment = False
                        rospy.loginfo(f"      End of sphere with {len(average_idxs)} points")
                
                # Position average and append
                average_position = sum/len(average_idxs)
                filtered_positions.append(average_position)
                rospy.loginfo(f"    Average {average_position} computed.")

                # Orientation average and append
                quaternions = orientations[average_idxs,:]
                q = average_quaternions(quaternions)
                filtered_orientations.append(q)

            # Format and append
            filtered_positions_arr = np.zeros((len(filtered_positions),3))
            for i in range(len(filtered_positions)):
                filtered_positions_arr[i,:] = filtered_positions[i]
            filtered_demos_xyz.append(filtered_positions_arr)

            filtered_orientations_arr = np.zeros((len(filtered_orientations),4))
            for i in range(len(filtered_orientations)):
                filtered_orientations_arr[i,:] = filtered_orientations[i]
            filtered_demos_q.append(filtered_orientations_arr)

            rospy.loginfo(f"    Filtered trajectory of lenght {filtered_orientations_arr.shape[0]}")


        return filtered_demos_xyz, filtered_demos_q

    
    
    def find_unique_closest_points(self, trajectories, centroids):
        print("UNIQUE CLOSEST POINT:", trajectories[1].shape)
        closest_points_indices = [[] for _ in trajectories]
        for centroid in centroids:
            for i, traj in enumerate(trajectories):
                distances = np.linalg.norm(traj - centroid, axis=1)
                sorted_indices = np.argsort(distances)
                for idx in sorted_indices:
                    if idx not in closest_points_indices[i]:
                        closest_points_indices[i].append(idx)
                        break
        
        return closest_points_indices

    # def cluster_filter(self, demos_xyz, demos_q):
        
    #     filtered_demos_xyz = []
    #     filtered_demos_q = []

    #     n_clusters = 50
    #     trajectory1_xyz = demos_xyz[0]
    #     trajectory2_xyz = demos_xyz[1]
    #     trajectory1_q = demos_q[0]
    #     trajectory2_q = demos_q[1]

    #     print("IN CLUSTER")
    #     print(trajectory1_xyz.shape)
    #     print(trajectory2_xyz.shape)
    #     print(np.concatenate((trajectory1_xyz, trajectory2_xyz), axis=0).shape)
        
    #     kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(np.concatenate((trajectory1_xyz, trajectory2_xyz), axis=0))
    #     centroids = kmeans.cluster_centers_
    #     closest_points_indices = self.find_unique_closest_points([trajectory1_xyz, trajectory2_xyz], centroids)

    #     closest_points_trajectory1_xyz = trajectory1_xyz[np.sort(closest_points_indices[0]), :]
    #     closest_points_trajectory2_xyz = trajectory2_xyz[np.sort(closest_points_indices[1]), :]
    #     closest_points_trajectory1_q = trajectory1_q[np.sort(closest_points_indices[0]), :]
    #     closest_points_trajectory2_q = trajectory2_q[np.sort(closest_points_indices[1]), :]

    #     filtered_demos_xyz.append(closest_points_trajectory1_xyz)
    #     filtered_demos_xyz.append(closest_points_trajectory2_xyz)
    #     filtered_demos_q.append(closest_points_trajectory1_q)
    #     filtered_demos_q.append(closest_points_trajectory2_q)

    #     return filtered_demos_xyz, filtered_demos_q

    def cluster_filter(self, demos_xyz, demos_q):
        
        filtered_demos_xyz = []
        filtered_demos_q = []

        if len(demos_xyz) != len(demos_q):
            rospy.logerr("cluster_filter: len(demos_xyz) != len(demos_q)")
        elif len(demos_xyz) == 0 :
            rospy.logerr("cluster_filter: len(demos_xyz) == 0")

        nb_demos = len(demos_xyz)
        all_points = demos_xyz[0]

        for i in range(nb_demos-1):
            all_points = np.concatenate((all_points, demos_xyz[i+1]), axis=0)

        n_clusters = 50        
        kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(all_points)
        centroids = kmeans.cluster_centers_
        closest_points_indices = self.find_unique_closest_points(demos_xyz, centroids)

        for i in range(nb_demos):
            traj_xyz = demos_xyz[i]
            traj_q = demos_q[i]
            closest_points_traj_xyz = traj_xyz[np.sort(closest_points_indices[i]), :]
            closest_points_traj_q = traj_q[np.sort(closest_points_indices[i]), :]
            filtered_demos_xyz.append(closest_points_traj_xyz)
            filtered_demos_q.append(closest_points_traj_q)

        return filtered_demos_xyz, filtered_demos_q
    
    
    def dtw_filter(self, demos_xyz, demos_q):
        
        filtered_demos_xyz = []
        filtered_demos_q = []

        trajectory1_xyz = demos_xyz[0]
        trajectory2_xyz = demos_xyz[1]
        trajectory1_q = demos_q[0]
        trajectory2_q = demos_q[1]

        alignment = dtw(trajectory1_xyz, trajectory2_xyz, keep_internals=True, 
                step_pattern=rabinerJuangStepPattern(6, "c"))  

        dtw_distance = alignment.normalizedDistance
        print("Normalized DTW Distance between the trajectories:", dtw_distance)

        
        filtered_demos_xyz.append(trajectory1_xyz[alignment.index1s,:])
        filtered_demos_xyz.append(trajectory2_xyz[alignment.index2s,:])
        filtered_demos_q.append(trajectory1_q[alignment.index1s,:])
        filtered_demos_q.append(trajectory2_q[alignment.index2s,:])

        return filtered_demos_xyz, filtered_demos_q
                    
    def run(self):

        self.data_dir = "/home/jurassix16/TLGC_data"
        self.task = input("What is the name of the task? ")
        # self.task = "testlab"
        self.raw_dir = self.data_dir + f"/{self.task}/record-raw"

        raw_demos_xyz, raw_demos_q = self.get_data(self.raw_dir)
        # demos_xyz, demos_q = self.static_filter(raw_demos_xyz, raw_demos_q)
        # demos_xyz, demos_q = self.step_filter(demos_xyz, demos_q)
        # demos_xyz, demos_q = self.spatial_filter(raw_demos_xyz, raw_demos_q)
        # demos_xyz, demos_q = self.step_filter(demos_xyz, demos_q)
        # demos_xyz, demos_q = self.cluster_filter(raw_demos_xyz, raw_demos_q)
        demos_xyz, demos_q = self.dtw_filter(raw_demos_xyz, raw_demos_q)
        demos_xyz, demos_q, step = self.step_filter(demos_xyz, demos_q)

    
        max_length = 0
        for positions in demos_xyz:
            if positions.shape[0] > max_length: max_length=positions.shape[0]
        N =step*max_length

        demos_xyz, demos_q = self.resample_interpolate(demos_xyz, demos_q, N)

        # Visualise
        self.clear_viz_pub.publish("all")
        self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw")
        self.visualise_demos(demos_xyz, demos_q, "resampled")

        # Save
        self.request_save(demos_xyz, demos_q)
   
if __name__ == "__main__":
    my_node = DataProcessingNode()  
    my_node.run()