#!/usr/bin/env python
import rospy
import os
import numpy as np
from sksurgerycore.algorithms.averagequaternions import average_quaternions
from geometry_msgs.msg import Pose, PoseArray
from constructGC import construct_GC
from circularGC import circular_GC
from getTNB import get_TNB
from std_msgs.msg import String
from scipy.spatial.transform import Rotation, Slerp
from pyquaternion import Quaternion


class TLGCNode(): 
    def __init__(self):
        rospy.init_node("tlgc_node") 

        # Variables
        self.rate = rospy.Rate(10) 
        self.data_dir = None
        self.processed_dir = None
        self.task = None
        self.process_data = False

        # ROS Variables

        self.directrix_pub = rospy.Publisher("/tlgc/directrix", PoseArray, queue_size=10)
        self.raw_traj_pub = rospy.Publisher("/tlgc/raw_trajectory", PoseArray, queue_size=10)
        self.processed_traj_pub = rospy.Publisher("/tlgc/processed_trajectory", PoseArray, queue_size=10)
        self.gc_circle_pub = rospy.Publisher("/tlgc/gc_circle", PoseArray, queue_size=10)
        self.clear_viz_pub = rospy.Publisher("/tlgc/viz_clear", String, queue_size=2)
        self.raw_pose_traj_pub = rospy.Publisher("/tlgc/raw_pose_trajectory", PoseArray, queue_size=10)
        self.processed_pose_traj_pub = rospy.Publisher("/tlgc/processed_pose_trajectory", PoseArray, queue_size=10)
        self.directrix_pose_pub = rospy.Publisher("/tlgc/directrix_pose", PoseArray, queue_size=10)
        self.eT_axis_pub = rospy.Publisher("/tlgc/eT_axis", PoseArray, queue_size=10)
        self.eN_axis_pub = rospy.Publisher("/tlgc/eN_axis", PoseArray, queue_size=10)
        self.eB_axis_pub = rospy.Publisher("/tlgc/eB_axis", PoseArray, queue_size=10)
        self.x_corr_axis_pub = rospy.Publisher("/tlgc/x_correction_axis", PoseArray, queue_size=10)
        self.y_corr_axis_pub = rospy.Publisher("/tlgc/y_correction_axis", PoseArray, queue_size=10)

        #Init Message
        rospy.sleep(1)
        rospy.loginfo("tlgc_node has been started") 


    
    
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

    def reshape_demos(self, demos_xyz, demos_q):
        rospy.loginfo("Reshaping the demos...")

        if len(demos_xyz) != len(demos_q):
            rospy.logerr("reshape_demos: len(demos_xyz) != len(demos_q)")
            return

        nb_demos = len(demos_xyz) 
        nb_points = len(demos_xyz[0])
        dim_xyz = len(demos_xyz[0][0])
        dim_q = len(demos_q[0][0])

        rospy.loginfo(f"  Before reshape demos_xyz: ({nb_demos},{nb_points},{dim_xyz}")
        rospy.loginfo(f"  Before reshape demos_q: ({nb_demos},{nb_points},{dim_q} ")
        reshaped_demos_xyz = np.zeros((nb_points, nb_demos, dim_xyz))
        reshaped_demos_q = np.zeros((nb_points, nb_demos, dim_q))

        for i in range(nb_demos):
            reshaped_demos_xyz[:,i] = demos_xyz[i]
            reshaped_demos_q[:,i] = demos_q[i]
        
        rospy.loginfo(f"  After reshape demos_xyz: {reshaped_demos_xyz.shape}")
        rospy.loginfo(f"  After reshape demos_q: {reshaped_demos_q.shape} ")  
        
        return reshaped_demos_xyz, reshaped_demos_q
    
    
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
    
    def get_model(self, demos_xyz, idx):
        rospy.loginfo(f"Creating GC model over {demos_xyz.shape} xyz points...")
        numDemos = demos_xyz.shape[1]
        numPoints = demos_xyz.shape[0]
        crossSectionType = 'circle'
        model_xyz = construct_GC(demos_xyz, numDemos, numPoints, idx, crossSectionType)  
        rospy.loginfo(f"  GC model created.")  
        return model_xyz
    
    
    def get_mean(self, demos_q, idx):
        rospy.loginfo(f"Averaging quaternions...")
        nb_points = demos_q.shape[0]
        nb_demos = demos_q.shape[1]
        dim_q = demos_q.shape[2]
        
        mean_quaternions = []

        # Scikit lib
        for i in range(idx[0], nb_points-idx[1]):
            quaternions = demos_q[i,:]
            q = average_quaternions(quaternions)
            mean_quaternions.append(q)

                
        mean_quaternions_arr = np.zeros((len(mean_quaternions), dim_q))
        for i in range(len(mean_quaternions)):
            mean_quaternions_arr[i,:] = mean_quaternions[i]

        rospy.loginfo(f"  {mean_quaternions_arr.shape} mean quaternions computed")

        return mean_quaternions_arr

    def visualise_gc(self, gc_circles):  
        print(gc_circles.shape)
        # for circle in gc_circles:
        #     msg = PoseArray()
        #     for point in circle:
        #         p = Pose()
        #         p.position.x = point[0]
        #         p.position.y = point[1]
        #         p.position.z = point[2]
        #         msg.poses.append(p)

        #     self.gc_circle_pub.publish(msg)
        #     rospy.sleep(0.1)
    
        for i in range(gc_circles.shape[0]):
            msg = PoseArray()
            for point in gc_circles[i,:,:]:
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
    

    def request_save(self, model):
        model_dir = self.data_dir +f"/{self.task}/model"
        model_file = model_dir +f"/{self.task}-model.npz"
        save = False

        if input("Save this model? (y/n) ") == "y": save = True
        else : save = False

        if save:
            os.makedirs(model_dir, exist_ok=True) 
            np.savez(str(model_file), **model)
            print(f"TLGC model of '{self.task}' task saved in {model_file}.")
        else:
            print(f"TLGC model of '{self.task}' task not saved.")

    
    
    def project_onto_plane(self, v, normal):
        # Normalize the normal vector to ensure it's a unit vector
        normal_normalized = normal / np.linalg.norm(normal)
        # Calculate the projection of v onto the plane defined by the normal
        projection = v - np.dot(v, normal_normalized) * normal_normalized
        return projection
    
    # def saturate_model(self, model):
    #     gc_circles = model["GC"]
    #     nb_points = gc_circles.shape[0]
    #     saturate_idxs = []

    #     for i in range(nb_points):
    #         circle = gc_circles[i,:,:]
    #         for point in circle:
    #             if point[2] <= 0: 
    #                 saturate_idxs.append(i)
    #                 break
        
    #     if len(saturate_idxs) != 0: 
    #         rospy.loginfo(f"Model saturation necessary at idxs: {saturate_idxs}")
    #     else: 
    #         return model
        
    #     global_x = np.array([1, 0, 0])
    #     global_y = np.array([0, 1, 0])
    #     global_z = np.array([0, 0, 1])
    #     eT = model["eT"]
    #     reverse_eT = model["reverse_eT"]
    #     eN = model["eN"]
    #     reverse_eN = model["reverse_eN"]
    #     eB = model["eB"]
    #     reverse_eB = model["reverse_eB"]
    #     directrix = model["directrix"]
    #     Rc = model["Rc"]
        
    #     for i in saturate_idxs:
    #         # Compute eT vector angle
    #         T = eT[i,:]/np.linalg.norm(eT[i,:])
    #         cos_angle_z = np.clip(np.dot(T, global_z), -1.0, 1.0)
    #         angle_z = np.arccos(cos_angle_z)
    #         angle_z = np.degrees(angle_z)
    #         # Change T
    #         if angle_z > 90: 
    #             T = -global_z
    #         else: 
    #             T = global_z

    #         # Compute reverse_eT vector angle
    #         reverse_T = reverse_eT[i,:]/np.linalg.norm(reverse_eT[i,:])
    #         reverse_cos_angle_z = np.clip(np.dot(reverse_T, global_z), -1.0, 1.0)
    #         reverse_angle_z = np.arccos(reverse_cos_angle_z)
    #         reverse_angle_z = np.degrees(reverse_angle_z)
    #         # Change reverse_eT 
    #         if reverse_angle_z > 90: 
    #             reverse_T = -global_z
    #         else: 
    #             reverse_T = global_z
            
    #         # Redefine GC
    #         gc_circles[i,:,:] = circular_GC(np.array([directrix[i,:]]), np.array([global_x]), 
    #                                         np.array([global_y]), np.array([Rc[i]]))

    #         # Change N and B
    #         N = self.project_onto_plane(eN[i,:], global_z)
    #         reverse_N = self.project_onto_plane(reverse_eN[i,:], global_z)


    #         # Normalise
    #         T = T/np.linalg.norm(T)
    #         N = N/np.linalg.norm(N)
    #         B = np.cross(N, T)
    #         B = B/np.linalg.norm(B)

    #         reverse_T = reverse_T/np.linalg.norm(reverse_T)
    #         reverse_N = reverse_N/np.linalg.norm(reverse_N)
    #         reverse_B = np.cross(reverse_N, reverse_T)
    #         reverse_B = reverse_B/np.linalg.norm(reverse_B)


    #         # Assign
    #         eT[i,:] = T
    #         eN[i,:] = N
    #         eB[i,:] = B
    #         reverse_eT[i,:] = reverse_T
    #         reverse_eN[i,:] = reverse_N
    #         reverse_eB[i,:] = reverse_B


    #     # Update model
    #     model["GC"] = gc_circles
    #     model["eT"] = eT
    #     model["eN"] = eN
    #     model["eB"] = eB
    #     model["reverse_eT"] = reverse_eT
    #     model["reverse_eN"] = reverse_eN
    #     model["reverse_eB"] = reverse_eB

         
    #     return model
    
    def saturate_model(self, model, min_start, min_end):
        gc_circles = model["GC"]
        nb_points = gc_circles.shape[0]
        saturate_idxs = []
        minimum_z = min(min_start, min_end)

        for i in range(nb_points):
            circle = gc_circles[i,:,:]
            for point in circle:
                if point[2] <= minimum_z: 
                    saturate_idxs.append(i)
                    break
        
        if len(saturate_idxs) != 0: 
            rospy.loginfo(f"Model saturation for {len(saturate_idxs)} points ...")
        else: 
            rospy.loginfo(f"No model saturation.")
            return model
        
        global_x = np.array([1, 0, 0])
        global_y = np.array([0, 1, 0])
        global_z = np.array([0, 0, 1])
        eT = model["eT"]
        eN = model["eN"]
        eB = model["eB"]
        directrix = model["directrix"]
        Rc = model["Rc"]
        
        for i in saturate_idxs:
            # Compute eT vector angle
            T = eT[i,:]/np.linalg.norm(eT[i,:])
            cos_angle_z = np.clip(np.dot(T, global_z), -1.0, 1.0)
            angle_z = np.arccos(cos_angle_z)
            angle_z = np.degrees(angle_z)
            # Change T
            if angle_z > 90: 
                T = -global_z
            else: 
                T = global_z
            
            # Redefine GC
            gc_circles[i,:,:] = circular_GC(np.array([directrix[i,:]]), np.array([global_x]), 
                                            np.array([global_y]), np.array([Rc[i]]))

            # Change N and B
            N = self.project_onto_plane(eN[i,:], global_z)


            # Normalise
            T = T/np.linalg.norm(T)
            N = N/np.linalg.norm(N)
            B = np.cross(N, T)
            B = B/np.linalg.norm(B)

            # Assign
            eT[i,:] = T
            eN[i,:] = N
            eB[i,:] = B


        # Update model
        model["GC"] = gc_circles
        model["eT"] = eT
        model["eN"] = eN
        model["eB"] = eB

        rospy.loginfo(f"  Model saturation completed.")
         
        return model



    
    def visualise_TNB_axes(self, axes, directrix, type):

        
        nb_points = directrix.shape[0]

        for i in range(nb_points):
            # if i%3 != 0 or i==0 : continue for tnb vs txy
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
    
    
    def add_reverse_model(self, model):

        directrix = model["directrix"]
        reverse_directrix = np.flip(directrix, axis=0)

        # Recompute T frames
        reverse_eT, reverse_eN, reverse_eB = get_TNB(reverse_directrix)
        # Re-index to match the original model
        reverse_eT = np.flip(reverse_eT, axis=0)
        reverse_eN = np.flip(reverse_eN, axis=0)
        reverse_eB = np.flip(reverse_eB, axis=0)

        print(reverse_eT.shape)
        
        model["reverse_eT"] = reverse_eT
        model["reverse_eN"] = reverse_eN
        model["reverse_eB"] = reverse_eB

        return model

    def rodrigues_rotation(self, v, k, theta):
        v_rot = v * np.cos(theta) + np.cross(k, v) * np.sin(theta) + k * np.dot(k, v) * (1 - np.cos(theta))
        return v_rot

    def project_onto_plane(self, v, normal):
        # Normalize the normal vector to ensure it's a unit vector
        normal_normalized = normal / np.linalg.norm(normal)
        # Calculate the projection of v onto the plane defined by the normal
        projection = v - np.dot(v, normal_normalized) * normal_normalized
        return projection

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
    
    def mean_direction(self, unit_vectors):
        sum_vector = np.sum(unit_vectors, axis=0)
        if np.linalg.norm(sum_vector) == 0:
            return unit_vectors[-1]
            
        mean_vector = sum_vector / np.linalg.norm(sum_vector)

        return mean_vector
    
    def slerp(self, v0, v1, t, omega):
        v0 = np.asarray(v0)
        v1 = np.asarray(v1)

        v0 = v0 / np.linalg.norm(v0)
        v1 = v1 / np.linalg.norm(v1)

        omega = np.radians(omega)


        sin_omega = np.sin(omega)

        # If sin(omega) is close to zero or 180
        if sin_omega < 1e-10:
            print ("The vectors are alomst the same, so we will keep the previous x axis as otherwise its abrupt")
            resulting_vector = v1
        else:
            resulting_vector = (1/sin_omega) * ( np.sin((1.0 - t) * omega) * v0 + np.sin(t * omega) * v1)

        return resulting_vector
    
    def get_correction_axes(self, model):
            
        eT = model["eT"]
        directrix = model["directrix"]
        global_x = np.array([1, 0, 0])
        global_y = np.array([0, 1, 0])
        global_z = np.array([0, 0, 1])
        nb_points = directrix.shape[0]

        x_corr_axes = np.zeros((nb_points, 3))
        y_corr_axes = np.zeros((nb_points, 3))
        
        current_quadrant = None
        previous_quadrant = None
        previous_correction_x_axis = None

        ##newly added
        previous_correction_y_axes = []  #we will use this list to find the average vector within the defined window
        window_size = 10 ## We can play with this to see the effect

        for i in range(nb_points):
            eT_curr =eT[i,:]
            # # No axis change code
            # x_axis = np.cross(global_z, eT_curr)
            # y_axis = np.cross(eT_curr, x_axis)
            # x_axis /= np.linalg.norm(x_axis)
            # y_axis /= np.linalg.norm(y_axis)
            # x_corr_axes[i,:] = x_axis
            # y_corr_axes[i,:] = y_axis

            # Alternative method method
            T_normalized = eT_curr / np.linalg.norm(eT_curr)

            projected_global_x = self.project_onto_plane(global_x, T_normalized)


            correction_x_axis = projected_global_x
            correction_y_axis = np.cross(T_normalized, correction_x_axis)

            if i == 0 : 
                previous_correction_x_axis = correction_x_axis
                previous_correction_y_axis = correction_y_axis
                previous_correction_y_axes.append(correction_y_axis)
                x_corr_axes[i,:] = correction_x_axis
                y_corr_axes[i,:] = correction_y_axis
                continue
            
            ### Weighting the importance w.r.t global x and projected previous correction x axis
            #project previous correction x axis onto the plane
            previous_correction_x_axis = previous_correction_x_axis / np.linalg.norm(previous_correction_x_axis)
            
            projected_previous_correction = self.project_onto_plane(previous_correction_x_axis, T_normalized)
            
            #### Newly added ####
            if (projected_global_x == np.array([0, 0, 0])).all():
                projected_global_x = projected_previous_correction

            theta_X = self.calculate_angle_between_vectors(global_x, projected_global_x) #in degrees
            theta_C = self.calculate_angle_between_vectors(projected_previous_correction, projected_global_x) #in degrees

            # theta_C = np.radians(theta_C)
            # theta_X = np.radians(theta_X)
            print(f"theta_X = {theta_X}, theta_C = {theta_C}")
            t = np.radians(theta_X)/(np.pi/2)
            
            correction_x_axis = self.slerp(projected_global_x, projected_previous_correction, t, theta_C)

            # # Classic code for y axis
            # correction_y_axis = np.cross(T_normalized, correction_x_axis)

            # # Newly added code for correction y axis
            ### Handling for correction  Y axis

            if i >= window_size:
                previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes[-window_size:])
                projected_previous_mean_y_correction = self.project_onto_plane(previous_mean_correction_y_axis, T_normalized)
                correction_y_axis = np.cross(T_normalized, correction_x_axis)
                y_angle_diff = self.calculate_angle_between_vectors(correction_y_axis, projected_previous_mean_y_correction)
                if y_angle_diff > 90:
                    correction_y_axis = -correction_y_axis
            else:
                if previous_correction_y_axes == []: 
                    correction_y_axis = np.cross(T_normalized, correction_x_axis)
                else:
                    previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes)
                    projected_previous_mean_y_correction = self.project_onto_plane(previous_mean_correction_y_axis, T_normalized)
                    correction_y_axis = np.cross(T_normalized, correction_x_axis)
                    y_angle_diff = self.calculate_angle_between_vectors(correction_y_axis, projected_previous_mean_y_correction)
                    if y_angle_diff > 90:
                        correction_y_axis = -correction_y_axis
                   
            
            previous_correction_x_axis = correction_x_axis
            previous_correction_y_axes.append(correction_y_axis)

            # Normalise and append
            correction_x_axis /= np.linalg.norm(correction_x_axis)
            correction_y_axis /= np.linalg.norm(correction_y_axis)
            x_corr_axes[i,:] = correction_x_axis
            y_corr_axes[i,:] = correction_y_axis

        return x_corr_axes, y_corr_axes
    
    def get_orientation_weights(self, processed_q, mean_q, idx):
        
        nb_demos = processed_q.shape[0]
        # print(f"processed_q: {processed_q.shape}")

        # print(f"idx: {idx}")
        nb_points_model = mean_q.shape[0]
        # print(f"mean_q: {mean_q.shape}")

        sums = np.zeros((nb_points_model,1))
        standard_deviations = np.zeros((nb_points_model,1))
        # print(standard_deviations.shape)


        for i in range(nb_points_model):
            mean_quat = mean_q[i,:]
            mean_quat_object = Quaternion(mean_quat[0], mean_quat[1], mean_quat[2], mean_quat[3])
            squared_distance_sum = 0
            # print(f"i={i}")
            # print(f"Mean quat:{mean_quat}")
            # print(f"Mean quat object:{mean_quat_object}")

            for j in range(nb_demos):
                k = i+idx[0]
                quat = processed_q[j,k,:]
                quat_object = Quaternion(quat[0], quat[1], quat[2], quat[3])
                distance = Quaternion.absolute_distance(quat_object, mean_quat_object)
                squared_distance_sum += distance**2
                # print(f"quat:{quat}")
                # print(f"quat object:{quat_object}")
                # print(f"distance{distance}")            
            
            sums[i,:] = squared_distance_sum
            standard_deviations[i,:] = np.sqrt(squared_distance_sum/nb_demos)
        
        # print(st0andard_deviations)

        weights = np.exp(-9*(standard_deviations-0.3)) #normally -8.5
        # print(weights)
        
        
        return weights
    
    def visualise_correction_axes(self, x_axes, y_axes, directrix):
        
        nb_points = directrix.shape[0]
        for i in range(nb_points):
            # if i%3 != 0 or i==0 : continue # for tnb vs txy
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

    
    def get_minimums(self, processed_demos_xyz):
        # print(processed_demos_xyz.shape)
        nb_demos = processed_demos_xyz.shape[0]
        nb_points = processed_demos_xyz.shape[1]
        
        starting_points = []
        ending_points = []

        # for i in range(nb_demos):
        #     demo = processed_demos_xyz[i,:,:]
        #     starting_points.append(demo[0,:])
        #     ending_points.append(demo[-1,:])
        
        starting_z = processed_demos_xyz[:,0,2]
        ending_z = processed_demos_xyz[:,-1,2]

        min_start_z = np.min(starting_z)
        min_end_z = np.min(ending_z)

        # print(starting_z, ending_z)
        # print(min_start_z, min_end_z)

        return min_start_z, min_end_z

    def run(self):
        self.data_dir = "/home/jurassix16/TLGC_data"
        self.task = input("What is the name of the task? ")
        # self.task = "laundry_loop3"
        raw_dir = self.data_dir + f"/{self.task}/record-raw"
        processed_dir = self.data_dir + f"/{self.task}/record-processed"
        
        raw_demos_xyz, raw_demos_q = self.get_raw_demos(raw_dir)
        processed_demos_xyz, processed_demos_q = self.get_processed_demos(processed_dir)
        demos_xyz, demos_q = self.reshape_demos(processed_demos_xyz, processed_demos_q)
        
        idx = np.array([0, 0]) #These values should be adjusted depending on demo #3,20 for tnb vs tny
        xyz_model = self.get_model(demos_xyz, idx)
        
        min_start, min_end = self.get_minimums(processed_demos_xyz)
        # xyz_model = self.saturate_model(xyz_model, min_start, min_end)

        x_corr_axes, y_corr_axes = self.get_correction_axes(xyz_model)

        q_mean = self.get_mean(demos_q, idx)
        q_weights = self.get_orientation_weights(processed_demos_q, q_mean, idx)
        model = xyz_model
        model["q"] = q_mean
        model["idx"] = idx
        model["x_corr_axes"] = x_corr_axes
        model["y_corr_axes"] = y_corr_axes
        model["q_weights"] = q_weights

        self.clear_viz_pub.publish("all")
        self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
        self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
        self.visualise_gc(model["GC"])
        self.visualise_directrix(model["directrix"], model["q"])
        # self.visualise_TNB_axes(model["eT"], model["directrix"], "eT")
        # self.visualise_TNB_axes(model["eN"], model["directrix"], "eN")
        # self.visualise_TNB_axes(model["eB"], model["directrix"], "eB")
        self.visualise_correction_axes(model["x_corr_axes"], model["y_corr_axes"], model["directrix"])

        crop = True

        while crop:
            strt = int(input("Crop size start? "))
            end = int(input("Crop size end? "))
            idx = np.array([strt, end]) #These values should be adjusted depending on demo
            xyz_model = self.get_model(demos_xyz, idx)
            
            min_start, min_end = self.get_minimums(processed_demos_xyz)
            xyz_model = self.saturate_model(xyz_model, min_start, min_end)

            x_corr_axes, y_corr_axes = self.get_correction_axes(xyz_model)

            q_mean = self.get_mean(demos_q, idx)
            q_weights = self.get_orientation_weights(processed_demos_q, q_mean, idx)
            model = xyz_model
            model["q"] = q_mean
            model["idx"] = idx
            model["x_corr_axes"] = x_corr_axes
            model["y_corr_axes"] = y_corr_axes
            model["q_weights"] = q_weights

            self.clear_viz_pub.publish("all")
            self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
            self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
            # First 50 points
            self.visualise_gc(model["GC"][:50,:,:])
            self.visualise_directrix(model["directrix"][:50,:], model["q"][:50,:])
            self.visualise_TNB_axes(model["eT"][:50,:], model["directrix"][:50,:], "eT")
            self.visualise_correction_axes(model["x_corr_axes"][:50,:], model["y_corr_axes"][:50,:], model["directrix"][:50,:])
            # Last 50 points
            self.visualise_gc(model["GC"][-50:,:,:])
            self.visualise_directrix(model["directrix"][-50:,:], model["q"][-50:,:])
            self.visualise_TNB_axes(model["eT"][-50:,:], model["directrix"][-50:,:], "eT")
            self.visualise_correction_axes(model["x_corr_axes"][-50:,:], model["y_corr_axes"][-50:,:], model["directrix"][-50:,:])

            if input("Crop again ? ([y]/n]") == "n":
                crop=False

        # Add a code to adjust the idx values in real time
        self.request_save(model)

        
if __name__ == "__main__":
    my_node = TLGCNode()  
    my_node.run()