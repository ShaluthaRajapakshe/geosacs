#!/usr/bin/env python3
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
from scipy.optimize import minimize


class TLGCNode(): 
    def __init__(self):
        rospy.init_node("tlgc_node") 

        # Variables
        self.rate = rospy.Rate(10) 
        self.data_dir = None
        self.processed_dir = None
        self.task = None
        self.process_data = False
        self.vertical_start = False
        self.vertical_end = False
        
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


    def make_horizontal(self, i, gc_circles, Rc, eT, eN, eB, directrix, global_x, global_y, global_z):

        T = eT[i,:]/np.linalg.norm(eT[i,:])
        cos_angle_z = np.clip(np.dot(T, global_z), -1.0, 1.0)
        angle_z = np.arccos(cos_angle_z)
        angle_z = np.degrees(angle_z)

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

        return gc_circles, eT, eN, eB
    
    def adjust_TNB(self, i, T, gc_circles, Rc, eT, eN, eB, directrix):

        T = T/np.linalg.norm(T)
        
        # Change N and B
        N = self.project_onto_plane(eN[i,:], T)

        # Normalise
        N = N/np.linalg.norm(N)
        B = np.cross(N, T)
        B = B/np.linalg.norm(B)
        
        # Redefine GC
        gc_circles[i,:,:] = circular_GC(np.array([directrix[i,:]]), np.array([N]), 
                                        np.array([B]), np.array([Rc[i]]))
        
        
        # Assign
        eT[i,:] = T
        eN[i,:] = N
        eB[i,:] = B

        return gc_circles, eT, eN, eB
    

    def make_vertical(self, i, gc_circles, Rc, eT, eN, eB, directrix, location=None):

        if location is not None:
            if location == "start":
                T = eT[0,:]/np.linalg.norm(eT[0,:])
            elif location == "end":
                T = eT[-1,:]/np.linalg.norm(eT[-1,:])
        else:
            T =  eT[i,:]/np.linalg.norm(eT[i,:])

        global_z = [0,0,1]

        projected_T = self.project_onto_plane(T, global_z)
        projected_T = projected_T / np.linalg.norm(projected_T)

        T = projected_T
        # Change N and B
        N = self.project_onto_plane(eN[i,:], T)

        # Normalise
        T = T/np.linalg.norm(T)
        N = N/np.linalg.norm(N)
        B = np.cross(N, T)
        B = B/np.linalg.norm(B)
        
        # Redefine GC

        # print("before verticaliztion", gc_circles[i,:,:30])
        gc_circles[i,:,:] = circular_GC(np.array([directrix[i,:]]), np.array([N]), 
                                        np.array([B]), np.array([Rc[i]]))
        
        # print("after verticaliztion", gc_circles[i,:,:30])

        # Assign
        eT[i,:] = T
        eN[i,:] = N
        eB[i,:] = B

        return gc_circles, eT, eN, eB


    

    ## This doesnt work well if the circles are too close. so we migth have to stop making them horizontal after some point!!!!!
    def check_intersections(self, i, nb_points, gc_circles, min_start, min_end, directrix, saturate_idxs):

        ## We have to decide where to stop looking at this. Maybe we can check this only at the edge or at the begining (if they are horizontal)

        # Horizontal flattening is only required if the start is horizontal
        if i <= int(nb_points/50) and (not self.vertical_start):
            circle = gc_circles[i+1,:,:]
            for point in circle:
                if point[2] <= directrix[i, 2]:
                    print("at the horizontal start, i < nb/2 and the z value of intersecting point", i, directrix[i, 2])
                    if (i+1) not in saturate_idxs:
                        saturate_idxs.append(i+1)
                    break
                    # self.make_horizontal(i+1, gc_circles,........)

        # Horizontal flattening is only required if the end is horizontal
        elif ( (i >= int(49*nb_points/50)) and (i < nb_points)  and (not self.vertical_end)):  ### Tune this value and think of a way
            circle = gc_circles[i-1,:,:]
            for point in circle:
                if point[2] <= directrix[i, 2]: 
                    print("at the horizontal end, i > nb/2 and the z value of intersecting point", i, directrix[i, 2])
                    if (i-1) not in saturate_idxs:
                        saturate_idxs.append(i-1)
                    break
        
        return saturate_idxs
    

    def relative_angle_between_planes(self, normal1, normal2):
        """
        Calculate the relative angle between two planes defined by their normal vectors.

        Parameters:
            normal1 (array_like): Normal vector of the first plane.
            normal2 (array_like): Normal vector of the second plane.

        Returns:
            float: Relative angle between the two planes (in radians).
        """
        # Normalize the normal vectors
        normal1_normalized = normal1 / np.linalg.norm(normal1)
        normal2_normalized = normal2 / np.linalg.norm(normal2)

        # Calculate the dot product between the normalized normal vectors
        dot_product = np.dot(normal1_normalized, normal2_normalized)

        # Clamp the dot product to the range [-1, 1] to avoid numerical errors
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # Calculate the relative angle using the inverse cosine (arccos) function
        relative_angle = np.arccos(dot_product)

        return relative_angle


    
    def check_intersections_simple(self, circle1, center2, normal1, normal2, i):

        for point in circle1:
            
            vector_pointing_to_directrix = (point[0] - center2[0], point[1] - center2[1], point[2] - center2[2])

            theta_val = self.calculate_angle_between_vectors(normal2, vector_pointing_to_directrix) #in degrees

            if theta_val < 90: ##Intersection exist

                # print("intersection exist in", i)

                ## check the current angle between the two planes/normal vector and the rotated normal
                angle_between_two_planes = self.relative_angle_between_planes(normal1, normal2)
                penalty = angle_between_two_planes
                return True, penalty

        else:
            # print("No intersection")
            return False, None
    

    def objective_function(self, angle, normal1, circle1, normal2, center2, i):
    
        axis = np.cross(normal1, normal2)
        rotated_normal2 = self.rodrigues_rotation(normal2, axis, angle[0])

        # Check for intersection
        intersection, penalty = self.check_intersections_simple(circle1, center2, normal1, rotated_normal2, i)
        if intersection:
            # Penalize intersection heavily
            return penalty

        return 0


    def optimize_orientation(self, i, gc_circles, directrix, eT):
    
        center2 = directrix[i+1]
        center1 = directrix[i]  

        normal2 = eT[i+1]
        normal1 = eT[i]

        # circle2 = gc_circles[i+1,:,:]
        circle1 = gc_circles[i,:,:]

        intersection_exist = False

        for point in circle1:
            vector_pointing_to_directrix = (point[0] - center2[0], point[1] - center2[1], point[2] - center2[2])
            theta_val = self.calculate_angle_between_vectors(normal2, vector_pointing_to_directrix) #in degrees
            if theta_val < 90: ##Intersection exist
                # print("intersection exist in", i)
                intersection_exist = True
                break
                ## check the current angle between the two planes/normal vector and the rotated normal
                # angle_between_two_planes = self.relative_angle_between_planes(normal1, normal2)
                # penalty = angle_between_two_planes
                
                
    
        if intersection_exist:
            initial_guess = self.relative_angle_between_planes(normal1, normal2)
            # print("initial angle between", i, "and ", i+1, " th cross sections", initial_guess)
            
            initial_guess = initial_guess*0.1 #initially we will adjust 10% from the initial angle between the two planes
            
        
            result = minimize(self.objective_function, initial_guess, args=(normal1, circle1, normal2, center2, i), bounds=[(-np.pi / 4, np.pi / 4)])

            optimized_angle = result.x[0]
            # print("the angle needed to rotate", optimized_angle)
            axis = np.cross(normal1, normal2)
            optimized_normal2 = self.rodrigues_rotation(normal2, axis, optimized_angle)

            angle_after_opt = self.relative_angle_between_planes(normal1, optimized_normal2)
            # print("######### after opt angle between", i, "and ", i+1, " th cross sections", angle_after_opt)

            return optimized_normal2
        
        else:
            print("No intersection returning the previous normal")
            return normal2
    

    # def optimize_orientation(self, i, gc_circles, directrix, eT):
    
    #     center2 = directrix[i+1]
    #     center1 = directrix[i]  

    #     normal2 = eT[i+1]
    #     normal1 = eT[i]

    #     # circle2 = gc_circles[i+1,:,:]
    #     circle1 = gc_circles[i,:,:]
    
    #     # initial_guess = [0.0]
    #     initial_guess = self.relative_angle_between_planes(normal1, normal2)
    #     print("initial angle between", i, "and ", i+1, " th cross sections", initial_guess)
        
    #     result = minimize(self.objective_function, initial_guess, args=(normal1, circle1, normal2, center2, i), bounds=[(-np.pi / 8, np.pi / 8)])

    #     optimized_angle = result.x[0]
    #     axis = np.cross(normal1, normal2)
    #     optimized_normal2 = self.rodrigues_rotation(normal2, axis, optimized_angle)

    #     angle_after_opt = self.relative_angle_between_planes(normal1, optimized_normal2)
    #     print("######### after opt angle between", i, "and ", i+1, " th cross sections", angle_after_opt)

    #     return optimized_normal2

    # Objective function for optimization
    def objective_function_bulk_normal(self, intermediate_normals, eT, circles, directrix):
        # Combine fixed start/end normals with the intermediate normals
        all_normals = np.vstack(([eT[0]], intermediate_normals.reshape((-1, 3)), [eT[-1]]))
        penalty = 0

        for i in range(len(all_normals) - 1):
            angle = self.relative_angle_between_planes(all_normals[i], all_normals[i + 1])
            penalty += angle

            intersection_exist,_ = self.check_intersections_simple(circles[i], directrix[i+1], all_normals[i], all_normals[i + 1], i)

            if intersection_exist:
                penalty += 10  # Large penalty for intersections
        print("### Penalty", penalty)
        return penalty

    def optimized_bulk_normal(self, model):
        gc_circles = model["GC"]
        eT = model["eT"]
        directrix = model["directrix"]

        num_intermediate = len(eT) - 2
        initial_guess = eT[1:-1].flatten()
        
        # Define bounds for each component of each intermediate normal
        bounds = [(-1, 1)] * (num_intermediate * 3)
        
        result = minimize(self.objective_function_bulk_normal, initial_guess, args=(eT, gc_circles, directrix), bounds=bounds)
        
        optimized_normals = result.x.reshape((-1, 3))
        eT[1:-1] = optimized_normals
        return eT

    
    def saturate_model(self, model, min_start, min_end):
        gc_circles = model["GC"]
        nb_points = gc_circles.shape[0]
        saturate_idxs = []
        minimum_z = min(min_start, min_end)  ## We can try two points for the two ends

        intersection_exist_at_start = True
        intersection_exist_at_end = True

        intersection_found_at_start = False
        intersection_found_at_end = False

        for i in range(nb_points):
            circle = gc_circles[i,:,:]
            for point in circle:
                # if point[2] <= minimum_z: 
                if point[2] <= min_start and (not self.vertical_start):
                    saturate_idxs.append(i)
                    break
            
                if point[2] <= min_end and (not self.vertical_end) and (point[2] not in saturate_idxs):
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
        
        #Flattening out the starting/ending circular cross sections ##Maybe just a one would also be sufficient rather than falttening a few
        # for i in saturate_idxs:
        #     print("saturate idxs", saturate_idxs)
        #     gc_circles, eT,eN, eB = self.make_horizontal(i, gc_circles, Rc, eT, eN, eB, directrix, global_x, global_y, global_z)
        #     # saturate_idxs = self.check_intersections(i, nb_points, gc_circles, min_start, min_end, directrix, saturate_idxs)


        ## correcting all the circles in the canal 
        for i in range(nb_points-1):

            optimized_normal2 = self.optimize_orientation(i, gc_circles, directrix, eT)
            gc_circles, eT, eN, eB = self.adjust_TNB(i+1, optimized_normal2, gc_circles, Rc, eT, eN, eB, directrix)


        print("###### Started checking for intersections ########")
        ### checking intersections exist only in neighboring cross sections
        for i in range(nb_points-1):
            intersection, penalty = self.check_intersections_simple(gc_circles[i,:,:], directrix[i+1], eT[i], eT[i+1], i)
            print(i, "intersection ", intersection, "angle between the two planes", penalty)

        ##TODO write a code to check intersections again on the whole canal
        ##TODO flatten the first and last circle and think of a way to do the optimization in the middle part with start and end fixed


        ## Second overlap check with non horizontal planes
        ####TODO ###Code for Horizontalizing the circles.  We might have to come up with a method where rather than just making horizontal but also changing the elevation of the circles as well
        #### If we just horizontalize then still overlapps can occur and also things become abnormal in some cases

        # while (intersection_exist_at_start or intersection_exist_at_end):

        #     if (not self.vertical_start):
        #         circle = gc_circles[int(nb_points/50)+1+1,:,:]
        #         directrix_point = directrix[int(nb_points/50)+1, 2]

        #         print("######################## at the start edge", directrix_point, int(nb_points/50), nb_points, directrix.shape, gc_circles.shape)

        #         for point in circle:
        #             if point[2] <= directrix_point:
        #                 intersection_found_at_start = True
        #                 # print("##### Adjesting the intersections at start", int(nb_points/50), 0)
        #                 print("before directrix correction", directrix[0:int(nb_points/50), 2])
        #                 for i in range (int(nb_points/50), 0, -1):
        #                     directrix[i, 2] = (directrix[i, 2] + directrix[i-1, 2]) / 2
        #                     print("############ In Here and the z point is, ", directrix[i, 2])
                        
        #                 # print("After directrix correction", directrix[0:int(nb_points/50), 2])
        #                 # directrix[int(nb_points/50), 2] = ( directrix[int(nb_points/50), 2] + directrix[int(nb_points/50)-1, 2] ) / 2
        #                 break
        #         else:
        #             intersection_found_at_start = False
        #             # print("No intersections with the")

        #         if not intersection_found_at_start:
        #             intersection_exist_at_start = False
        #             # print ("No more intersections at the start")
        #     else:
        #         intersection_exist_at_start  = False     

        #     if (not self.vertical_end):
        #         circle = gc_circles[int(49*nb_points/50)-1-1,:,:]
        #         # circle = gc_circles[int(49*nb_points/50)-1,:,:]
        #         directrix_point = directrix[int(49*nb_points/50)-1, 2]
        #         # directrix_point = directrix[int(49*nb_points/50), 2]

        #         # print("######################## at the end edge", directrix_point, int(49*nb_points/50)-1, int(49*nb_points/50)-1-1, nb_points, directrix.shape, gc_circles.shape)

        #         #### check here, we need to check starting from the 49*nb_points/50 th circle
        #         

        #         for point in circle:
        #             # print("Z value of the point", point[2])
        #             if point[2] <= directrix_point:
        #                 intersection_found_at_end = True
        #                 # print("point and directrix Z", point[2], directrix_point)
        #                 # print("##### Adjesting the intersections at end", int(49*nb_points/50)-1, nb_points-1)
        #                 # print("before directrix correction", directrix[int(49*nb_points/50)-1 : nb_points-1, 2])
        #                 for i in range (int(49*nb_points/50)-1, nb_points-1):
        #                     directrix[i, 2] = (directrix[i, 2] + directrix[i+1, 2]) / 2
        #                 # print("After directrix correction", directrix[int(49*nb_points/50)-1 : nb_points-1, 2])
        #                 break
        #                 # directrix[int(nb_points/50), 2] =s ( directrix[int(nb_points/50), 2] + directrix[int(nb_points/50)-1, 2] ) / 2
        #         else:
        #             intersection_found_at_end = False


        #         if not intersection_found_at_end:
        #             intersection_exist_at_end = False
        #             # print (intersection_exist_at_start, intersection_exist_at_end, "No more intersections at the end")
        #     else:
        #         intersection_exist_at_end  = False   

        #     #Use the above functiop
        # ### In here for horizontal planes, we have to again go through the previous circle and check any point of it has been intersecting with the current circle


        # Update model
        model["GC"] = gc_circles
        model["eT"] = eT
        model["eN"] = eN
        model["eB"] = eB

        rospy.loginfo(f"  Model saturation completed.")
         
        return model


    def saturate_model_new(self, model, min_start, min_end):
        gc_circles = model["GC"]
        nb_points = gc_circles.shape[0]
        saturate_idxs = []

        if not self.vertical_start:
            circle = gc_circles[0,:,:]
            for point in circle:
                if point[2] <= min_start:
                    saturate_idxs.append(0)
                    break
        if not self.vertical_end:
            circle = gc_circles[-1,:,:]
            for point in circle:
                if point[2] <= min_end:
                    saturate_idxs.append(nb_points-1)
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
        
        #Flattening out the starting/ending circular cross sections ##Maybe just a one would also be sufficient rather than falttening a few
        for i in saturate_idxs:
            print("saturate idxs", saturate_idxs)
            gc_circles, eT,eN, eB = self.make_horizontal(i, gc_circles, Rc, eT, eN, eB, directrix, global_x, global_y, global_z)
            # saturate_idxs = self.check_intersections(i, nb_points, gc_circles, min_start, min_end, directrix, saturate_idxs)

        #Run bulk optimization for middle normals
        eT = self.optimized_bulk_normal(model)
        print("######################################### Optimization done for eT")

        ## correcting all the circles in the canal accoring to the new normals
        for i in range(nb_points-2):
            gc_circles, eT, eN, eB = self.adjust_TNB(i+1, eT[i+1], gc_circles, Rc, eT, eN, eB, directrix)


        print("###### Started checking for intersections ########")
        ### checking intersections exist only in neighboring cross sections
        for i in range(nb_points-1):
            intersection, penalty = self.check_intersections_simple(gc_circles[i,:,:], directrix[i+1], eT[i], eT[i+1], i)
            print(i, "intersection ", intersection, "angle between the two planes", penalty)


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

    def get_correction_x_axes(self, model):

        eT = model["eT"]
        directrix = model["directrix"]
        global_x = np.array([1, 0, 0])
        global_y = np.array([0, 1, 0])
        global_z = np.array([0, 0, 1])
        nb_points = directrix.shape[0]

        x_corr_axes = np.zeros((nb_points, 3))
        previous_correction_x_axis = None

        window_size = 10 ## We can play with this to see the effect
    
        for i in range(nb_points):
            eT_curr =eT[i,:]
    
            # Alternative method method
            T_normalized = eT_curr / np.linalg.norm(eT_curr)
            projected_global_x = self.project_onto_plane(global_x, T_normalized)
            correction_x_axis = projected_global_x

            if i == 0 : 
                previous_correction_x_axis = correction_x_axis
                x_corr_axes[i,:] = correction_x_axis
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

            print(f"theta_X = {theta_X}, theta_C = {theta_C}")
            t = np.radians(theta_X)/(np.pi/2)
            
            correction_x_axis = self.slerp(projected_global_x, projected_previous_correction, t, theta_C)
            previous_correction_x_axis = correction_x_axis
         
            # Normalise and append
            correction_x_axis /= np.linalg.norm(correction_x_axis)
            x_corr_axes[i,:] = correction_x_axis

        return x_corr_axes
    


    def get_correction_y_axes(self, model, x_corr_axes):
        eT = model["eT"]
        directrix = model["directrix"]

        nb_points = directrix.shape[0]

        y_corr_axes = np.zeros((nb_points, 3))
      
        ##newly added
        previous_correction_y_axes = []  #we will use this list to find the average vector within the defined window
        window_size = 10 ## We can play with this to see the effect

    
        for i in range(nb_points):
            eT_curr =eT[i,:]
            
            # Alternative method method
            T_normalized = eT_curr / np.linalg.norm(eT_curr)

            correction_x_axis = x_corr_axes[i,:]
            correction_y_axis = np.cross(T_normalized, correction_x_axis)

            # # Newly added code for correction y axis
            ### Handling for correction  Y axis

            if i >= window_size:

                ## Newly Added on 08th April 2024
                previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes[:window_size])
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
                   
            

            # Normalise and append
         
            correction_y_axis /= np.linalg.norm(correction_y_axis)

            previous_correction_y_axes.append(correction_y_axis)

            y_corr_axes[i,:] = correction_y_axis

        return y_corr_axes
    

    def smoothen_correction_y_axes(self, model, y_corr_axes):
        eT = model["eT"]
        directrix = model["directrix"]
        refine_window_size = 10

        nb_points = directrix.shape[0]
        t = 0.2

        for i in range(int(0.1*nb_points), int(0.9*nb_points)):

            eT_curr =eT[i,:]
            
            T_normalized = eT_curr / np.linalg.norm(eT_curr)

            previous_mean_correction_y_axis = self.mean_direction(y_corr_axes[i-refine_window_size:i])
            next_mean_correction_y_axis = self.mean_direction(y_corr_axes[i+1:i+1+refine_window_size])

            projected_previous_mean_y_correction = self.project_onto_plane(previous_mean_correction_y_axis, T_normalized)
            projected_next_mean_y_correction = self.project_onto_plane(next_mean_correction_y_axis, T_normalized)

            mean_y_angle_diff = self.calculate_angle_between_vectors(projected_previous_mean_y_correction, projected_next_mean_y_correction)

            ## Before doing slerp we might have to introduce a thereshold so that the orthogonality will be only removed at those points
            ## Also ensure that the y axis are within the disk
            if mean_y_angle_diff > 60:

                corrected_y_corr_axis = self.slerp(projected_previous_mean_y_correction, projected_next_mean_y_correction, t, mean_y_angle_diff)
            
                y_corr_axes[i] = corrected_y_corr_axis

        return y_corr_axes

        
    def start_end_check(self, model):
        eT = model["eT"]
        directrix = model["directrix"]
        global_x = np.array([1, 0, 0])
        global_y = np.array([0, 1, 0])
        global_z = np.array([0, 0, 1])

        window_size = 10

        mean_eT_dir_at_start = self.mean_direction(eT[:window_size])
        mean_eT_dir_at_end = self.mean_direction(eT[-window_size:])

        z_diff_at_start = self.calculate_angle_between_vectors(mean_eT_dir_at_start, global_z)
        z_diff_at_end = self.calculate_angle_between_vectors(mean_eT_dir_at_end, global_z)


        if z_diff_at_start < 90:
            if z_diff_at_start > 45:
                self.vertical_start = True 
                print("Vertical start", z_diff_at_start)
            else:
                self.vertical_start = False
                print("Horizontal start", z_diff_at_start)
        else:
            if z_diff_at_start < 145:
                self.vertical_start = True  
                print("Vertical start", z_diff_at_start)
            else:
                self.vertical_start = False
                print("Horizontal start", z_diff_at_start)

        if z_diff_at_end < 90: # check for greater than 90 scenrios
            if z_diff_at_end > 45:
                self.vertical_end = True 
                print("Vertical end", z_diff_at_end)
            else:
                self.vertical_end = False
                print("Horizontal end", z_diff_at_end)

        else:
            if z_diff_at_end < 145:
                self.vertical_end = True
                print("Vertical end", z_diff_at_end)
            else:
                self.vertical_end = False
                print("Horizontal end", z_diff_at_end)
            



    
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
        forward_window = 50

        y_axis_changed = False
        # vertical_end = False
        # verticl_start = False
        

        self.corr_y_changed_idx_xyz = None

        

        # print("############## z_diff_at_start", z_diff_at_start, "z_diff_at_end", z_diff_at_end)





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
            # print(f"theta_X = {theta_X}, theta_C = {theta_C}")
            t = np.radians(theta_X)/(np.pi/2)
            
            correction_x_axis = self.slerp(projected_global_x, projected_previous_correction, t, theta_C)

            # # Classic code for y axis
            # correction_y_axis = np.cross(T_normalized, correction_x_axis)

            # # Newly added code for correction y axis
            ### Handling for correction  Y axis 
            

            if i >= window_size:
                # previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes[-window_size:])
                previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes[:window_size])
                projected_previous_mean_y_correction = self.project_onto_plane(previous_mean_correction_y_axis, T_normalized)
                correction_y_axis = np.cross(T_normalized, correction_x_axis)
                y_angle_diff = self.calculate_angle_between_vectors(correction_y_axis, projected_previous_mean_y_correction)

                both_vertical = (self.vertical_end) and (self.vertical_start)
                both_horizontal = (not self.vertical_end) and (not self.vertical_start)

                #If both ends are either vertical or horizontal
                if (both_vertical) or (both_horizontal): 

                    if i < eT.shape[0]-forward_window-1:
                        eT_mean =  self.mean_direction(eT[i+1:i+forward_window])
                        eT_mean_diff_with_z = self.calculate_angle_between_vectors(eT_mean, global_z)
                        curr_eT_diff_with_z = self.calculate_angle_between_vectors(T_normalized, global_z)

                        z_diff = np.abs(eT_mean_diff_with_z - curr_eT_diff_with_z)

                        if y_axis_changed: #Should only apply if both ends are horizontal
                            correction_y_axis = -correction_y_axis
                        
                        if z_diff > 30 and (not y_axis_changed): #Should only apply if both ends are horizontal
                            # print(i, "eT mean angle", eT_mean_diff_with_z, " curr eT angle", curr_eT_diff_with_z, z_diff)

                            if y_angle_diff > 90:
                                correction_y_axis = -correction_y_axis
                                y_axis_changed  = True
                                self.corr_y_changed_idx_xyz = directrix[i]

                    else:
                        ## We have to change the variable vertical end by identifying whether an end is horizontal or vertical
                        if y_angle_diff > 90:
                            correction_y_axis = -correction_y_axis

                else: ###  for one horizontal and one vertical end 
                    previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes[-window_size:])
                    projected_previous_mean_y_correction = self.project_onto_plane(previous_mean_correction_y_axis, T_normalized)
                    correction_y_axis = np.cross(T_normalized, correction_x_axis)
                    y_angle_diff = self.calculate_angle_between_vectors(correction_y_axis, projected_previous_mean_y_correction)
                
                    if y_angle_diff > 90:
                        correction_y_axis = -correction_y_axis


                # if y_angle_diff > 90:
                #     # correction_y_axis = -correction_y_axis
                    
                #     #####TO DO: THIS SHOULD BE ONLY PERFORMED IF BOTH ENDS ARE HORIZONTAL # WE CAN FIND IT THROUGH THE NEW METHOD


                #     if i < eT.shape[0]-forward_window-1:
                #         eT_mean =  self.mean_direction(eT[i+1:i+forward_window])
                #         eT_mean_diff_with_z = self.calculate_angle_between_vectors(eT_mean, global_z)
                #         curr_eT_diff_with_z = self.calculate_angle_between_vectors(T_normalized, global_z)

                        

                #         z_diff = np.abs(eT_mean_diff_with_z - curr_eT_diff_with_z)

                #         if y_axis_changed:
                #             correction_y_axis = -correction_y_axis
                        
                #         if z_diff > 30 and (not y_axis_changed):
                #             print(i, "eT mean angle", eT_mean_diff_with_z, " curr eT angle", curr_eT_diff_with_z, z_diff)
                            
                #             correction_y_axis = -correction_y_axis
                #             y_axis_changed  = True

                #     else:
                #         print("HEEEEU")
                #         correction_y_axis = -correction_y_axis

            
                # ## Newly Added on 08th April 2024
                # if i < nb_points*0.1 or i > nb_points*0.1:
                #     previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes[:window_size])
                #     projected_previous_mean_y_correction = self.project_onto_plane(previous_mean_correction_y_axis, T_normalized)
                #     correction_y_axis = np.cross(T_normalized, correction_x_axis)
                #     y_angle_diff = self.calculate_angle_between_vectors(correction_y_axis, projected_previous_mean_y_correction)

                #     if y_angle_diff > 90:
                #                 correction_y_axis = -correction_y_axis

                # else:
                #     previous_mean_correction_y_axis = self.mean_direction(previous_correction_y_axes[:window_size])
                #     projected_previous_mean_y_correction = self.project_onto_plane(previous_mean_correction_y_axis, T_normalized)
                #     correction_y_axis = np.cross(T_normalized, correction_x_axis)



                # print (eT.shape)
                ## Check the future mean direction of the tangent vectors to identify noise 
                # if i < eT.shape[0]-forward_window-1:
                #     eT_mean =  self.mean_direction(eT[i+1:i+forward_window])
                #     eT_angle_diff = self.calculate_angle_between_vectors(eT_mean, T_normalized)
                #     if eT_angle_diff > 45:
                #         if y_angle_diff > 90:
                #             correction_y_axis = -correction_y_axis
                # else:
                #     if y_angle_diff > 90:
                #             correction_y_axis = -correction_y_axis
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
    

    def get_furthest_start(self, processed_demos_xyz):

        starting_x = processed_demos_xyz[0,0,0]
        starting_y = processed_demos_xyz[0,0,1]

        furthest_start_point_on_xy = np.sqrt(starting_x**2 + starting_y**2)

        return furthest_start_point_on_xy
    
    def get_furthest_end(self, processed_demos_xyz):

        ending_x = processed_demos_xyz[-1,0,0]
        ending_y = processed_demos_xyz[-1,0,1]

        furthest_end_point_on_xy = np.sqrt(ending_x**2 + ending_y**2)

        return furthest_end_point_on_xy


    def saturate_model_vertical(self, model): #Verticalizing the ends only for the 5% of all the cicles 

        gc_circles = model["GC"]
        nb_points = gc_circles.shape[0]
        saturate_start_idxs = []
        saturate_end_idxs = []

        global_z = np.array([0, 0, 1])

        eT = model["eT"]
        eN = model["eN"]
        eB = model["eB"]
        directrix = model["directrix"]
        Rc = model["Rc"]

      
        if self.vertical_start:

            tangent_at_start = eT[0,:] / np.linalg.norm(eT[0,:]) #
            projected_vertical_start_tangent = self.project_onto_plane(tangent_at_start, global_z)
            projected_vertical_start_tangent = projected_vertical_start_tangent / np.linalg.norm(projected_vertical_start_tangent)

            eT[0,:] = projected_vertical_start_tangent

            #verticalizing the first circle
            gc_circles, eT,eN, eB = self.make_vertical(-1, gc_circles, Rc, eT, eN, eB, directrix)

            ### Here we have to ensure that the starting circle is orthogonal to the projected vector ::: Need to write a verticalizing function

            directrix_at_start = directrix[0]
            
            for i in range(int(nb_points/20)):
                circle = gc_circles[i+1,:,:]
                for point in circle:
                    vector_pointing_to_directrix = (directrix_at_start[0] - point[0], directrix_at_start[1] - point[1], directrix_at_start[2] - point[2])

                    theta_val = self.calculate_angle_between_vectors(projected_vertical_start_tangent, vector_pointing_to_directrix) #in degrees

                    if theta_val < 90: ##Intersection exist
                        saturate_start_idxs.append(i+1)
                        break
                
        if self.vertical_end:

            tangent_at_end = eT[-1,:] / np.linalg.norm(eT[-1,:]) #
            projected_vertical_end_tangent = self.project_onto_plane(tangent_at_end, global_z)
            projected_vertical_end_tangent = projected_vertical_end_tangent / np.linalg.norm(projected_vertical_end_tangent)

            eT[-1,:] = projected_vertical_end_tangent

            #verticalizing the final circle
            gc_circles, eT,eN, eB = self.make_vertical(-1, gc_circles, Rc, eT, eN, eB, directrix)

            directrix_at_end = directrix[-1]

            print("Found a vertical end and the shape of gc_circles is", gc_circles.shape)

            for j in range(nb_points-1, int(19*nb_points/20), -1):
                circle = gc_circles[j-1,:,:]
                for point in circle:
                    vector_pointing_to_directrix = (directrix_at_end[0] - point[0], directrix_at_end[1] - point[1], directrix_at_end[2] - point[2])

                    theta_val = self.calculate_angle_between_vectors(projected_vertical_end_tangent, vector_pointing_to_directrix) #in degrees

                    if theta_val < 90: ##Intersection exist
                        print("intersection exist")
                        saturate_end_idxs.append(j-1)
                        break

        
        for i in saturate_start_idxs:
            print("saturate_start_idxs", saturate_start_idxs)
            gc_circles, eT,eN, eB = self.make_vertical(i, gc_circles, Rc, eT, eN, eB, directrix, location = "start")

        for i in saturate_end_idxs:
            print("saturate_end_idxs", saturate_end_idxs)
            gc_circles, eT,eN, eB = self.make_vertical(i, gc_circles, Rc, eT, eN, eB, directrix, location = "end")
            ##TODO We have to agaon check for intersections after verticalization. We migth need to use the same tangent as in the last point for
            # the all tangent vectors of the verticalized planes. Basically replacing the tangent vectors with the final point's tangent. Otherwise
            # still intersections can be seen. Look at the images 


        # Update model
        model["GC"] = gc_circles
        model["eT"] = eT
        model["eN"] = eN
        model["eB"] = eB

        rospy.loginfo(f"  Model saturation completed.")
         
        return model


    def optimize_canal(self, model):

        ### First check for horizontal and vertical start/ends
        ### if horizontal end/start flatten the first 5% of the circles (do this for both ends)
        ### then start the optimization process untill the end of the canal. (the optimization process should be able to make the canal circles
        ### intersection free when we reach the end point. Note that the orientation of the first 5% and the last 5% of circles will be fixed)

        ### start from the first circle next to 5%th circle and iterate through the circle before the last 5%th circle
        ### first check whether if there are any intersections,
        ### if an intersetion exist, find the relative angle between the two circular planes (the initial guess for the optimization function should 
        ### be the currnet relative angle between the two planes)
        ### Then use an objective fuction to minimize this angle until there is no intersection.
        return model

    def run(self):
        self.data_dir = "/home/shalutha/TLGC_data"
        self.task = input("What is the name of the task? ")
        # self.task = "laundry_loop3"
        raw_dir = self.data_dir + f"/{self.task}/record-raw"
        processed_dir = self.data_dir + f"/{self.task}/record-processed"
        
        raw_demos_xyz, raw_demos_q = self.get_raw_demos(raw_dir)
        processed_demos_xyz, processed_demos_q = self.get_processed_demos(processed_dir)
        demos_xyz, demos_q = self.reshape_demos(processed_demos_xyz, processed_demos_q)
        
        idx = np.array([20,20]) #These values should be adjusted depending on demo #3,20 for tnb vs tny  #20,70 for task 1 and 10,10 for task2 
        xyz_model = self.get_model(demos_xyz, idx)
        
        # min points along the z axis
        min_start, min_end = self.get_minimums(processed_demos_xyz)

        self.start_end_check(xyz_model)

        # # furthest points along the xy plane
        # if self.vertical_start:
        #     furthest_start = self.get_furthest_start(processed_demos_xyz)

        # if self.vertical_end:
        #     furthest_end = self.get_furthest_end(processed_demos_xyz)

        # xyz_model = self.saturate_model(xyz_model, min_start, min_end)

        x_corr_axes, y_corr_axes = self.get_correction_axes(xyz_model)

        # x_corr_axes = self.get_correction_x_axes(xyz_model)
        # y_corr_axes = self.get_correction_y_axes(xyz_model, x_corr_axes)
        # y_corr_axes = self.smoothen_correction_y_axes(xyz_model, y_corr_axes)

        q_mean = self.get_mean(demos_q, idx)
        q_weights = self.get_orientation_weights(processed_demos_q, q_mean, idx)
        model = xyz_model
        model["q"] = q_mean
        model["idx"] = idx
        model["x_corr_axes"] = x_corr_axes
        model["y_corr_axes"] = y_corr_axes
        model["q_weights"] = q_weights
        model["vertical_start"] = self.vertical_start
        model["vertical_end"] = self.vertical_end

        if self.corr_y_changed_idx_xyz is not None:
            model["xyz_corr_y"] = self.corr_y_changed_idx_xyz
        else:
            model["xyz_corr_y"] = None

        self.clear_viz_pub.publish("all")
        # self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
        # self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
        # self.visualise_gc(model["GC"])
        # self.visualise_gc(model["GC"][200:335,:,:])
        # self.visualise_directrix(model["directrix"], model["q"])
        # self.visualise_TNB_axes(model["eT"], model["directrix"], "eT")
        # self.visualise_TNB_axes(model["eN"], model["directrix"], "eN")
        # self.visualise_TNB_axes(model["eB"], model["directrix"], "eB")
        # self.visualise_correction_axes(model["x_corr_axes"], model["y_corr_axes"], model["directrix"])

        crop = True

        while crop:
            strt = int(input("Crop size start? "))
            end = int(input("Crop size end? "))
            idx = np.array([strt, end]) #These values should be adjusted depending on demo
            xyz_model = self.get_model(demos_xyz, idx)
            
            min_start, min_end = self.get_minimums(processed_demos_xyz)

            self.start_end_check(xyz_model)

            # xyz_model_prev = self.saturate_model(xyz_model, min_start, min_end)
            # xyz_model = self.saturate_model(xyz_model, min_start, min_end)
            xyz_model = self.saturate_model_new(xyz_model, min_start, min_end)

            xyz_model = self.saturate_model_vertical(xyz_model)

            x_corr_axes, y_corr_axes = self.get_correction_axes(xyz_model)

            q_mean = self.get_mean(demos_q, idx)
            q_weights = self.get_orientation_weights(processed_demos_q, q_mean, idx)
            model = xyz_model
            model["q"] = q_mean
            model["idx"] = idx
            model["x_corr_axes"] = x_corr_axes
            model["y_corr_axes"] = y_corr_axes
            model["q_weights"] = q_weights
            # model["xyz_corr_y"] = self.corr_y_changed_idx_xyz  #TO DO:  change this when the ends are vertical

            if self.corr_y_changed_idx_xyz is not None:
                model["xyz_corr_y"] = self.corr_y_changed_idx_xyz
            else:
                model["xyz_corr_y"] = [0,0,0]

            model["vertical_start"] = self.vertical_start
            model["vertical_end"] = self.vertical_end


            self.clear_viz_pub.publish("all")

            # self.visualise_gc(model["GC"])

            # self.visualise_demos(raw_demos_xyz, raw_demos_q, "raw" )
            # self.visualise_demos(processed_demos_xyz, processed_demos_q, "processed" )
            # # First 50 points
            # self.visualise_gc(model["GC"][:50,:,:])
            # self.visualise_directrix(model["directrix"][:50,:], model["q"][:50,:])
            # self.visualise_TNB_axes(model["eT"][:50,:], model["directrix"][:50,:], "eT")
            # self.visualise_correction_axes(model["x_corr_axes"][:50,:], model["y_corr_axes"][:50,:], model["directrix"][:50,:])
            # # Last 50 points
            # self.visualise_gc(model["GC"][-50:,:,:])
            # self.visualise_directrix(model["directrix"][-50:,:], model["q"][-50:,:])
            # self.visualise_TNB_axes(model["eT"][-50:,:], model["directrix"][-50:,:], "eT")

            # self.visualise_gc(model["GC"][-10:,:,:])
            # self.visualise_gc(model["GC"][300:,:,:])
            self.visualise_gc(model["GC"])
            # self.visualise_directrix(model["directrix"][-50:,:], model["q"][-50:,:])
            self.visualise_TNB_axes(model["eT"][-10:,:], model["directrix"][-10:,:], "eT")

            
            # self.visualise_correction_axes(model["x_corr_axes"][-5:,:], model["y_corr_axes"][-5:,:], model["directrix"][-5:,:])

            if input("Crop again ? ([y]/n]") == "n":
                crop=False

        # Add a code to adjust the idx values in real time
        self.request_save(model)

        
if __name__ == "__main__":#
    my_node = TLGCNode()  
    my_node.run()