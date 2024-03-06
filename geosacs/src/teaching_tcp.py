#!/usr/bin/env python
import rospy
import os
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from custom_msgs.srv import LoadData, LoadDataRequest
from rospy.exceptions import ROSException, ROSInterruptException
from std_msgs.msg import String, Empty
from pathlib import Path
from scipy.spatial.transform import Rotation 



class TeachingTCP(): 
    def __init__(self):
        rospy.init_node("teaching_tcp") 

        # Variables
        self.move_rate = rospy.Rate(10)
        self.recording = False  
        self.observations = {"position":[], "orientation":[]}
        self.demo_index = 1
        self.release_to_record = False
        self.frame = "LIO_base_link"

        # ROS Variables
        rospy.Subscriber("/lio_1c/pose", PoseStamped, self.pose_cb )
        rospy.Subscriber("/service2topic/status", String, self.status_cb)
        self.lio_req_tcp_pub = rospy.Publisher("/request_tcp_pub", String, queue_size=2)
        self.commanded_pos_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=2)
        self.lio_status_req_pub = rospy.Publisher("/service2topic/request", String, queue_size=2)
        self.tlgc_load_data_client = rospy.ServiceProxy("/tlgc/load_data_raw", LoadData)
        # # Visualisation
        self.raw_traj_pub = rospy.Publisher("/tlgc/raw_trajectory", PoseArray, queue_size=10)
        self.clear_viz_pub = rospy.Publisher("/tlgc/viz_clear", String, queue_size=2)
        self.raw_pose_traj_pub = rospy.Publisher("/tlgc/raw_pose_trajectory", PoseArray, queue_size=2)

        #Path Variables (absolute)
        self.data_directory = Path("/home/jurassix16/TLGC_data") #/home/atharvad/TLGC_data
        
        #Init Message
        rospy.sleep(1)
        rospy.loginfo("teaching_tcp has been started") 


    def status_cb(self, msg):

        # Update released class variable
        if msg.data == "RELEASED": self.recording = True
        else: self.recording= False


    def extract_position(self, msg):

        position = msg.pose.position
        point3d = [position.x, position.y, position.z]
        return point3d
    

    def pose_cb(self, msg):

        if self.recording:            
            # Save position
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            orientation = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]

            self.observations["position"].append(position)
            self.observations["orientation"].append(orientation)    
    

    def change_frame(self, raw_dict):

        # Define transform matrix
        angle =  1.570
        R_ab = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle) , 0],
                         [0            , 0             , 1]])
        p_ab = np.array([[0, 0, 0.266]]).reshape(3,1)
        _ = np.array([[0, 0, 0, 1]])
        T_ab = np.column_stack((R_ab, p_ab))
        T_ab = np.vstack((T_ab, _))

        # Get and check data
        positions_b = raw_dict["position"]
        orientations_b = raw_dict["orientation"]
        if positions_b.shape[0] != orientations_b.shape[0]:
            rospy.logerr("change_frame function: Nb positions != nb orientations")
            return
        
        # Transform data into new frame
        nb_points = positions_b.shape[0]
        positions_a = np.zeros(positions_b.shape)
        orientations_a = np.zeros(orientations_b.shape)

        for i in range(nb_points):
            # Position
            pos_b = np.append(positions_b[i,:],1)
            pos_b = np.array([pos_b]).reshape(4,1)
            pos_a = np.dot(T_ab, pos_b)
            pos_a = np.ravel(pos_a)
            pos_a = pos_a[:-1]
            positions_a[i,:] = pos_a
            # Orientation
            q_b = orientations_b[i,:]
            R_b = Rotation.from_quat([q_b[1], q_b[2], q_b[3], q_b[0]]).as_matrix()
            R_a = np.dot(R_ab, R_b)
            q_a = Rotation.from_matrix(R_a).as_quat()
            q_a = np.array([[q_a[3],q_a[0], q_a[1], q_a[2]]])
            orientations_a[i,:] = q_a

        # Update data
        raw_dict["position"] = positions_a
        raw_dict["orientation"] = orientations_a
        
        return raw_dict    
       
    def playback(self, raw_dict):
 
        print(f"[*] Starting Playback for Demonstration `{self.demo_index}`...")

        # Get and check data 
        positions = raw_dict["position"]
        orientations = raw_dict["orientation"]
        if orientations.shape[0] != positions.shape[0] : 
            rospy.logerr("playback functin: Nb positions != nb orientations")
            return
        nb_points = positions.shape[0]
        
        # Initial pose
        initPose = PoseStamped()
        initPose.pose.position.x = positions[0,0]
        initPose.pose.position.y = positions[0,1]
        initPose.pose.position.z = positions[0,2]

        initPose.pose.orientation.w = orientations[0,0]
        initPose.pose.orientation.x = orientations[0,1]
        initPose.pose.orientation.y = orientations[0,2]
        initPose.pose.orientation.z = orientations[0,3]

        initPose.header.frame_id = self.frame
        initPose.header.stamp = rospy.Time.now()
        self.commanded_pos_pub.publish(initPose)
        self.lio_req_tcp_pub.publish("stop")       


        # Rest of the poses
        resp = input("Enter to start playback ?")
        for i in range(nb_points):
            pose = PoseStamped()
            pose.pose.position.x = positions[i,0]
            pose.pose.position.y = positions[i,1]
            pose.pose.position.z = positions[i,2]

            pose.pose.orientation.w = orientations[i,0]
            pose.pose.orientation.x = orientations[i,1]
            pose.pose.orientation.y = orientations[i,2]
            pose.pose.orientation.z = orientations[i,3]

            pose.header.frame_id = self.frame
            pose.header.stamp = rospy.Time.now()
            self.commanded_pos_pub.publish(pose)
            self.move_rate.sleep()
        self.lio_req_tcp_pub.publish("stop")       

    
    def call_tlgc_load_data_raw(self, data_dir, task):

        request = LoadDataRequest()  
        request.data_directory = data_dir
        request.task = task

        try:
            response = self.tlgc_load_data_client(request)
            if response.success: rospy.loginfo(f"Success: {response.message}")
            else: rospy.loginfo(f"Error: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

   
    def request_save(self, demo_file, raw_dict):

        if input("Save this demo? (y/n) ")=="y": 
            np.savez(str(demo_file), **raw_dict)
            print(f"Demo {self.demo_index} saved in {demo_file}")
        else: print(f"Demo {self.demo_index} not saved")


    def record_demo(self):
        print(f"[*] Starting to Record Demonstration `{self.demo_index}`...")
        user_input = input("Ready to record! ENTER after placing Lio in starting position !")

        # Reset...
        if user_input.startswith("r"):
            return None
        
        # Clear previous data
        self.observations["position"].clear()
        self.observations["orientation"].clear()

        # Recording will start&stop when Lio released.
        self.lio_status_req_pub.publish("start")
        print("Recording will start when Lio released.")
        self.release_to_record = True
        while not self.recording:
            rospy.sleep(0.1)
        print("Recording...")
        while self.recording:
            rospy.sleep(0.1)
        print("Recording stopped.")
        self.release_to_record = False
        self.lio_status_req_pub.publish("stop")

        # Get Data
        raw_list_pos = self.observations["position"]
        raw_list_ori = self.observations["orientation"]

        # Check data
        if len(raw_list_pos) == 0:
            rospy.logerr("No data points record. Abort")
            exit()

        # Convert to np array
        raw_array_pos = np.zeros((len(raw_list_pos),len(raw_list_pos[0])))
        for i in range(len(raw_list_pos)):
            raw_array_pos[i] = np.array(raw_list_pos[i])
        raw_array_ori = np.zeros((len(raw_list_ori), len(raw_list_ori[0])))
        for i in range(len(raw_list_ori)):
            raw_array_ori[i] = np.array(raw_list_ori[i])
       
        # Display messages
        print("Position np.array shape", raw_array_pos.shape)
        print("Orientation np.array shape", raw_array_ori.shape)

        # Convert to dictionary
        raw_dict = {"position": raw_array_pos, "orientation": raw_array_ori}

        return raw_dict


    def visualise_demo(self, raw_dict):
        
        # Get and check data
        positions = raw_dict["position"]
        orientations = raw_dict["orientation"]
        if positions.shape[0] != orientations.shape[0] : 
            rospy.logerr("Cannot visualise pose trajectory. Nb positions != nb orientations")
            return 
        
        # Format
        msg = PoseArray()
        nb_points = positions.shape[0]
        for i in range(nb_points):
            pose = Pose()
            pose.position.x = positions[i,0]
            pose.position.y = positions[i,1]
            pose.position.z = positions[i,2]

            pose.orientation.w = orientations[i,0]
            pose.orientation.x = orientations[i,1]
            pose.orientation.y = orientations[i,2]
            pose.orientation.z = orientations[i,3]

            msg.poses.append(pose)
            msg.header.frame_id = self.frame

        # Publish for visualisation
        self.raw_pose_traj_pub.publish(msg)
        self.raw_traj_pub.publish(msg)
    

    def run(self):
        more = True
        task = input("What is the name of the task? ")
        # task = "bin"
        raw_dir = self.data_directory / task / "record-raw"
        print(raw_dir)
        os.makedirs(raw_dir, exist_ok=True)

        self.clear_viz_pub.publish("all")

        while more:
            demo_file = raw_dir / f"{task}-{self.demo_index}.npz"
            raw_dict = self.record_demo()
            raw_dict = self.change_frame(raw_dict)
            self.visualise_demo(raw_dict)
            while input("Playback? (y/[n])") == "y":
                self.playback(raw_dict)
            self.request_save(demo_file, raw_dict)
            if input("Another demo? (y/n) ")=="n": more = False
            self.clear_viz_pub.publish("all")
            self.demo_index += 1


if __name__ == "__main__":
    my_node = TeachingTCP()  
    my_node.run()