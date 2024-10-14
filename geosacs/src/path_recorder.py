#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from sensor_msgs.msg import Joy
from geosacs.msg import SurfacePose
from std_msgs.msg import String

class PathRecorder(): 
    def __init__(self):
        rospy.init_node("path_recorder") 

        # Variables
        self.rate = rospy.Rate(20) 
        self.path = Path() 
        self.path.header.frame_id = "LIO_robot_base_link"
        self.clear = False
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.state = None
        self.start = False
        self.stop = False
        self.save_correction_traj = False
        self.save_autonomous_traj = False
        self.force_load = False

        
        self.current_trajectory = []
        self.correction_trajectories = []
        self.autonomous_trajectories = []

        self.pose_type = "commanded"
        if self.pose_type == "real":
            rospy.Subscriber("/lio_1c/pose", PoseStamped, self.lio_pose_cb)
            rospy.Subscriber("/tlgc/surface_pose", SurfacePose, self.surface_pose_cb)
        elif self.pose_type == "commanded":
            rospy.Subscriber("/commanded_pose", PoseStamped, self.commanded_pose_cb)
            rospy.Subscriber("/tlgc/surface_pose", SurfacePose, self.surface_pose_alternative_cb)
        else:
            rospy.logerr("Pose type unidentified.")


        rospy.Subscriber("/joy", Joy, self.joy_cb)
        rospy.Subscriber("/tlgc/events", String, self.events_cb)
        

        self.path_pub = rospy.Publisher("/full_trajectory", Path, queue_size=10)
        self.autonomous_traj_pub = rospy.Publisher("/tlgc/autonomous_trajectory", PoseArray, queue_size=10)
        self.corrected_traj_pub = rospy.Publisher("/tlgc/correction_trajectory", PoseArray, queue_size=10)
        self.clear_rviz_pub = rospy.Publisher("/tlgc/viz_clear", String, queue_size=2)


        #Init Message
        rospy.sleep(1)
        rospy.loginfo("path_recorder has been started")  

    def joy_cb(self, msg):
        # Filter buttons input
        current_buttons = list(msg.buttons)
        if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
        else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
        self.previous_buttons = current_buttons
        
        if changed_buttons[2] != 0:
            print("CLEAR")
            self.clear = True
            return
        if changed_buttons[0] != 0:
            print("Force load.")
            self.force_load = True
            return
    

    def commanded_pose_cb(self,msg):
        msg.header.frame_id = "LIO_base_link"
        self.path.poses.append(msg)
    
    def events_cb(self, msg):
        
        if msg.data == "Start.":
            self.start = True
            rospy.loginfo("*Start")
        if msg.data == "Terminate.":
            self.start = False
            self.stop = True
            rospy.loginfo("*Stop")
    
    def lio_pose_cb(self, msg):
        self.path.poses.append(msg)
        if self.start:
            if self.save_correction_traj:
                self.save_correction_traj = False
                self.correction_trajectories.append(self.current_trajectory)
                rospy.loginfo(f"Saved correction trajectory of {len(self.current_trajectory)} points")
                self.current_trajectory = []
            elif self.save_autonomous_traj:
                self.save_autonomous_traj = False
                self.autonomous_trajectories.append(self.current_trajectory)
                rospy.loginfo(f"Saved autonomous trajectory of {len(self.current_trajectory)} points")
                self.current_trajectory = []

            self.current_trajectory.append(msg)
    
    def surface_pose_alternative_cb(self, msg):

        if self.state == None:
            self.state = msg.state
            if self.state == "":
                rospy.loginfo("Recording as autonomous state ...")
            elif self.state == "correction":
                rospy.loginfo("Recording as correction state ...")
            
        if msg.state == "correction" and self.state == "":
            self.autonomous_trajectories.append(self.current_trajectory)
            rospy.loginfo(f"Saved autonomous trajectory of {len(self.current_trajectory)} points")
            self.current_trajectory = []
            self.state = msg.state
            rospy.loginfo("Recording as correction state ...")
        elif msg.state == "" and self.state == "correction":
            self.correction_trajectories.append(self.current_trajectory)
            rospy.loginfo(f"Saved correction trajectory of {len(self.current_trajectory)} points")
            self.current_trajectory = []
            self.state = msg.state
            rospy.loginfo("Recording as autonomous state ...")


        self.current_trajectory.append(msg.pose_global)
        
    
    def surface_pose_cb(self, msg):
        
        if self.state == None:
            self.state = msg.state
            if self.state == "":
                rospy.loginfo("Recording as autonomous state ...")
            elif self.state == "correction":
                rospy.loginfo("Recording as correction state ...")
            return

        if msg.state == "correction" and self.state == "":
            self.save_autonomous_traj = True
            while self.save_autonomous_traj:
                pass
            self.state = msg.state
            rospy.loginfo("Recording as correction state ...")
        elif msg.state == "" and self.state == "correction":
            self.save_correction_traj = True
            while self.save_correction_traj:
                pass
            self.state = msg.state
            rospy.loginfo("Recording as autonomous state ...")

    
    def visualise_trajectory(self, trajectories, type, pose_type):

        nb_trajectories = len(trajectories)

        if pose_type == "real":
            for i in range(nb_trajectories):
                
                trajectory = trajectories[i]
                nb_points = len(trajectory)
                msg = PoseArray()
                msg.header.frame_id = "LIO_robot_base_link"
                for j in range(nb_points):
                    pose = Pose()
                    pose.position.x = trajectory[j].pose.position.x
                    pose.position.y = trajectory[j].pose.position.y
                    pose.position.z = trajectory[j].pose.position.z

                    msg.poses.append(pose)
                
                if type == "correction":
                    self.corrected_traj_pub.publish(msg)
                elif type == "autonomous": 
                    self.autonomous_traj_pub.publish(msg)
                else :
                    rospy.logerr("Type of trajectory not detected.")
                
        elif pose_type == "commanded":
            for i in range(nb_trajectories):
                
                trajectory = trajectories[i]
                nb_points = len(trajectory)
                msg = PoseArray()
                msg.header.frame_id = "LIO_base_link"
                for j in range(nb_points):
                    pose = Pose()
                    pose.position.x = trajectory[j].position.x
                    pose.position.y = trajectory[j].position.y
                    pose.position.z = trajectory[j].position.z

                    msg.poses.append(pose)
                
                if type == "correction":
                    self.corrected_traj_pub.publish(msg)
                elif type == "autonomous": 
                    self.autonomous_traj_pub.publish(msg)
                else :
                    rospy.logerr("Type of trajectory not detected.")

    
    
    
    def run(self):
        self.clear_rviz_pub.publish("trajectory")

        while not rospy.is_shutdown():
            # if self.stop:
            #     rospy.loginfo(f" {len(self.correction_trajectories)} corrected trajectories to publish.")
            #     self.visualise_trajectory(self.correction_trajectories, "correction", self.pose_type)
            #     rospy.loginfo(f" {len(self.autonomous_trajectories)} autonomous trajectories to publish.")
            #     self.visualise_trajectory(self.autonomous_trajectories, "autonomous", self.pose_type)
            #     return

            if self.stop or self.force_load:
                if self.state == "":
                    self.autonomous_trajectories.append(self.current_trajectory)
                    rospy.loginfo(f"Saved autonomous trajectory of {len(self.current_trajectory)} points")
                    self.current_trajectory = []
                elif self.state == "correction" :
                    self.correction_trajectories.append(self.current_trajectory)
                    rospy.loginfo(f"Saved correction trajectory of {len(self.current_trajectory)} points")
                    self.current_trajectory = []
                
                self.visualise_trajectory(self.correction_trajectories, "correction", self.pose_type)
                self.visualise_trajectory(self.autonomous_trajectories, "autonomous", self.pose_type)
                
                if self.stop: 
                    return
                elif self.force_load:
                    self.force_load = False
                    continue

            if self.clear :
                self.path.poses.clear()
                self.correction_trajectories.clear()
                self.autonomous_trajectories.clear()
                self.clear_rviz_pub.publish("trajectory")
                self.clear = False
            
            
            self.visualise_trajectory(self.correction_trajectories, "correction", self.pose_type)
            self.visualise_trajectory(self.autonomous_trajectories, "autonomous", self.pose_type)
            
            self.path_pub.publish(self.path)
            self.rate.sleep()



if __name__ == "__main__":
    my_node = PathRecorder()  
    my_node.run()