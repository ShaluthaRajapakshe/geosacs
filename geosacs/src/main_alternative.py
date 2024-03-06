#!/usr/bin/env python
import rospy
import os
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy

from randomInitialPoints import randomInitialPoints
from reproduce import reproduce

class MainNode(): 
    def __init__(self):
        rospy.init_node("main_node") 

        # Variables
        self.rate = rospy.Rate(2)  
        self.directrix_pub = rospy.Publisher("/tlgc/directrix", PoseArray, queue_size=10)
        self.reproduced_traj_pub = rospy.Publisher("/tlgc/reproduced_trajectory", PoseArray, queue_size=10)
        self.raw_traj_pub = rospy.Publisher("/tlgc/raw_trajectory", PoseArray, queue_size=10)
        self.processed_traj_pub = rospy.Publisher("/tlgc/processed_trajectory", PoseArray, queue_size=10)
        self.gc_circle_pub = rospy.Publisher("/tlgc/gc_circle", PoseArray, queue_size=10)
        self.reset_viz_pub = rospy.Publisher("/tlgc/viz_reset", Empty, queue_size=2)
        self.clear_rviz_pub = rospy.Publisher("/tlgc/viz_clear", String, queue_size=2)
        self.commanded_pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=10)
        self.ratio = None
        self.eN_corr = None
        self.eB_corr = None
        
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.correction = False


        #Init Message
        rospy.sleep(0.5)
        rospy.loginfo("main_node has been started") 
    
    def joy_cb(self, msg):
        
        if msg.axes[0] != 0 or msg.axes[1] != 0:
            self.eN_corr = msg.axes[1]/50
            self.eB_corr = msg.axes[0]/50
            self.correction = True


        if msg.buttons[0] == 1:
            self.correction = True

    def load_model(self, data_dir, task):

        model_dir = data_dir +f"/{task}/model"
        print(f"Model directory {model_dir}")

        items = os.listdir(model_dir)
        print(f"Files: ", items)

        filename = model_dir + f"/{items[0]}"
        model = np.load(filename)
        print(f"File data: {model.files}")
        print(f"Model loaded with {len(model['directrix'])} directrix points.")

        return model

    def load_raw_demos(self, data_dir, task):

        raw_dir = data_dir + f"/{task}/record-raw"
        print(f"Model directory {raw_dir}")

        items = os.listdir(raw_dir)
        print(f"Files: ", items)

        demos = []
        for file in items:
            demo_file = raw_dir + f"/{file}"
            raw_dict = np.load(demo_file)
            traj = raw_dict["position"]
            demos.append(traj)
    
        return demos

    def load_processed_demos(self, data_dir, task):

        processed_dir = data_dir + f"/{task}/record-processed"
        print(f"Processed directory {processed_dir}")

        items = os.listdir(processed_dir)
        rospy.loginfo(f"Files;; {items}.")

        filename = processed_dir + f"/{items[0]}"
        dict = np.load(filename)
        demos = dict["demos"]

        return demos
    
    def visualise_trajectories(self, trajectories, type):

        for traj in trajectories:      
            msg = PoseArray()
            for point in traj:
                p = Pose()
                p.position.x = point[0]
                p.position.y = point[1]
                p.position.z = point[2]
                msg.poses.append(p)
            
            if type == "processed":
                self.processed_traj_pub.publish(msg)
            elif type == "raw": 
                self.raw_traj_pub.publish(msg)
            elif type == "reproduced":
                self.reproduced_traj_pub.publish(msg)
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
        
        directrix = model["directrix"]
        msg = PoseArray()
        for point in directrix:
            p = Pose()
            p.position.x = point[0]
            p.position.y = point[1]
            p.position.z = point[2]
            msg.poses.append(p)

        self.directrix_pub.publish(msg)
        
    def clear_rviz(self, type):
        msg = String()
        msg.data = type
        self.clear_rviz_pub.publish(msg)

    def run(self):

        # Specify data directory and task
        data_dir = "/home/jurassix16/TLGC_data"
        task = "test1816"

        # Get model & demos
        model = self.load_model(data_dir, task)
        raw_demos = self.load_raw_demos(data_dir, task)
        processed_demos = self.load_processed_demos(data_dir, task)

        # Generate trajectory from random a starting point
        numRepro = 1
        starting = np.ones((1, numRepro), dtype=int) 
        crossSectionType = "circle"

        initPoints, Ratio = randomInitialPoints(model, numRepro, crossSectionType)
        self.ratio = Ratio
        print(f"Starting point {initPoints} with of norm {np.linalg.norm(initPoints[0]-model['directrix'][0])}")
        print(f"Rc[0]={model['Rc'][0]}. Starting ratio={Ratio}")

        repro_trajectories = reproduce(model, numRepro, starting, initPoints, Ratio, crossSectionType)
        trajectory = repro_trajectories[0]
        print(f"First trajectory point {trajectory[0]}")
        # trajectory = np.insert(repro_trajectory[0], 0, initPoints[0], axis=0)

        # Visualise model, demos and reproduced trajectories
        self.clear_rviz("all")
        self.visualise_trajectories(raw_demos, "raw")
        self.visualise_trajectories(processed_demos, "processed")
        self.visualise_trajectories(repro_trajectories, "reproduced")
        self.visualise_model(model)

        # Loop trajectory
        print("START LOOP")

        for i in range(len(trajectory)):

            PcurrG = trajectory[i]

            if self.correction:
                print(f"***********************Correction request at i={i}******************************")
                
                # Truncate model
                eN = model["eN"][i:]
                eB = model["eB"][i:]
                eT = model["eT"][i:]
                directrix = model["directrix"][i:]
                Rc = model["Rc"][i:]
                
                small_model = {"eN":eN, "eB":eB, "eT":eT,
                               "directrix":directrix, "Rc":Rc}
                
                # Integrate correction
                # raw_joystickDisplacement = 0.1 * eN[0] + 0 * eB[0]
                raw_joystickDisplacement = self.eN_corr*eN[0] + self.eB_corr*eB[0]
                print ("raw_joystickDisplacement: ", raw_joystickDisplacement)
                print("Before correction PcurrG: ", PcurrG)
                ## Bring point to origin
                AcurrG = PcurrG - directrix[0]
                print("Directrix[0]: ", directrix[0])
                print("AcurrG: ", AcurrG)
                ## Apply correction
                AcurrG_corr = AcurrG + raw_joystickDisplacement
                print("AcurrG_corr: ", AcurrG_corr, "of norm: ", np.linalg.norm(AcurrG_corr))
                ## Saturate correction and compute ratio
                if np.linalg.norm(AcurrG_corr) > Rc[0] : 
                    AcurrG_corr = Rc[0] * AcurrG_corr / np.linalg.norm(AcurrG_corr)
                    Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
                    print(f"Rc[0]={Rc[0]} and (saturated) ratio={Ratio}")
                else:
                    Ratio = np.sqrt(AcurrG_corr[0]**2 + AcurrG_corr[1]**2 + AcurrG_corr[2]**2) / Rc[0]
                    print(f"Rc[0]={Rc[0]} and (corr): ratio={Ratio}")
                ## Translate back to directrix point
                PcurrG_corr = AcurrG_corr + directrix[0]
                print("After correction PcurrG: ", PcurrG_corr)
                ## Change current point 
                PcurrG = PcurrG_corr
                ## Get the right shape for 'reproduce' function
                PcurrG_corr = np.array([[PcurrG_corr[0], PcurrG_corr[1], PcurrG_corr[2]]])
                Ratio = np.array([Ratio])

                # Create new trajectory for remaining points
                newTrajectory = reproduce(small_model, numRepro, starting, PcurrG_corr, Ratio, crossSectionType)
                # print("NewTrajectory", newTrajectory)
                # print("Tajectory", trajectory)
                trajectory[i:]= newTrajectory[0]
                # print("Tajectory updated", trajectory)
                
                print("New trajectory of length ",len(newTrajectory[0]), " with first point: ",newTrajectory[0][0] )
                

                # Update visualisation
                self.clear_rviz("reproduced")
                self.visualise_trajectories([trajectory], "reproduced")

                self.correction = False

            
            msg = PoseStamped()
            msg.header.frame_id = "LIO_robot_base_link"
            msg.pose.position.x = PcurrG[0]
            msg.pose.position.y = PcurrG[1]
            msg.pose.position.z = PcurrG[2]
            msg.pose.orientation.x= -0.697770234523002
            msg.pose.orientation.y= 0.7157399411033368
            msg.pose.orientation.z= 0.01581835965885365
            msg.pose.orientation.w= -0.024141582814929168
            msg.header.stamp = rospy.Time.now()
            self.commanded_pose_pub.publish(msg)






            self.rate.sleep()
        
        print("END LOOP")



if __name__ == "__main__":
    my_node = MainNode()  
    my_node.run()