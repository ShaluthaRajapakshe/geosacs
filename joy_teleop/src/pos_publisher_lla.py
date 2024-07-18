#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, PoseStamped
import torch
import numpy as np
import torch.nn as nn
from std_msgs.msg import String
from scipy.spatial.transform import Rotation 



# conditional autoencoder
class CAE(nn.Module):

  def __init__(self):
    super(CAE, self).__init__()

    self.name = "CAE"
    # state-action pair is size 14
    # latent space is size 2
    self.fc1 = nn.Linear(12,30) #should be 12 for Lio
    self.fc2 = nn.Linear(30,30)
    self.fc3 = nn.Linear(30,2)

    # state is size 7, latent space is size 2
    self.fc4 = nn.Linear(8,30)
    self.fc5 = nn.Linear(30,30)
    self.fc6 = nn.Linear(30,6)  #should be 6 for Lio

    self.loss_func = nn.MSELoss()

  def encoder(self, x):
    h1 = torch.tanh(self.fc1(x))
    h2 = torch.tanh(self.fc2(h1))
    return self.fc3(h2)

  def decoder(self, z_with_state):
    h4 = torch.tanh(self.fc4(z_with_state))
    h5 = torch.tanh(self.fc5(h4))
    return self.fc6(h5)

  def forward(self, x):
    s = x[:, 0:6] #should be 6 for Lio
    a_target = x[:, 6:12]  #should be 6, 12 for Lio
    z = self.encoder(x)
    z_with_state = torch.cat((z, s), 1)
    a_decoded = self.decoder(z_with_state)
    loss = self.loss(a_decoded, a_target)
    return loss

  def loss(self, a_decoded, a_target):
    return self.loss_func(a_decoded, a_target)





task = "marshmellow"
# task = "other"




class Model(object):

    def __init__(self):
        global task
        self.model = CAE()
        # model_dict = torch.load('models/CAE_model', map_location='cpu')
        # model_dict = torch.load('../models/CAE_model_lio_updated', map_location='cpu')

        if task == "marshmellow":
            # model_dict = torch.load('/home/shalutha/geosacs_ws/src/geosacs/joy_teleop/models/CAE_model_lio_updatedv7', map_location='cpu')
            model_dict = torch.load('/home/shalutha/geosacs_ws/src/geosacs/joy_teleop/models/CAE_model_lio_laundry_18th', map_location='cpu')
        else:
            model_dict = torch.load('/home/shalutha/geosacs_ws/src/geosacs/joy_teleop/models/CAE_model_lio_laundry', map_location='cpu')
        
        self.model.load_state_dict(model_dict)
        self.model.eval()

    def decoder(self, z, q):
        if abs(z[0])+ abs(z[1]) < 0.01:
             return [0.0] * 6
        z_tensor = torch.FloatTensor(z + q) # 2 inputs from joy + 6 joint states
        a_tensor = self.model.decoder(z_tensor)
        return a_tensor.tolist()
    

class PoseControllerNode():
    def __init__(self):
        global task
        rospy.init_node("joy_controller_lla")

        # Variables
        self.rate = rospy.Rate(12)  # Control frequency
        self.time_step = 1.0 / 12 # Time step based on control frequency
        # self.joint_positions = [0.0] * 6  # Initialize positions for seven joints

        # self.lio_pose = None
        self.correction = False
        self.previous_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


        self.joystick_input = [0.0, 0.0]  # Initialize joystick input
        self.position_scale = 1  # Scale factor for joystick input

        self.physical_robot = rospy.get_param("physical_robot")
        

        self.terminate = False
        self.start = False

        

        # ROS Variables
        self.joint_position_pub = rospy.Publisher("/pos_control/output", Float64MultiArray, queue_size=10)
        self.gripper_state_pub =rospy.Publisher("/gripper_state", String, queue_size=10)
        self.myp_app_pub = rospy.Publisher("/myp_manager/app_control", String, queue_size=2)

        self.task_end_pub =rospy.Publisher("/task_end", String, queue_size=10)

        self.commanded_pose_pub = rospy.Publisher("/commanded_pose", PoseStamped, queue_size=10)


        rospy.Subscriber("/joy", Joy, self.joy_cb)

        if self.physical_robot:
            print("Physical robot active")
            rospy.Subscriber("/lio_1c/joint_states", JointState, self.lio_joint_states_cb)
            rospy.Subscriber("/lio_1c/pose", PoseStamped, self.lio_pose_cb)
        else:
            rospy.Subscriber("ik_interface/joint_states_sim", JointState, self.joint_states_sim_cb)

        rospy.Subscriber("/panda_ik/output", Float64MultiArray, self.ik_cb_end)

            
        # rospy.Subscriber("ik_interface/joint_states_lio", JointState, self.joint_states_lio_cb)
        # rospy.Subscriber("/lio_1c/joint_states", JointState, self.lio_joint_states_cb)
        # self.correction = False

        self.first  = True
        self.task = task # "marshmellow" or "other"
        # self.task = "other" # "marshmellow" or "other"

        self.marsh_selected = False

        self.cumulative_correction_distance = 0
        self.lio_cumulative_correction_distance = 0

        self.cumulative_correction_time = 0

        if self.task == "marshmellow":
            # ### Marshmallow ###

            
            # self.initial_joint_positions = [-1.3858905781607287, 0.7159300780040135, 0.4632712412584416, -1.2268456073293923, 1.6233500351764731, -0.03530500317349961]
            self.initial_joint_positions = [-1.410956, 0.383656, 1.350812, 1.518535, -1.492571, -2.692524]
        else:
            ### Laundry ####
            self.initial_joint_positions = [-1.1657304272784037, 1.0865569625827096, 0.7532967406451435, -0.9674269093442143, 1.6453289999999998, 0.3506041628038431]
        

        # Load the trained model
        self.model = Model()
        
        # Init
        rospy.loginfo("joy_controller for lla has been started")


    def lio_pose_cb(self,msg):
        # print("in pose cb")
        self.lio_pose = msg
        # print("########## Lio pose", self.lio_pose.pose.position)

    def joy_cb(self, msg):
        # Map joystick axes to input
        self.joystick_input[0] = msg.axes[0] * self.position_scale  # Axis 0 controls joint1
        self.joystick_input[1] = msg.axes[1] * self.position_scale  # Axis 1 controls joint2
        # Other joystick inputs can be handled if necessary

       # Filter buttons input
        current_buttons = list(msg.buttons)
        if current_buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] : changed_buttons = current_buttons
        else: changed_buttons = [button1 - button2 for (button1, button2) in zip(current_buttons, self.previous_buttons)]
        self.previous_buttons = current_buttons

        if changed_buttons[0] != 0:
            self.gripper_state_pub.publish("toggle_gripper")
            self.gripper_change_request = True

        if changed_buttons[7] != 0:
            # print("STOP")
            self.terminate = True

        if changed_buttons[6] != 0:
            # print("START")
            self.start = True

        if changed_buttons[2] != 0:
            self.marsh_selected = True
            print("marsh_selected")
            

 

        if self.joystick_input[0] != 0.0 or self.joystick_input[1] != 0.0:
            # print("correction given")
            if not self.correction:
                self.correction_start_time = rospy.Time.now()
            self.correction = True
            return
        else:
            if self.correction and not self.first:
                correction_duration = (rospy.Time.now() - self.correction_start_time).to_sec()
                self.cumulative_correction_time += correction_duration
                self.correction_start_time = None
            self.correction = False

        

    def joint_states_sim_cb(self, msg):
        self.lio_joint_positions = list(msg.position[6:12])


    def lio_joint_states_cb(self, msg):
        if self.physical_robot: 
            rospy.loginfo_once("**** Physical robot active ****")
            # self.joint_positions = list(msg.position[6:12])
            self.lio_joint_positions = list(msg.position)[0:6]
            # print("In here", self.joint_positions)



    def change_frame(self, position, orientation):

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
        positions_b = np.array(position)
        orientations_b = np.array(orientation)

        # Transform data into new frame
        
        positions_a = np.zeros(positions_b.shape)
        orientations_a = np.zeros(orientations_b.shape)

        
        pos_b = np.append(positions_b,1)
        pos_b = np.array([pos_b]).reshape(4,1)
        pos_a = np.dot(T_ab, pos_b)
        pos_a = np.ravel(pos_a)
        pos_a = pos_a[:-1]
        
        # Orientation
        q_b = orientations_b
        R_b = Rotation.from_quat([q_b[1], q_b[2], q_b[3], q_b[0]]).as_matrix()
        R_a = np.dot(R_ab, R_b)
        q_a = Rotation.from_matrix(R_a).as_quat()
        q_a = np.array([q_a[3],q_a[0], q_a[1], q_a[2]])
        orientations_a = q_a

        
        return pos_a, orientations_a   


    def ik_cb_end(self, msg):

        self.ik_js = msg.data

        # if self.marsh_selected:



        #     # self.task_end = False
        # # print("task end by ik_cb_end and the data is", self.ik_js) #these should be in radians
        #     print("at the end received ik_js: ", self.ik_js)
        #     print("####################################")



    def run(self):

        if self.physical_robot: self.myp_app_pub.publish("start")
        # self.task_end_pub.publish("True")
        
        while not rospy.is_shutdown():

            if self.first:

                self.update_joint_positions()
                self.publish_joint_positions()
                # self.rate.sleep()

                if self.physical_robot: 
                    # self.start_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])
                    self.start_position_lio = np.array([[-0.290637, -0.573343, 0.561794]])
                    # self.start_position_lio = np.array([[-0.290637, 0.573343, 0.961794]])  #this is a fake value 
                    rospy.loginfo("##### physical robot ####")

                rospy.loginfo("Press START button on joystick ...")
                while not self.start:
                    self.update_joint_positions()
                    self.publish_joint_positions()
                    rospy.sleep(0.1)

                rospy.loginfo("##### started ####")
                start_task_time = rospy.Time.now()
                
                rospy.sleep(0.1)
                self.first = False


            else:
                
                while self.correction:
                    if self.physical_robot:
                        self.current_position_lio = np.array([[self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z]])
                        lio_correction_distance = np.linalg.norm(self.current_position_lio - self.start_position_lio)
                        self.lio_cumulative_correction_distance += lio_correction_distance

                        # print("current and start", self.current_position_lio, self.start_position_lio )

                        self.start_position_lio = self.current_position_lio

                    correction_duration = (rospy.Time.now() - self.correction_start_time).to_sec()
                    self.cumulative_correction_time += correction_duration
                    self.correction_start_time = rospy.Time.now()
                    rospy.sleep(0.2)

                    self.update_joint_positions()
                    self.publish_joint_positions()
                    self.rate.sleep()


                if self.task == "marshmellow" and self.marsh_selected: 

                    if self.physical_robot:

                        self.task_end_pub.publish("True")

                        rospy.sleep(0.5)


                        # print("Before position vals", self.lio_pose.pose.position)
                        print("Before ik vals", self.lio_joint_positions)

                        pos_vals = [self.lio_pose.pose.position.x, self.lio_pose.pose.position.y, self.lio_pose.pose.position.z - 0.05]
                        ori_vals = [self.lio_pose.pose.orientation.w, self.lio_pose.pose.orientation.x, self.lio_pose.pose.orientation.y, self.lio_pose.pose.orientation.z]

                        pos_vals_new, ori_vals_new = self.change_frame(pos_vals, ori_vals)

                        msg1 = PoseStamped()
                        msg1.header.frame_id = "LIO_base_link"
                        # msg1.pose.position.x = self.lio_pose.pose.position.x
                        msg1.pose.position.x = pos_vals_new[0]
                        msg1.pose.position.y = pos_vals_new[1]
                        msg1.pose.position.z = pos_vals_new[2]

                        msg1.pose.orientation.w = ori_vals_new[0]
                        msg1.pose.orientation.x = ori_vals_new[1]
                        msg1.pose.orientation.y = ori_vals_new[2]
                        msg1.pose.orientation.z = ori_vals_new[3]


                        # msg1.pose.position.y = self.lio_pose.pose.position.y
                        # msg1.pose.position.z = self.lio_pose.pose.position.z - 0.03
                        # msg1.pose.orientation.w=  self.lio_pose.pose.orientation.w
                        # msg1.pose.orientation.x= self.lio_pose.pose.orientation.x
                        # msg1.pose.orientation.y= self.lio_pose.pose.orientation.y
                        # msg1.pose.orientation.z= self.lio_pose.pose.orientation.z
                        msg1.header.stamp = rospy.Time.now()

                        

                        count = 0

                        print("Final position vals", msg1.pose.position)

                        self.commanded_pose_pub.publish(msg1)

                        while count < 50:
                            self.commanded_pose_pub.publish(msg1)
                            count += 1

                            self.joint_positions = list(self.ik_js)
                            self.publish_joint_positions()

                            # self.update_joint_positions()
                            # self.publish_joint_positions()
                            self.rate.sleep()

                        print("ik vales ", self.ik_js)

                        # print("##### After published commanded pose", msg1.pose)

                        self.terminate = True

                else:
                
                    self.update_joint_positions()
                    self.publish_joint_positions()
                    self.rate.sleep()




            if self.terminate:

                rospy.loginfo("###################################################")
                if self.physical_robot : self.myp_app_pub.publish("stop")
                task_duration = (rospy.Time.now() - start_task_time).to_sec()
                rospy.loginfo(f"Total task time: {task_duration} seconds")
                rospy.loginfo(f"Total correction time: {self.cumulative_correction_time} seconds")

                # rospy.loginfo(f"Total sim correction distance: {self.cumulative_correction_distance} meters")
                rospy.loginfo(f"Total Lio correction distance: {self.lio_cumulative_correction_distance} meters")

                rospy.loginfo(f"Correction time as a percentage from total: {(self.cumulative_correction_time/task_duration)*100}%)")


                file_path = '/home/shalutha/geosacs_ws/src/geosacs/geosacs/data/experiment_data_latent.txt'

                # Write the values to the file
                with open(file_path, 'a') as file:
                    file.write(f"Total task time: {task_duration} seconds\n")
                    file.write(f"Total correction time: {self.cumulative_correction_time} seconds\n")
                    file.write(f"Total Lio correction distance: {self.lio_cumulative_correction_distance} meters\n")
                    file.write(f"Correction time as a percentage from total:  ({(self.cumulative_correction_time/task_duration)*100}%)\n")
                    file.write("\n")  # Adding a newline for separation between entries
                





                rospy.sleep(1)
                return
            
            self.rate.sleep()
            



    def update_joint_positions(self):
        

        # rospy.loginfo("joy x and y %s",self.joystick_input )
        # rospy.loginfo("velocities: %s", action_velocities)

        if self.first:
            self.joint_positions = self.initial_joint_positions
            # print("still at start", self.joint_positions)

        else:
            # Get model output in rad/s
            # print("################# Before joint positions", self.lio_joint_positions)
            action_velocities = self.model.decoder(self.joystick_input, self.lio_joint_positions)  # Model output in rad/s
            # print("action velocities", action_velocities)

            # Convert velocity commands to position commands
            for i in range(len(self.joint_positions)):
                self.joint_positions[i] += action_velocities[i] * self.time_step  #can multiply with a scale if needed


    def publish_joint_positions(self):
        # Prepare Float64MultiArray message
        msg = Float64MultiArray()

       
        msg.data = self.joint_positions

        # if self.first:
        #     print("first publishable data", msg.data)

        # print("################# After joint positions", self.joint_positions)

        # Publish the positions
        self.joint_position_pub.publish(msg)
        # rospy.loginfo("Published joint positions: %s", msg.data)

if __name__ == "__main__":
    my_node = PoseControllerNode()
    my_node.run()
