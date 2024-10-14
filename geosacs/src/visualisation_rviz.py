#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Empty, String



class VisualisationRViz(): 
    def __init__(self):
        rospy.init_node("visualisation_rviz_node") 

        # Variables
        self.rate = rospy.Rate(15)
        self.raw_trajectories = []  
        self.autonomous_trajectories = []
        self.correction_trajectories = []
        self.processed_trajectories = []  
        self.reproduced_trajectory = []
        self.gc_circles = []
        self.directrix = []
        self.x_correction_axes = []
        self.y_correction_axes = []
        self.eT_axes = []
        self.eN_axes = []
        self.eB_axes = []

        self.raw_pose_trajectory = PoseArray()
        self.processed_pose_trajectory = PoseArray()
        self.directrix_pose = PoseArray()
        self.reproduced_pose_trajectory = PoseArray()
        self.colors={}
        self.id = 0
        self.frame = "LIO_base_link"
        self.alternative_frame = ""
        
        # ROS Variables
        rospy.Subscriber("/tlgc/raw_trajectory", PoseArray, self.raw_trajectory_cb)
        rospy.Subscriber("/tlgc/processed_trajectory", PoseArray, self.processed_trajectory_cb)
        rospy.Subscriber("/tlgc/directrix", PoseArray, self.directrix_cb)
        rospy.Subscriber("/tlgc/reproduced_trajectory", PoseArray, self.reproduced_trajectory_cb)
        rospy.Subscriber("/tlgc/gc_circle", PoseArray, self.gc_circle_cb)
        rospy.Subscriber("/tlgc/viz_clear", String, self.clear_cb)
        rospy.Subscriber("/tlgc/x_correction_axis",PoseArray, self.x_correction_axes_cb)
        rospy.Subscriber("/tlgc/y_correction_axis",PoseArray, self.y_correction_axes_cb)
        rospy.Subscriber("/tlgc/eT_axis",PoseArray, self.eT_axis_cb)
        rospy.Subscriber("/tlgc/eN_axis",PoseArray, self.eN_axis_cb)
        rospy.Subscriber("/tlgc/eB_axis",PoseArray, self.eB_axis_cb)

        rospy.Subscriber("/tlgc/raw_pose_trajectory",PoseArray, self.raw_pose_trajectory_cb)
        rospy.Subscriber("/tlgc/processed_pose_trajectory",PoseArray, self.processed_pose_trajectory_cb)
        rospy.Subscriber("/tlgc/directrix_pose",PoseArray, self.directrix_pose_cb)
        rospy.Subscriber("/tlgc/reproduced_pose_trajectory",PoseArray, self.reproduced_pose_trajectory_cb)

        rospy.Subscriber("/tlgc/autonomous_trajectory",PoseArray, self.autonomous_trajectory_cb)
        rospy.Subscriber("/tlgc/correction_trajectory",PoseArray, self.correction_trajectory_cb)


        self.raw_traj_pub = rospy.Publisher("/raw_traj", Marker, queue_size=10)
        self.correction_traj_pub = rospy.Publisher("/correction_traj", Marker, queue_size=10)
        self.autonomous_traj_pub = rospy.Publisher("/autonomous_traj", Marker, queue_size=10)
        self.processed_traj_pub = rospy.Publisher("/processed_traj", Marker, queue_size=10) 
        self.reproduced_traj_pub = rospy.Publisher("/reproduced_traj", Marker, queue_size=10) 
        self.directrix_pub = rospy.Publisher("/directrix", Marker, queue_size=10) 
        self.gc_pub = rospy.Publisher("/gc", Marker, queue_size=10) 
        self.x_corr_axes_pub = rospy.Publisher("/corr_axes_x", Marker, queue_size=10) 
        self.y_corr_axes_pub = rospy.Publisher("/corr_axes_y", Marker, queue_size=10) 
        self.eT_axes_pub = rospy.Publisher("/eT_axes", Marker, queue_size=10) 
        self.eN_axes_pub = rospy.Publisher("/eN_axes", Marker, queue_size=10) 
        self.eB_axes_pub = rospy.Publisher("/eB_axes", Marker, queue_size=10) 
        self.raw_pose_trajectory_pub = rospy.Publisher("/raw_pose_traj", PoseArray, queue_size=10)
        self.processed_pose_trajectory_pub = rospy.Publisher("/processed_pose_traj", PoseArray, queue_size=10)
        self.directrix_pose_pub = rospy.Publisher("/directrix_pose", PoseArray, queue_size=10)
        self.reproduced_pose_trajectory_pub = rospy.Publisher("/reproduced_pose_traj", PoseArray, queue_size=10)


        self.init_color_dict()

        #Init Message
        rospy.loginfo("visualisation_rviz_node has been started") 

    def eN_axis_cb(self, msg):
        rospy.loginfo(" eN correction axis received for display.")
        if len(msg.poses) != 2 : rospy.logerr("Need only 2 values to define an axis!")

        arrow = []
        for pose in msg.poses:
            arrow.append(pose.position)
        
        self.eN_axes.append(arrow)
    
    def eB_axis_cb(self, msg):
        rospy.loginfo(" eB correction axis received for display.")
        if len(msg.poses) != 2 : rospy.logerr("Need only 2 values to define an axis!")

        arrow = []
        for pose in msg.poses:
            arrow.append(pose.position)
        
        self.eB_axes.append(arrow)

    def eT_axis_cb(self, msg):
        rospy.loginfo(" eT correction axis received for display.")
        if len(msg.poses) != 2 : rospy.logerr("Need only 2 values to define an axis!")

        arrow = []
        for pose in msg.poses:
            arrow.append(pose.position)
        
        self.eT_axes.append(arrow)
    
    
    def reproduced_pose_trajectory_cb(self, msg):
        rospy.loginfo("Reproduced pose trajectory received for display")
        for pose in msg.poses:
            self.reproduced_pose_trajectory.poses.append(pose)
    
    
    def directrix_pose_cb(self, msg):
        rospy.loginfo("Directrix pose trajectory received for display")
        for pose in msg.poses:
            self.directrix_pose.poses.append(pose)

    
    
    def raw_pose_trajectory_cb(self, msg):
        rospy.loginfo("Raw pose trajectory received for display")
        for pose in msg.poses:
            self.raw_pose_trajectory.poses.append(pose)
        
    
    def processed_pose_trajectory_cb(self, msg):
        rospy.loginfo("Processed pose trajectory received for display")
        for pose in msg.poses:
            self.processed_pose_trajectory.poses.append(pose)
    
    
    def init_color_dict(self):
        green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        blue = ColorRGBA(0.0, 0.0, 1.5, 0.8)
        yellow = ColorRGBA(1.0, 1.0, 0.0, 1)
        gray = ColorRGBA(0.5, 0.5, 0.5, 0.3)
        black = ColorRGBA(0, 0, 0, 0.3)
        dark_green = ColorRGBA(0.1, 0.6, 0.32, 1)
        purple = ColorRGBA(0.16, 0.032, 0.24, 1)
        self.colors = {"red":red, "green":green, "blue":blue, "yellow":yellow, "gray":gray, "black":black, "dark green":dark_green,
                       "purple":purple}
    
    def x_correction_axes_cb(self, msg):

        rospy.loginfo(" X correction axis received for display.")
        if len(msg.poses) != 2 : rospy.logerr("Need only 2 values to define an axis!")

        arrow = []
        for pose in msg.poses:
            arrow.append(pose.position)
        
        self.x_correction_axes.append(arrow)
    
    def y_correction_axes_cb(self, msg):

        rospy.loginfo("Y correction axes received for display.")
        if len(msg.poses) != 2 : rospy.logerr("Need only 2 values to define an axis!")

        arrow = []
        for pose in msg.poses:
            arrow.append(pose.position)
        
        self.y_correction_axes.append(arrow)
    
    def clear_cb(self, msg):
        cmd = msg.data
        delete_marker = Marker()
        delete_marker.action = 3

        if cmd == "raw" : 
            self.raw_trajectories.clear()
            self.raw_traj_pub.publish(delete_marker)
        elif cmd == "processed" : 
            self.processed_trajectories.clear()
            self.processed_traj_pub.publish(delete_marker)
        elif cmd == "directrix" : 
            self.directrix.clear()
            self.directrix_pub.publish(delete_marker)
        elif cmd == "reproduced" : 
            self.reproduced_trajectory.clear()
            self.reproduced_traj_pub.publish(delete_marker)
            self.reproduced_pose_trajectory.poses.clear()
        elif cmd == "gc" : 
            self.gc_circles.clear()
            self.gc_pub.publish(delete_marker)
        elif cmd == "trajectory" :
            rospy.loginfo("clear trajectory")
            self.correction_trajectories.clear()
            self.autonomous_trajectories.clear()
            self.correction_traj_pub.publish(delete_marker)
            self.autonomous_traj_pub.publish(delete_marker)
        elif cmd == "all" :
            self.raw_trajectories.clear()
            self.processed_trajectories.clear()
            self.directrix.clear()
            self.reproduced_trajectory.clear()
            self.gc_circles.clear()
            self.x_correction_axes.clear()
            self.y_correction_axes.clear()
            self.eT_axes.clear()
            self.eB_axes.clear()
            self.eN_axes.clear()
            self.raw_pose_trajectory.poses.clear()
            self.processed_pose_trajectory.poses.clear()
            self.directrix_pose.poses.clear()
            self.reproduced_pose_trajectory.poses.clear()
            
            self.raw_traj_pub.publish(delete_marker)
            self.processed_traj_pub.publish(delete_marker)
            self.directrix_pub.publish(delete_marker)
            self.reproduced_traj_pub.publish(delete_marker)
            self.gc_pub.publish(delete_marker)
            self.x_corr_axes_pub.publish(delete_marker)
            self.y_corr_axes_pub.publish(delete_marker)
            self.eT_axes_pub.publish(delete_marker)
            self.eB_axes_pub.publish(delete_marker)
            self.eN_axes_pub.publish(delete_marker)
            rospy.loginfo("***CLEAR***")
            
        else:
            rospy.logwarn("Clear command not understood.")

    def gc_circle_cb(self, msg):
        rospy.loginfo("gc circle trajectory received for display.")
        traj = []
        for pose in msg.poses:
            traj.append(pose.position)
        self.gc_circles.append(traj)

    def reproduced_trajectory_cb(self, msg):
        rospy.loginfo("Reproduced trajectory received for display.")
        traj = []
        for pose in msg.poses:
            traj.append(pose.position)
        self.reproduced_trajectory.append(traj)
    
    def directrix_cb(self, msg):
        rospy.loginfo("Directrix trajectory received for display.")
        traj = []
        for pose in msg.poses:
            traj.append(pose.position)
        self.directrix.append(traj)

    def processed_trajectory_cb(self,msg):
        rospy.loginfo("Processed trajectory received for display.")
        traj = []
        for pose in msg.poses:
            traj.append(pose.position)
        self.processed_trajectories.append(traj)

    def raw_trajectory_cb(self, msg):
        rospy.loginfo("Raw trajectory received for display.")
        traj = []
        for pose in msg.poses:
            traj.append(pose.position)
        self.raw_trajectories.append(traj)

    def autonomous_trajectory_cb(self, msg):
        rospy.loginfo("Autonomous trajectory received for display.")
        traj = []
        self.alternative_frame = msg.header.frame_id
        for pose in msg.poses:
            traj.append(pose.position)
        self.autonomous_trajectories.append(traj)

    def correction_trajectory_cb(self, msg):
        rospy.loginfo("Correction trajectory received for display.")
        traj = []
        self.alternative_frame = msg.header.frame_id
        for pose in msg.poses:
            traj.append(pose.position)
        self.correction_trajectories.append(traj)
    
    def get_points_markers(self, trajectories, color, size):
        points_msgs = []

        for traj in trajectories:
            msg = Marker()
            msg = self.points(traj, self.colors[color], self.id, size)
            points_msgs.append(msg)
            self.id+= 1

        return points_msgs

    def publish_markers(self, msgs, type):
        for msg in msgs:
            msg.header.stamp = rospy.Time.now()
            # print(msg)
            if type == "raw": self.raw_traj_pub.publish(msg)
            elif type == "processed": self.processed_traj_pub.publish(msg)
            elif type == "directrix": self.directrix_pub.publish(msg)
            elif type == "reproduced": self.reproduced_traj_pub.publish(msg)
            elif type == "gc": self.gc_pub.publish(msg)
            elif type == "x_correction_axes": self.x_corr_axes_pub.publish(msg)
            elif type == "y_correction_axes": self.y_corr_axes_pub.publish(msg)
            elif type == "eT_axes" : self.eT_axes_pub.publish(msg)
            elif type == "eB_axes" : self.eB_axes_pub.publish(msg)
            elif type == "eN_axes" : self.eN_axes_pub.publish(msg)
            elif type == "correction" : self.correction_traj_pub.publish(msg)
            elif type == "autonomous" : self.autonomous_traj_pub.publish(msg)
            else: rospy.logerr("Type of trajectory not detected.")

    def points(self, points, color, id, size):
        # Prepare parameters
        action = 0
        orientation = Quaternion(0,0,0,1)
        scale = Vector3(size, size, 0)
        lifetime=rospy.Duration(1000)

        # Create Message
        msg = Marker()
        msg.header.frame_id = self.frame
        msg.action = action
        msg.id = id
        msg.pose.orientation = orientation
        msg.color = color
        msg.scale = scale
        msg.lifetime = lifetime
        msg.type = Marker.POINTS
        msg.points = points

        return msg
    
    def get_line_markers(self, trajectories, color, width, frame):
        lines_msgs = []

        for traj in trajectories:
            msg = Marker()
            msg = self.line(traj, self.colors[color], self.id, width, frame)
            lines_msgs.append(msg)
            self.id+= 1

        return lines_msgs

    def line(self, points, color, id, width, frame):

        # Prepare parameters
        action = 0
        orientation = Quaternion(0,0,0,1)
        color = color
        scale = Vector3(width, 0, 0)
        lifetime=rospy.Duration(1000)
        # Create Message
        msg = Marker()
        msg.header.frame_id = frame
        msg.action = action
        msg.id = id
        msg.pose.orientation = orientation
        msg.color = color
        msg.scale = scale
        msg.lifetime = lifetime
        msg.type = Marker.LINE_STRIP
        msg.points = points

        return msg

    def arrow(self, arrow, color, id):

        # Create Message
        msg = Marker()
        msg.header.frame_id = self.frame
        msg.action = 0
        msg.id = id
        msg.color = color
        msg.pose.orientation = Quaternion(0,0,0,1)
        msg.scale = Vector3(0.002, 0.005, 0)
        msg.lifetime = rospy.Duration(1000)
        msg.type = Marker.ARROW
        msg.points.append(arrow[0])
        msg.points.append(arrow[1])

        return msg

    def get_arrow_marker(self, arrows, color):

        arrows_msgs = []

        for arrow in arrows:
            msg = Marker()
            msg = self.arrow(arrow, self.colors[color], self.id)
            arrows_msgs.append(msg)
            self.id+= 1

        return arrows_msgs


    def run(self):

        msg = String()
        msg.data = "all"    
        self.clear_cb(msg)
        
        while not rospy.is_shutdown():
            self.id = 0
            # msg_list1 = self.get_points_markers(self.raw_trajectories, "red")
            msg_list0 = self.get_line_markers(self.correction_trajectories, "dark green", width=0.002, frame=self.alternative_frame) #"LIO_robot_base_link" if real tcp
            msg_list00 = self.get_line_markers(self.autonomous_trajectories, "blue", width=0.002, frame=self.alternative_frame) #"LIO_robot_base_link" if real tcp
            msg_list1 = self.get_line_markers(self.raw_trajectories, "red", width=0.002, frame=self.frame)
            msg_list2 = self.get_points_markers(self.processed_trajectories, "green", size=0.005)
            # msg_list3 = self.get_points_markers(self.directrix, "blue", size=0.005)
            msg_list3 = self.get_line_markers(self.directrix, "purple", width=0.001, frame=self.frame)
            msg_list4 = self.get_points_markers(self.reproduced_trajectory, "yellow", size=0.005)
            msg_list5 = self.get_line_markers(self.gc_circles, "gray", width=0.005, frame=self.frame)
            msg_list6 = self.get_arrow_marker(self.x_correction_axes, "red")
            msg_list7 = self.get_arrow_marker(self.y_correction_axes, "dark green")
            msg_list12 = self.get_arrow_marker(self.eT_axes, "yellow")
            msg_list13 = self.get_arrow_marker(self.eN_axes, "green")
            msg_list14 = self.get_arrow_marker(self.eB_axes, "blue")

        
            self.publish_markers(msg_list0, "correction")
            self.publish_markers(msg_list00, "autonomous")
            self.publish_markers(msg_list1, "raw")
            self.publish_markers(msg_list2, "processed")
            self.publish_markers(msg_list3, "directrix")
            self.publish_markers(msg_list4, "reproduced")
            self.publish_markers(msg_list5, "gc")
            self.publish_markers(msg_list6, "x_correction_axes")
            self.publish_markers(msg_list7, "y_correction_axes")
            self.publish_markers(msg_list12, "eT_axes")
            self.publish_markers(msg_list13, "eN_axes")
            self.publish_markers(msg_list14, "eB_axes")
            
            # if not self.raw_pose_trajectory.poses == []:
            msg_8 = self.raw_pose_trajectory
            msg_8.header.stamp = rospy.Time.now()
            msg_8.header.frame_id = self.frame
            self.raw_pose_trajectory_pub.publish(msg_8)
            
            # if not self.processed_pose_trajectory.poses == []:
            msg_9 = self.processed_pose_trajectory
            msg_9.header.stamp = rospy.Time.now()
            msg_9.header.frame_id = self.frame
            self.processed_pose_trajectory_pub.publish(msg_9)

            # if not self.directrix_pose.poses == []:
            msg_10 = self.directrix_pose
            msg_10.header.stamp = rospy.Time.now()
            msg_10.header.frame_id = self.frame
            self.directrix_pose_pub.publish(msg_10)
        
            # if not self.reproduced_pose_trajectory.poses == []:
            msg_11 = self.reproduced_pose_trajectory
            msg_11.header.stamp = rospy.Time.now()
            msg_11.header.frame_id = self.frame
            self.reproduced_pose_trajectory_pub.publish(msg_11)

            self.rate.sleep()

if __name__ == "__main__":
    my_node = VisualisationRViz()  
    my_node.run()