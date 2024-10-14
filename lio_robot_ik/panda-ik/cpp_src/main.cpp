#include <stdint.h>
#include "PandaIKRust.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64MultiArray.h"
#include "geosacs/WeightedPose.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <kdl/kdl.hpp>
#include <iostream>
#include <chrono>
#include <math.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <fstream>


std::array<double, 6> joint_angles = {-1.6211348095305202, -1.1968796730902458, 1.2744727365039032, 0.02532022537521617, 1.481539492575534, -0.056737866157453104};
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_ik");
    boost::mutex mutex;
    ros::NodeHandle nh("~");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto get_required_param = [&](const std::string k, auto& v) {
        if (!nh.getParam(k, v)) {
            std::string msg = "Could not find required parameter: " + k;
            ROS_ERROR(
                msg.c_str()
            );
            return false;
        }
        return true;
    };

    std::string urdf;
    bool weighted_pose;
    if (!get_required_param("URDF", urdf)) return 0;
    if (!get_required_param("weighted_pose", weighted_pose)) return 0; // Adjust the following variables depending on if you want dynamic weight change or not
    if (!init(urdf.c_str())) return 0;


    std::chrono::system_clock::time_point last_msg_time = std::chrono::system_clock::now();
    std::chrono::milliseconds minimum_msg_delay = std::chrono::milliseconds(0);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("output", 1);

    geometry_msgs::PoseStamped commandedPose = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped nextCommandedPose = geometry_msgs::PoseStamped();
    geometry_msgs::Twist commandedVel = geometry_msgs::Twist();

    bool initialized = false;
    int freq = 100;
    bool start = true;
    bool run = true;
    double q_weight = 1.5;          //will use this constant weight if weighted_pose is false

    // If dynamic weight adjustment is needed (set through the panda ik launch file)
    ros::Subscriber twistSub1 = nh.subscribe<geosacs::WeightedPose>("/weighted_pose", 10,
    [&](const geosacs::WeightedPose::ConstPtr& msg) {
        if (weighted_pose){
        ROS_INFO_ONCE("Weighted pose received");
        nextCommandedPose.pose = msg->pose;
        q_weight = msg->weight;
        start = true;
        }
    }
    );

    // Normal mode
    ros::Subscriber twistSub2 = nh.subscribe<geometry_msgs::PoseStamped>("/commanded_pose", 1,
    [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if(!weighted_pose){
        ROS_INFO_ONCE("Commanded pose received");
        nextCommandedPose.pose = msg->pose;
        start = true;
        }
    }
    );


    ros::Rate loop_rate(freq);    

    while (ros::ok()){
        if(! start || ! run){
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        //robot error, robot out of time, drone error, drone out of time
        commandedPose = nextCommandedPose;
        bool valid_output = true;
        std::string name = "lio_tcp_joint";
        std::array<bool, 4> errors = {0,0,0,0};
        std::array<double, 6> robot_state = joint_angles;
        std::array<double, 3> position = {commandedPose.pose.position.x, commandedPose.pose.position.y, commandedPose.pose.position.z};
        std::array<double, 4> orientation = {commandedPose.pose.orientation.x, commandedPose.pose.orientation.y, commandedPose.pose.orientation.z, commandedPose.pose.orientation.w};
        std::array<double, 3> velocity = {commandedVel.linear.x, commandedVel.linear.y,commandedVel.linear.z};
        // ROS_INFO("The value of weight is: %f", q_weight);
        solve(robot_state.data(), name.c_str(), position.data(), orientation.data(), velocity.data(), errors.data(), q_weight);
        
        if(errors[0]){
            robot_state=joint_angles;
        }
        if(errors[1]){
            ;
        }
        if(errors[2]){
            ;
        }
        if(errors[3]){
            ;
        }

        if(initialized && valid_output){
            for(int ii=0;ii<6;ii++){
                if(fabs(robot_state[ii]-joint_angles[ii]>.1)){
                    valid_output = false;
                }
            }
            if(!valid_output){
               if(fabs(commandedPose.pose.position.x - position[0])<.05 && fabs(commandedPose.pose.position.y - position[1])<.05 && fabs(commandedPose.pose.position.z - position[2])<.05){
                   valid_output = true;
               }
            }
        }
        if(!valid_output)
            robot_state=joint_angles;

        initialized = true;
        auto joint_msg = std_msgs::Float64MultiArray();
        joint_msg.data.assign(robot_state.data(), robot_state.data() + 6);
        joint_angles = robot_state;


        tf2::Quaternion q(0,0,0,1);

        auto now = std::chrono::system_clock::now();

        pub.publish(joint_msg);

        while ((now - last_msg_time) < minimum_msg_delay) {
            now = std::chrono::system_clock::now();
        }
        last_msg_time = now;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}