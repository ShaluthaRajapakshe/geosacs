// File name:  dronepanel.cpp
// Description: implementation related to custom rviz panel for drone viewpoints
// Author: Mike Hagenow
// Date: 2/11/22
// Using information found in the ROS
// custom plugins tutorial: https://github.com/ros-visualization/visualization_tutorials/blob/groovy-devel/rviz_plugin_tutorials/src/teleop_panel.cpp

#include <stdio.h>
#include "dronepanel.h"

#include <QWidget>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFont>
#include <QSizePolicy>
#include <QString>
#include <QScreen>
#include <QGuiApplication>


#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rviz/visualization_manager.h>
#include <rviz/view_controller.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <OGRE/OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>


namespace drone_panel{

    DronePanel::DronePanel(QWidget* parent): rviz::Panel(parent){

        int screenWidth = QGuiApplication::primaryScreen()->geometry().size().width();
        int screenRatio = screenWidth/1920;

        cf = 0; //control frame is originally global robot
        mapping = false;

        // Initialize publishers
        cam_pos_pub = n.advertise<geometry_msgs::Point>("rviz_camera_p", 1);
        quat_pub = n.advertise<geometry_msgs::Quaternion>("rviz_camera_q", 1);
        cf_pub = n.advertise<std_msgs::String>("/control_frame", 1);
        rviz_pub = n.advertise<std_msgs::String>("rvizToggle", 1);
        altview_pub = n.advertise<std_msgs::String>("/view_manager/command", 1);

        static tf2_ros::TransformBroadcaster br;

        battery_sub = n.subscribe("/tello/battery", 1, &DronePanel::batteryCallback, this);

        QWidget *cfBox = new QWidget;
        QHBoxLayout* cfLayout = new QHBoxLayout(cfBox);
        cfBox->setStyleSheet("background-color: #dae3e3; border-radius: 10pt; border-color: #b6b8b8");
        cfBox->setFixedWidth(500*screenRatio);
        QPushButton* toggleControlbutton = new QPushButton("Toggle Control Frame");
        toggleControlbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
        QLabel* curr_cf = new QLabel("Robot Base");
        curr_cf->setAlignment(Qt::AlignCenter);
        curr_cf->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Fixed);
        QFont curr_cf_font = curr_cf -> font();
        curr_cf_font.setPointSize(18);
        curr_cf_font.setBold(true);
        curr_cf->setFont(curr_cf_font);
        cfLayout->addWidget(curr_cf);
        cfLayout->addWidget(toggleControlbutton);

        QWidget *mappingBox = new QWidget;
        QHBoxLayout* mappingLayout = new QHBoxLayout(mappingBox);
        mappingBox->setStyleSheet("background-color: #dae3e3; border-radius: 10pt; border-color: #b6b8b8");
        mappingBox->setFixedWidth(300*screenRatio);
        QPushButton* toggleMappingbutton = new QPushButton("Start Mapping");
        toggleMappingbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
        mappingLayout->addWidget(toggleMappingbutton);

        QWidget *viewBox = new QWidget;
        QHBoxLayout* viewLayout = new QHBoxLayout(viewBox);
        viewBox->setStyleSheet("background-color: #dae3e3; border-radius: 10pt; border-color: #b6b8b8");
        viewBox->setFixedWidth(400*screenRatio);
        QPushButton* altViewbutton = new QPushButton("Go to Alternate");
        altViewbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; padding: 6pt;");
        altViewbutton->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Minimum);
        altViewbutton->setMaximumWidth(300*screenRatio);
        viewLayout->addWidget(altViewbutton);
        QPushButton* altViewNewbutton = new QPushButton("Cycle");
        altViewNewbutton->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
        altViewNewbutton->setMaximumWidth(100*screenRatio);
        altViewNewbutton->setStyleSheet("background-color: #B6E7C1; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; padding: 6pt;");
        viewLayout->addWidget(altViewNewbutton);

        QWidget *batteryBox = new QWidget;
        QHBoxLayout* batteryLayout = new QHBoxLayout(batteryBox);
        batteryBox->setStyleSheet("background-color: #dae3e3; border-radius: 10pt; border-color: #b6b8b8");
        batteryBox->setFixedWidth(500*screenRatio);
        bat = new QLabel("Drone Battery: NA");
        bat->setAlignment(Qt::AlignCenter);
        bat->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Fixed);
        QFont bat_font = curr_cf -> font();
        bat_font.setPointSize(18);
        bat_font.setBold(true);
        bat->setFont(bat_font);
        batteryLayout->addWidget(bat);
        QPushButton* armDronebutton = new QPushButton("Start Drone");
        armDronebutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 5em; padding: 6pt;");
        batteryLayout->addWidget(armDronebutton);

        QHBoxLayout* hlayout = new QHBoxLayout;
        hlayout->addWidget(cfBox);
        hlayout->addWidget(mappingBox);
        hlayout->addWidget(viewBox);
        hlayout->addWidget(batteryBox);
        hlayout->setSpacing(0);
        setLayout(hlayout);

        // Change control frame
        connect(toggleControlbutton, &QPushButton::clicked, [this,toggleControlbutton,curr_cf](){
           if(cf==0){
               this->cf++;
               s_out.data ="drone";
               curr_cf->setText("Drone");
           }
           else if(cf==1){
               this->cf++;
               s_out.data ="panda_gripper";
               curr_cf->setText("Gripper");
           }
            else if(cf==2){
               this->cf++;
               s_out.data ="rvizframe";
               curr_cf->setText("Simulation");
           }
           else{
               this->cf = 0;
               s_out.data ="panda_link0";
               curr_cf->setText("Robot Base");
           }
           cf_pub.publish(s_out);
        });

        // Turn on and off mapping
        connect(toggleMappingbutton, &QPushButton::clicked, [this,toggleMappingbutton](){
           if(!mapping){
            s_out.data = "on";
            toggleMappingbutton->setText("End Mapping");
            toggleMappingbutton->setStyleSheet("background-color: #FF968A; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
            mapping = true;
           }
           else{
               s_out.data = "off";
               toggleMappingbutton->setText("Start Mapping");
               toggleMappingbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
               mapping = false;
           }
           rviz_pub.publish(s_out);
        });

        // Go to current alternate view
        connect(altViewbutton, &QPushButton::clicked, [this,altViewbutton](){
            // altViewbutton->setEnabled(false);
            s_out.data = "go";
            altview_pub.publish(s_out);
        });

        // Get a different alternate view
        connect(altViewNewbutton, &QPushButton::clicked, [this,altViewNewbutton](){
            // altViewbutton->setEnabled(false);
            s_out.data = "new";
            altview_pub.publish(s_out);
        });

        // Start the real drone
        connect(armDronebutton, &QPushButton::clicked, [this,armDronebutton](){
            armDronebutton->setEnabled(false);
            s_out.data = "tello";
            rviz_pub.publish(s_out);
        });
        
        // Timer used to publish the camera orientation from RVIZ for camera-centric controls
        QTimer* output_timer = new QTimer( this );  
        connect(output_timer, &QTimer::timeout, [this](){
            rviz::ViewManager* viewm = vis_manager_->getViewManager();
            rviz::ViewController* vc = viewm->getCurrent();
            Ogre::Camera* camera = vc->getCamera();
            const Ogre::Quaternion quat = camera->getOrientation();
            const Ogre::Vector3 cam_pos = camera->getPosition();
            
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "panda_link0";
            transformStamped.child_frame_id = "rvizcamera";

            transformStamped.transform.translation.x = cam_pos.x;
            transformStamped.transform.translation.y = cam_pos.y;
            transformStamped.transform.translation.z = cam_pos.z;
            transformStamped.transform.rotation.x = quat.x;
            transformStamped.transform.rotation.y = quat.y;
            transformStamped.transform.rotation.z = quat.z;
            transformStamped.transform.rotation.w = quat.w;

            br.sendTransform(transformStamped);
        }); 
        output_timer->start(100);

    }

    void DronePanel::batteryCallback(std_msgs::Int16 data){
        // Update the label with battery
        bat->setText(("Drone Battery: "+std::to_string(data.data)+"%").c_str());
        bat->setStyleSheet(("color: rgb("+std::to_string(2*(100-data.data)+50)+",0,0)").c_str());
    }

} // end namespace

// Make pluginlib aware of the class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(drone_panel::DronePanel,rviz::Panel)
