#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <ratio>
#include <chrono>
#include <math.h>
#include <array>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>

#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "dhdc.h"

using namespace std;

class ForceDimensionController{
    public:
        int init_inputDevice();
        void run_controller(ros::NodeHandle n);
        ForceDimensionController();
};

ForceDimensionController::ForceDimensionController(){
}

int ForceDimensionController::init_inputDevice() {
    cout <<"Setting up Force Dimension\n";
    int deviceID = dhdOpen();
    if (deviceID < 0) {
        cout << "Unable to open device" << endl;
        dhdSleep (1.0);
        return -1;
    }
    cout << "Force Dimension Setup Complete" << endl;

    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // Second button press to start control & forces
    bool buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Press Button to start controller" << endl << endl;

    bool buttonPressed=false;
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
    
    buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    return deviceID;
}


void ForceDimensionController::run_controller(ros::NodeHandle n){

    ros::Publisher fd_publisher;
    ros::Publisher fd_button_publisher;

    fd_publisher = n.advertise<geometry_msgs::Vector3>("/fdinput/input", 5);
    fd_button_publisher = n.advertise<std_msgs::Float64>("/fdinput/button", 5);

    array<double, 3> forceDimensionPos = {0,0,0};


    double dmp_fx, dmp_fy, dmp_fz;
    
    while(ros::ok()){
        dhdGetPosition (&forceDimensionPos[0],&forceDimensionPos[1],&forceDimensionPos[2]);

        // backwards if the button is pressed
        bool backwards = (dhdGetButton(0)==1);

        geometry_msgs::Vector3 fd_input;
        fd_input.x = forceDimensionPos[0];
        fd_input.y = forceDimensionPos[1];
        fd_input.z = forceDimensionPos[2];
        fd_publisher.publish(fd_input);
        std_msgs::Float64 button_pressed;
        if(backwards){
            button_pressed.data=1.0;
        }
        else{
            button_pressed.data = 0.0;
        }
        fd_button_publisher.publish(button_pressed);

        usleep(1000);
    }
}

int main(int argc, char **argv) {
    ForceDimensionController* controller = new ForceDimensionController();
    int deviceID = controller -> init_inputDevice();
    if (deviceID==-1) {
        cout << endl << "Failed to initialize input device." << endl;
        return -1;
    }
    ros::init(argc, argv, "FDController");
    ros::NodeHandle n("~");
    controller->run_controller(n);
    return 0;
}
