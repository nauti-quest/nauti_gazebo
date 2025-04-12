#include "cmd_vel_bridge.h"
#include <nav_msgs/Odometry.h>

CommandListener::CommandListener() : nh_("~") {
    // Load scale factors (with default values)
    nh_.param("surge_scale", surge_scale_, 50.0);
    nh_.param("sway_scale", sway_scale_, 1.0);
    nh_.param("heave_scale", heave_scale_, 1.0);
    nh_.param("yaw_scale", yaw_scale_, 1.0);
    
    // Initialize the UWV
    v2_.initUWV(nh_, 40.0, 0.4);
    
    // Subscribe to LoCO command topic
    command_sub_ = nh_.subscribe<auv_control::LocoCommand>("/loco/command", 10, 
        &CommandListener::commandCallback, this);
        
    // Initial values 
    v2_.set_xyz_[0] = 0;
    v2_.set_xyz_[1] = 0;
    v2_.set_xyz_[2] = 0.7;
    v2_.set_orient_.yaw = 180;
    is_traversing_ = false;
    
    ROS_INFO_STREAM("Command Listener Initialized");
    ROS_INFO_STREAM("Initial depth value: " << v2_.set_xyz_[2] 
        << " | Initial heading: " << v2_.set_orient_.yaw);
}

void CommandListener::commandCallback(const auv_control::LocoCommand::ConstPtr& command) {
    // Enable traversing mode
    is_traversing_ = true;
    
    v2_.set_xyz_[0] = command->throttle * surge_scale_;

    double yaw_command = command->yaw * yaw_scale_;
    v2_.set_orient_.yaw += yaw_command;
    

    v2_.set_xyz_[2] -= command->heave * heave_scale_;

    if (v2_.set_xyz_[2] < 0) {
        v2_.set_xyz_[2] = 0;
    }
    
    // Handle yaw wraparound
    if (v2_.set_orient_.yaw >= 180.01) {
        v2_.set_orient_.yaw = -(360 - v2_.set_orient_.yaw);
    }
    else if (v2_.set_orient_.yaw <= -179.9) {
        v2_.set_orient_.yaw = 180;
    }
    
    ROS_INFO_STREAM("Received Command - Throttle: " << command->throttle 
                        << " Yaw: " << command->yaw 
                        << " Heave: " << command->heave);
}

void CommandListener::run() {
    ros::Rate rate(50);  // 50Hz control loop
    std::thread pid_thread;
    
    while (ros::ok()) {
        if (is_traversing_) {
            if (!pid_thread.joinable()) {
                pid_thread = std::thread([&]() {
                    v2_.reuseThread();
                    v2_.run();
                });
            }
        }
        else {
            if (!v2_.stopRequested()) {
                v2_.stop();
            }
            if (pid_thread.joinable()) {
                pid_thread.join();
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    // Cleanup
    if (!v2_.stopRequested()) {
        v2_.stop();
    }
    if (pid_thread.joinable()) {
        pid_thread.join();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_bridge");
    CommandListener controller;
    controller.run();
    return 0;
}