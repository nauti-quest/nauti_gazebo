#ifndef CMD_VEL_BRIDGE_H
#define CMD_VEL_BRIDGE_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "soft_uwv.h"
#include <thread>
#include "v2_control/LocoCommand.h"

class CommandListener {
private:
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    SoftUWV v2_;
    
    // Scale factors for command inputs
    double surge_scale_;
    double sway_scale_;
    double heave_scale_;
    double yaw_scale_;
    
    bool is_traversing_;

public:
    CommandListener();
    void commandCallback(const v2_control::LocoCommand::ConstPtr& command);
    void run();
};

#endif // CMD_VEL_BRIDGE_H