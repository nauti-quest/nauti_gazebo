#include "joy_control.h"
#include <sensor_msgs/Joy.h>

class ROVJoyController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    
    // Configurable joystick mappings
    int axis_surge_;
    int axis_sway_;
    int axis_heave_;
    int axis_yaw_;
    int button_traverse_;
    int button_stop_;
    int button_pid_reload_;
    
    // Scale factors for joystick inputs
    double surge_scale_;
    double sway_scale_;
    double heave_scale_;
    double yaw_scale_;

    double surge_velocity_;
    double sway_velocity_;
    double heave_velocity_;
    double yaw_velocity_;
    
    bool is_traversing_;

public:
    ROVJoyController() : nh_("~") {
        // Load joystick mapping parameters
        nh_.param("axis_surge", axis_surge_, 1);     // Left stick vertical
        nh_.param("axis_sway", axis_sway_, 0);       // Left stick horizontal
        nh_.param("axis_heave", axis_heave_, 2);     // Right stick vertical
        nh_.param("axis_yaw", axis_yaw_, 3);         // Right stick horizontal
        nh_.param("button_traverse", button_traverse_, 4);  // LB button
        nh_.param("button_stop", button_stop_, 5);         // RB button
        nh_.param("button_pid_reload", button_pid_reload_, 6); // Back button
        
        // Load scale factors
        nh_.param("surge_scale", surge_scale_, 1.0);
        nh_.param("sway_scale", sway_scale_, 1.0);
        nh_.param("heave_scale", heave_scale_, 1.0);
        nh_.param("yaw_scale", yaw_scale_, 1.0);

        
        // Initialize the UWV
        v2_.initUWV(nh_, 40.0, 0.4);
        
        // Subscribe to joystick inputs
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, 
            &ROVJoyController::joyCallback, this);
            
        // Initial values (same as original)
        v2_.set_xyz_[0] = 0;
        v2_.set_xyz_[1] = 0;
        v2_.set_xyz_[2] = 0.7;
        v2_.set_orient_.yaw = 0;
        is_traversing_ = false;

        surge_velocity_ = 0.0;
        sway_velocity_ = 0.0;
        heave_velocity_ = 0.0;
        yaw_velocity_ = 0.0;
        
        ROS_INFO_STREAM("ROV Joystick Controller Initialized");
        ROS_INFO_STREAM("Initial depth value: " << v2_.set_xyz_[2] 
            << " | Initial heading: " << v2_.set_orient_.yaw);
    }
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        // Handle traverse mode toggle
        if (joy->buttons[button_traverse_]) {
            is_traversing_ = !is_traversing_;
            ROS_INFO_STREAM(is_traversing_ ? 
                "Entering traversing mode..." : 
                "Exiting traversing mode.");
        }
        
        // Handle emergency stop
        if (joy->buttons[button_stop_]) {
            v2_.allStop();
            v2_.set_xyz_[0] = v2_.set_xyz_[1] = 0;
            ROS_INFO_STREAM("Emergency Stop - All thrusters zeroed");
            return;
        }
        
        // Handle PID reload
        if (joy->buttons[button_pid_reload_]) {
            bool was_traversing = is_traversing_;
            is_traversing_ = false;
            v2_.reconfigPid(true, true);
            is_traversing_ = was_traversing;
            ROS_INFO_STREAM("PID configuration reloaded");
        }
        
        surge_velocity_ = joy->axes[axis_surge_] * surge_scale_;
        sway_velocity_ = -joy->axes[axis_sway_] * sway_scale_;   // Negative for correct direction
        heave_velocity_ = -joy->axes[axis_heave_] * heave_scale_;
        yaw_velocity_ = -joy->axes[axis_yaw_] * yaw_scale_;
        
        // Update positions based on velocities
        v2_.set_xyz_[0] = surge_velocity_;  // Direct velocity control
        v2_.set_xyz_[1] = sway_velocity_;
        v2_.set_xyz_[2] = heave_velocity_;  // Integrate for depth (0.02s = 50Hz)
        v2_.set_orient_.yaw = yaw_velocity_;  // Integrate for heading
        
        // Keep the same boundary checks
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
        
        ROS_INFO_STREAM("Velocities - Surge: " << surge_velocity_ 
            << " | Sway: " << sway_velocity_ 
            << " | Heave: " << heave_velocity_
            << " | Yaw: " << yaw_velocity_);
        // Ensure heave doesn't go negative (same as original)
        if (v2_.set_xyz_[2] < 0) {
            v2_.set_xyz_[2] = 0;
        }
    }
    
    void run() {
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_ctrl");
    ROVJoyController controller;
    controller.run();
    return 0;
}