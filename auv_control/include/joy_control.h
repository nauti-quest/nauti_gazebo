// ROS libraries
#include <ros/ros.h>

// Control library
#include "soft_uwv.h"

// Misc libraries
#include <thread>
#include <csignal>
#include "terminal_getch.h"

// Instance of the underwater vehicle (UWV)
SoftUWV v2_;

// Vars to read user cmds
bool do_not_quit_ = true;

// Functions to read user input
void displayHelp(void);
void readInput(void);

// Misc functions
void signalHandler(int signum);
