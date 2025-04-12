add_executable(nav_ctrl cmd_vel_bridge.cpp ${UWV_DEPENDENCIES})
target_link_libraries(nav_ctrl ${catkin_LIBRARIES})