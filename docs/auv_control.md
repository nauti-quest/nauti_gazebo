# About auv_control
This package contains files that control the nautiquest AUV vehicle inside the swimming pool or a near real life replica of how Singapore waters would look like. The visualisation is taken care by **auv_visual** package. The package must used in conjunction with **auv_visual**, and **auv_utility**.

---
## Interpackage Navigation
1. [include](../auv_control/include/README.md)
2. [common](../auv_control/src/common/README.md)
3. [rov_ctrl](../auv_control/src/rov_ctrl/README.md)
4. [sauvc_tasks](../auv_control/src/sauvc_tasks/README.md)
---

Types of control options currently present:
1. Keyboard control *(without the gripper)* in 4-DoF:
   * Surge 	*(x - direction)*
   * Sway 	*(y - direction)*
   * Heave 	*(z - direction)*
   * Yaw 	 *(About z axis)* 
2. Joystick control in 4-DoF.
3. Autonomous control with state machine navigation

## Resources referred and used:
* requires **auv_visual**
* requires **auv_utility**
* requires **darknet_ros**

## Commands
To simulate **SAUVC-2020** rounds with keyboard control:
1. You can choose to simulate any one of the following rounds:
  * Qualification Round
```
roslaunch auv_control sauvc.launch round:=1
```
  * Navigation Round
```
roslaunch auv_control sauvc.launch round:=2
```
Other arguments of  **sauvc.launch** :
- `use_rviz`: true | false (default: false)<br>
	If set _true_, a RViz window opens up in which one can see output of the camera.
- `show_task_balls`: true | false (default: false)<br>
	If set _true_, the task balls for Task 4 are placed on the stands. It is set as _false_ by default because, the balls are in continuous contact with the stand and it takes up a lot of processing power from Gazebo resulting in reduction of real time factor. On hiding the balls, the real time factor is 1.
	
2. To send the manual thrust commands
```
rosrun auv_control key-control
```
3. For set-point based control
```
rosrun auv_control rov_ctrl
```
  - More information on the vectored thruster fusion can be found [here](./On-thruster-configuration.md).
  - PID controller for heave and yaw have been implemented for this node. The PID gain values can be changed in the `.yaml` file located in `/path/to/auv_control/config`. The PID gain values are loaded into the parameter server and the gain values will change with `rosparam set` command while the `rov_ctrl` node is running.
  - **Known issue**: Sometimes after executing the *Start/stop traversing* command in the node, the vehicle snaps back to the origin. If it happens, just re-start the whole simulation, everything should be fine. This bug has been eliminated the issue didn't arise for many test runs as per the author's knowledge; but if it occurs, please inform.
  - Video demonstration of this node: [link](https://youtu.be/kQNYnM7btyY)

4. To view raw front and down camera output (uncalibrated)
```
rosrun image_view image_view image:=/auv_v2/f_cam/image_raw
rosrun image_view image_view image:=/auv_v2/d_cam/image_raw
```
5. To view output of all the sensors - IMU, depth sensor, and camera (front and down)
```
roslaunch auv_control auv_sensors.launch
```
Other arguments of  **auv_sensors.launch** :
- `show_frnt`: true | false (default: true)<br>
If _false_, front camera (uncalibrated) output window will not open.
- `show_dwn`: true | false (default: false)<br>
If _false_, down camera (uncalibrated) output window will not open.
- `show_loc`: true | false (default: true)<br>
If _false_, output from location sensors - IMU, depth sensor are not shown.

[Back to Home](./Home.md)