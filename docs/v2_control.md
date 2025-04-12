# About v2_control
This package contains files that control the NautiQuest vehicle inside the world. The visualisation is taken care by **v2_visual** package. The package must used in conjunction with **v2_visual**, and **v2_utility**.

---
## Interpackage Navigation
1. [include](../v2_control/include/README.md)
2. [common](../v2_control/src/common/README.md)
3. [rov_ctrl](../v2_control/src/rov_ctrl/README.md)
4. [sauvc_tasks](../v2_control/src/sauvc_tasks/README.md)
---

Types of control options currently present:
1. Keyboard control *(without the gripper)* in 4-DoF:
   * Surge 	*(x - direction)*
   * Sway 	*(y - direction)*
   * Heave 	*(z - direction)*
   * Yaw 	 *(About z axis)* 
2. Autonomous control (Level 1)

## Resources referred and used:
* requires **v2_visual**
* requires **v2_utility**
* requires **darknet_ros**

## Commands
1. To send manual keyboard controls
```
rosrun v2_control rov_ctrl
```
  - More information on the vectored thruster fusion can be found [here](./On-thruster-configuration.md).
  - PID controller for heave and yaw have been implemented for this node. The PID gain values can be changed in the `.yaml` file located in `/path/to/v2_control/config`. The PID gain values are loaded into the parameter server and the gain values will change with `rosparam set` command while the `rov_ctrl` node is running.
  - **Known issue**: Sometimes after executing the *Start/stop traversing* command in the node, the vehicle snaps back to the origin. If it happens, just re-start the whole simulation, everything should be fine. This bug has been eliminated the issue didn't arise for many test runs as per the author's knowledge; but if it occurs, please inform.
2. To send manual joystick controls:
```bash
roslaunch v2_control joy_teleop.launch
```
3. To view raw front and down camera output (uncalibrated)
```
rosrun image_view image_view image:=/auv_v2/f_cam/image_raw
rosrun image_view image_view image:=/auv_v2/d_cam/image_raw
```
4. To view output of all the sensors - IMU, depth sensor, and camera (front and down)
```
roslaunch v2_control auv_sensors.launch
```
Other arguments of  **auv_sensors.launch** :
- `show_frnt`: true | false (default: true)<br>
If _false_, front camera (uncalibrated) output window will not open.
- `show_dwn`: true | false (default: false)<br>
If _false_, down camera (uncalibrated) output window will not open.
- `show_loc`: true | false (default: true)<br>
If _false_, output from location sensors - IMU, depth sensor are not shown.

[Back to Home](./Home.md)