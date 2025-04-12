## An Overview
`nauti-gazebo` repository is a compilation of ROS packages that were designed for AUV simulations in a swimming pool or a recreated environment of the ocean waters of Singapore.

---
## Navigate through the documentation
### About packages
1. [v2_control](./v2_control.md)
    1. [include](../v2_control/include/README.md)
    2. [common](../v2_control/src/common/README.md)
    3. [rov_ctrl](../v2_control/src/rov_ctrl/README.md)
2. [v2_dataset](./v2_dataset.md)
3. [v2_visual](./v2_visual.md)
4. [v2_utility](./v2_utility.md)
5. [Setting up darknet](./Setting-up-darknet.md)

### Other Pages
1. [On thruster configuration](./On-thruster-configuration.md)
2. [References](./References.md)

---

## The packages
The following ROS packages are present as a part of this repository:
1. [v2_control](./v2_control.md):
Contains files that control the NautiQuest vehicle inside the swimming pool or the underwater environment.
2. [v2_dataset](./v2_dataset.md):
Contains image dataset from the AUV_v2 vehicle's front and down facing cameras.
3. [v2_visual](./v2_visual.md):
Contains all the files that help in visualising the swimming pool or the underwater environment, and NautiQuest vehicle.
1. [v2_utility](./v2_utility.md):
Contains utility ROS nodes, plugins, and other files required by v2_control.

## A suggestion
For a better workflow and management of various repositories of the AUV Society, create a new catkin workspace (something like `nauti_ws`) and clone various repositories related into it. Keep your personal catkin workspace and AUV workspace separate.

> **Note**: The packages have been developed and tested on Ubuntu 20.04 LTS and ROS Noetic. 