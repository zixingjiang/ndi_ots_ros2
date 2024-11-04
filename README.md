# NDI Optical Tracking Systems ROS2
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository contains a `ros2_control` driver for [Northern Digital Inc. (NDI)](https://www.ndigital.com/) manufactured optical tracking systems. This driver is a streamlined and updated version of [ICube-Robotics/ndisys_ros2](https://github.com/ICube-Robotics/ndisys_ros2). A comparison of this driver and ICube-Robotics' driver is shown below.

| Feature | This driver | ICube-Robotics' driver |
| --- | --- | --- |
| Compatible ROS2 distro | Jazzy (targets on Ubuntu 24.04) | Humble (targets on Ubuntu 22.04) |
| Data structure of trackers' pose | Uses the [PoseBroadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/pose_broadcaster) from `ros2_controllers` to publish trackers' pose in a standard ROS2 [PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) format | Implements a rigid pose broadcaster to publish trackers' pose in a self-defined format |
|ROS2 TF broadcasting| Supported via [PoseBroadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/pose_broadcaster) | Supported via self-implemented broadcaster |
|Shipped with NDI Combined API (C++)| Yes (v1.9.7) | Yes |  


## Compatible devices
[NDI optical tracking systems](https://www.ndigital.com/optical-navigation-technology/optical-navigation-products/#).

## System requirements
ROS2 Jazzy on Linux (this driver is developed and tested on Ubuntu 24.04 LTS).

## Packages in this repository
- `ndi_bringup`: This package contains the configuration files used to launch the driver (using polaris vega as an example). It serves not only as an entry point for the driver, but also as a reference for users to configure the driver for their own systems.
- `ndi_description`: This package contains the .rom file and `ros2_control` macros for describing the NDI optical tracking systems.
- `ndi_hardware`: This package contains the `ros2_control` hardware interface for NDI optical tracking systems. This package is built upon NDI's Combined API (C++) v1.9.7, which is shipped with this package at [ndi_hardware/external](https://github.com/zixingjiang/ndi_ots_ros2/tree/jazzy/ndi_hardware/external).

## Getting started
1. **Clone this repository in your ros2 workspace**. Taking `~/ndi_ros2_ws` as an example:
    ```bash
    mkdir -p ~/ndi_ros2_ws/src
    cd ~/ndi_ros2_ws/src
    git clone https://github.com/zixingjiang/ndi_ots_ros2.git
    ```
2. **Build dependencies**. Currently, the [PoseBroadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/pose_broadcaster) used by this driver and the new `ros2_control` features it depends on have not yet been synced to the ROS2 Jazzy release (probably in mid-November 2024), so for now we have to build it from source. 
    ```bash
    cd ~/ndi_ros2_ws

    # download the source code (ros2_control and ros2_controllers) for building PoseBroadcaster
    vcs import src < src/ndi_ots_ros2/ndi_ots_ros2.jazzy.repos

    # build ros2_control packages from source
    colcon build --packages-select \
        controller_interface \
        controller_manager \
        controller_manager_msgs \
        hardware_interface \
        hardware_interface_testing \
        joint_limits \
        ros2_control \
        ros2_control_test_assets \
        ros2controlcli \
        rqt_controller_manager \
        transmission_interface 

    # install the ros2_control packages we just built
    # we will use them to build PoseBroadcaster later
    source install/setup.bash
    ```
3. **Build and source the workspace**. Now we can build the driver and the [PoseBroadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/pose_broadcaster).
    ```bash
    colcon build --packages-select \
        ndi_bringup \
        ndi_description \
        ndi_hardware \
        pose_broadcaster

    source install/setup.bash
    ```
4. **Connect to NDI optical tracking system and bringup the driver**. Connect the NDI system with your PC through Ethernet. The `ndi_bringup` package contains the launch file for a polaris vega system with two trackers loaded (tracker_1: 8700339.rom, tracker_2: 8700340.rom). You can start the driver with the following command. Remainder: you should replace `<you_ndi_ip>` with your setup. 
   ```bash
   ros2 launch ndi_bringup polaris_vega.launch.py ip:=<your_ndi_ip> gui:=true
   ```
   By setting `gui:=true` you can start a Rviz visualization of the polaris vega base frame and the tracker frames. 
   
   <img src="ndi_bringup/doc/rviz.gif" width="500">

5. **Access the data**. After starting the driver with the above launch file, you can access the tracking data through the following two methods:
   1. `ros2_control` state interfaces. Taking tracker_1 as an example, its pose can be accessed in
      - `tracker_1/position.x`
      - `tracker_1/position.y`
      - `tracker_1/position.z`
      - `tracker_1/orientation.x`
      - `tracker_1/orientation.y`
      - `tracker_1/orientation.z`
      - `tracker_1/orientation.w`
   2. ROS2 topics. Taking tracker_1 as an example, its pose can be accessed in topic `/tracker_1_pose_broadcaster/pose`.

## Use this driver in your own project
If you want to use this driver in your own project, it is recommended to take `ndi_bringup` as an example and write your own bringup package. You may find [this guide](https://github.com/zixingjiang/ndi_ots_ros2/blob/jazzy/ndi_bringup/README.md) helpful. 

## Acknowledgement
This driver is written with reference to [ICube-Robotics/ndisys_ros2](https://github.com/ICube-Robotics/ndisys_ros2).