# Writing bringup package for your own project
If you want to use the driver in your own project, you are not advised to modify `ndi_description` and `ndi_hardware` packages as the driver is designed to put all customization and configuration in the **bringup** package. It is recommended to take `ndi_bringup` as an example and write your own bringup package. To do so, you need the following steps. 

## 1. Write your URDF file
Write your URDF file with reference to [urdf/polaris_vega_setup.urdf.xacro](https://github.com/zixingjiang/ndi_ros2_driver/blob/jazzy/ndi_bringup/urdf/polaris_vega_setup.urdf.xacro). Include the `ndi_tracker` xacro macro defined in `ndi_description`: [ndi_tracker.ros2_control.xacro](https://github.com/zixingjiang/ndi_ros2_driver/blob/jazzy/ndi_description/urdf/ndi_tracker.ros2_control.xacro) in your URDF file's `ros2_control` tag to load the trackers you are interested in tracking. Name and SROM file of the tracker should be provided as arguments to the macro, and should be distinct for each tracker.

## 2. Write your controller manager configuration file
Write your controller manager configuration file with reference to [config/polaris_vega_controllers.yaml](https://github.com/zixingjiang/ndi_ros2_driver/blob/jazzy/ndi_bringup/config/polaris_vega_controllers.yaml). You should configure the pose broadcaster for each tracker you defined in your URDF file. Tracker names should be consistent with the names you provided in the URDF file.

## 3. Write your launch file
Write your launch file with reference to [launch/polaris_vega.launch.py](https://github.com/zixingjiang/ndi_ros2_driver/blob/jazzy/ndi_bringup/launch/polaris_vega.launch.py). You should launch the following nodes with the URDF and configuration file you written to bring up the driver.
- `robot_state_publisher/robot_state_publisher` for publishing robot description. You should provide the URDF file you written as an argument.
- `controller_manager/ros2_control_node` for launching the `ros2_control` stack to run the NDI hardware interface and pose broadcasters. You should provide the controller manager configuration file you written as an argument.
- `controller_manager/spawner` for spawning each pose broadcaster you defined in the controller manager configuration file.
- `rviz2/rviz2` to visualize broadcasted tracker poses (if needed).

**Reminder**: If you want to use the NDI device with other devices controlled by `ros2_control`, please be careful to configure the **namespace** of each node and topic.