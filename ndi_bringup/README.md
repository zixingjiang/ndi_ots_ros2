# Writing bringup package for your own project
If you want to use the driver in your own project, you are not advised to modify `ndi_description` and `ndi_hardware` packages as the driver is designed to put all customization and configuration in the bringup package. It is recommended to take this package as an example and write your own bringup package. To do so, you need the following steps. 

## 1. Write your URDF file
Write your URDF file with reference to [urdf/polaris_vega_setup.urdf.xacro](https://github.com/zixingjiang/ndi_ots_ros2/blob/jazzy/ndi_bringup/urdf/polaris_vega_setup.urdf.xacro). Include the `ndi_tracker` xacro macro defined in `ndi_description`: [ndi_tracker.ros2_control.xacro](https://github.com/zixingjiang/ndi_ots_ros2/blob/jazzy/ndi_description/urdf/ndi_tracker.ros2_control.xacro) in your URDF file's `ros2_control` tag to load the trackers you are interested in tracking. Name and SROM file of the tracker should be provided as arguments to the macro, and should be distinct for each tracker.

**Example: polaris_vega_setup.urdf.xacro**
```xml
<?xml version="1.0"?>
<robot name="polaris_vega_setup" xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:include filename="$(find ndi_description)/urdf/ndi_tracker.ros2_control.xacro"/>
    
    <xacro:arg name="ip" default="0.0.0.0"/>

    <!-- single link represents the base of the polaris vega optical tracking system -->
    <!-- it cannot be negelected as robot state broadcaster requires at least one "root" link in urdf -->
    <!-- it will be used as the reference frame in tracker pose broadcasting -->
    <link name="polaris_vega_base"/>

    <ros2_control name="polaris_vega" type="sensor">        
        <hardware>
            <plugin>ndi_hardware/NdiTrackerHardwareInterface</plugin>
            <param name="ip">$(arg ip)</param>
        </hardware>

        <!-- BEGIN: add trackers here -->
        <!-- It is advised to load only the markers you are interested in broadcasting -->
        <xacro:ndi_tracker 
            name="tracker_1"
            srom="$(find ndi_description)/sroms/8700339.rom" 
        />
        <xacro:ndi_tracker 
            name="tracker_2"
            srom="$(find ndi_description)/sroms/8700340.rom" 
        />
        <!-- END: add trackers here -->

    </ros2_control>

</robot>
```
## 2. Write your controller manager configuration file
Write your controller manager configuration file with reference to [config/polaris_vega_controllers.yaml](https://github.com/zixingjiang/ndi_ots_ros2/blob/jazzy/ndi_bringup/config/polaris_vega_controllers.yaml). You should configure the pose broadcaster for each tracker you defined in your URDF file. Tracker names should be consistent with the names you provided in the URDF file.

**Example: polaris_vega_controllers.yaml**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    # BEGIN: add broadcaster for each tracker here
    # broadcaster name should be distinct
    tracker_1_pose_broadcaster:
      type: pose_broadcaster/PoseBroadcaster

    tracker_2_pose_broadcaster:
      type: pose_broadcaster/PoseBroadcaster
    # END: add broadcaster for each tracker here
  
# BEGIN: configure broadcaster for each tracker here
# pose_name should be the same as tracker name in URDF
# frame_id should be the same as base link in URDF
tracker_1_pose_broadcaster:
  ros__parameters:
    pose_name: "tracker_1"
    frame_id: "polaris_vega_base"

tracker_2_pose_broadcaster:
  ros__parameters:
    pose_name: "tracker_2"
    frame_id: "polaris_vega_base"
# END: configure broadcaster for each tracker here
```

## 3. Write your launch file
Write your launch file with reference to [launch/polaris_vega.launch.py](https://github.com/zixingjiang/ndi_ots_ros2/blob/jazzy/ndi_bringup/launch/polaris_vega.launch.py). You should launch the following nodes with the URDF and configuration file you written to bring up the driver.
- `robot_state_publisher/robot_state_publisher` for publishing robot description. You should provide the URDF file you written as an argument.
- `controller_manager/ros2_control_node` for launching the `ros2_control` stack to run the NDI hardware interface and pose broadcasters. You should provide the controller manager configuration file you written as an argument.
- `controller_manager/spawner` for spawning each pose broadcaster you defined in the controller manager configuration file.
- `rviz2/rviz2` to visualize broadcasted tracker poses (if needed).
