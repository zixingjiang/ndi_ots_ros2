# Copyright 2024 Zixing Jiang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ip = LaunchConfiguration('ip')
    gui = LaunchConfiguration('gui')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('ndi_bringup'),
                    'urdf',
                    'polaris_vega_setup.urdf.xacro',
                ]
            ),
            ' ip:=', ip,
        ]
    )
    robot_description = \
        {'robot_description': ParameterValue(
            robot_description_content, value_type=str)}

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare('ndi_bringup'),
            'rviz',
            'polaris_vega.rviz',
        ]
    )

    controllers_config = PathJoinSubstitution(
        [
            FindPackageShare('ndi_bringup'),
            'config',
            'polaris_vega_controllers.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ParameterFile(controllers_config, allow_substs=True)],
        output='screen',
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='log',
        condition=IfCondition(gui),
    )

    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
        )

    pose_broadcasters = [
        "tracker_1_pose_broadcaster",
        "tracker_2_pose_broadcaster",
    ]

    pose_broadcasters_spawners = [
        controller_spawner(pose_broadcaster) for pose_broadcaster in pose_broadcasters
    ]

    nodes_to_start = [
        node_robot_state_publisher,
        node_controller_manager,
        node_rviz,
    ] + pose_broadcasters_spawners

    return LaunchDescription([
        DeclareLaunchArgument(
            'ip',
            default_value='0.0.0.0',
            description='IP address of the NDI optical tracking system'),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Launch RViz for visualization'),
    ] + nodes_to_start
    )
