# Copyright 2020 ros2_control Development Team
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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import launch_ros

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='hoverboard_driver')
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "remap_odometry_tf",
            default_value="true",
            description="Remap odometry TF from the steering controller to the TF tree.",
        )
    )    
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "front_port",
    #         default_value="/dev/ttyV0",
    #         description="The serial port to use for the front hoverboard",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "rear_port",
    #         default_value="/dev/ttyV1",
    #         description="The serial port to use for the rear hoverboard",
    #     )
    # )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")
    # front_port = LaunchConfiguration("front_port")
    # rear_port = LaunchConfiguration("rear_port")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hoverboard_driver"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hoverboard_driver"),
            "config",
            "hoverboard_controllers.yaml",
        ]
    )

    ekf = PathJoinSubstitution(
        [
            FindPackageShare("hoverboard_driver"),
            "config",
            "ekf.yaml",
        ]
    )    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hoverboard_driver"), "config", "interface.rviz"]
    )

    control_node_remapped = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers,
            # {"serial_port": rear_port,}
        ],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/rear_hoverboard_base_controller/cmd_vel_unstamped", "/cmd_vel"),  # Rear에 맞게 토픽 매핑
            ("/front_hoverboard_base_controller/cmd_vel_unstamped", "/cmd_vel"),  # Front에 맞게 토픽 매핑
            # ("/rear_hoverboard_base_controller/odom", "/wheel/odom"),  # Rear에 맞게 토픽 매핑
            # ("/front_hoverboard_base_controller/odom", "/wheel/odom"),  # Front에 맞게 토픽 매핑
        ],
        condition=IfCondition(remap_odometry_tf),

    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(remap_odometry_tf),
    )

    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[ekf]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    lidar_node = Node(
        package="lds01",
        executable="lds01",
        output="screen",
        # parameters=[],
        # remappings=[
        #     ("/example_input", "/remapped_input"),  # 토픽 매핑 (필요 시)
        #     ("/example_output", "/remapped_output"),
        # ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
       package="rviz2",
       executable="rviz2",
       name="rviz2",
       output="log",
       arguments=["-d", rviz_config_file],
       condition=IfCondition(gui),
    )

    front_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["front_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    front_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["front_hoverboard_base_controller", "--controller-manager", "/controller_manager"],
    )

    rear_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rear_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rear_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rear_hoverboard_base_controller", "--controller-manager", "/controller_manager"],
    )
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
       event_handler=OnProcessExit(
           target_action=front_joint_state_broadcaster_spawner,
           on_exit=[rviz_node],
       )
    )

    delay_joint_state_publisher_after_all_nodes = TimerAction(
        period=4.0,  
        actions=[joint_state_publisher]
    )

    # Front spawner
    delay_front_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=front_joint_state_broadcaster_spawner,
            on_exit=[front_robot_controller_spawner],
        )
    )

    # Rear spawner
    delay_rear_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rear_joint_state_broadcaster_spawner,
            on_exit=[rear_robot_controller_spawner],
        )
    )

    nodes = [
        # rear_control_node,
        # front_control_node,
        # joint_state_publisher,
        control_node_remapped,
        control_node,
        robot_state_pub_node,
        front_joint_state_broadcaster_spawner,
        rear_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_front_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rear_controller_spawner_after_joint_state_broadcaster_spawner,
        robot_localization_node,
        # delay_joint_state_publisher_after_all_nodes,
        # lidar_node        
    ]

    return LaunchDescription(declared_arguments + nodes)
