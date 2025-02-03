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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
import launch_ros
def generate_launch_description():

    diff_launch_dir = os.path.join(get_package_share_directory('hoverboard_driver'), 'launch', 'diffbot.launch.py')
    map_file = os.path.join(get_package_share_directory('hoverboard_driver'), 'map', 'map.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('hoverboard_driver'), 'param', 'hover.yaml')
    lifecycle_nodes = ['map_server', 
                       'amcl',
                    #    'planner_server',
                    #    'controller_server',
                    #    'recoveries_server',
                    #    'bt_navigator'
                       ]

    diff_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diff_launch_dir)
    )


    start_map_server = Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'yaml_filename': map_file}]
                )
    map_server_lifecycle = RegisterEventHandler(
        OnProcessStart(
            target_action=start_map_server,
            on_start=[
                TimerAction(
                    period=1.0,  
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'run', 'nav2_util' ,'lifecycle_bringup','map_server'],
                            output='screen'
                        )
                    ]
                )
            ]
        )
    )

    amcl_node = Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml]
                )
    start_amcl = TimerAction(
        period=5.0,  
        actions=[
            amcl_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=amcl_node,
                    on_start=[
                        TimerAction(
                            period=1.0,  
                            actions=[
                                ExecuteProcess(
                                    cmd=['ros2', 'run', 'nav2_util' ,'lifecycle_bringup','amcl'],
                                    output='screen'
                                )
                            ]
                        )
                    ]
                )
            )
        ]
    )
    # start_amcl =  Node(
    #                 package='nav2_amcl',
    #                 executable='amcl',
    #                 name='amcl',
    #                 output='screen',
    #                 parameters=[nav2_yaml]
    #             )
            

    # start_controller_server = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=start_amcl,
    #         on_exit=[
    #             Node(
    #                 package='nav2_controller',
    #                 executable='controller_server',
    #                 name='controller_server',
    #                 output='screen',
    #                 parameters=[nav2_yaml]
    #             )
    #         ]
    #     )
    # )

    # start_planner_server = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=start_controller_server,
    #         on_exit=[
    #             Node(
    #                 package='nav2_planner',
    #                 executable='planner_server',
    #                 name='planner_server',
    #                 output='screen',
    #                 parameters=[nav2_yaml]
    #             )
    #         ]
    #     )
    # )

    # start_bt_navigator = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=start_planner_server,
    #         on_exit=[
    #             Node(
    #                 package='nav2_bt_navigator',
    #                 executable='bt_navigator',
    #                 name='bt_navigator',
    #                 output='screen',
    #                 parameters=[nav2_yaml]
    #             )
    #         ]
    #     )
    # )

    # start_lifecycle_manager = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=start_amcl,
    #         on_exit=[
    #             Node(
    #                 package='nav2_lifecycle_manager',
    #                 executable='lifecycle_manager_localization',
    #                 name='lifecycle_manager_localization',
    #                 output='screen',
    #                 parameters=[{'use_sim_time': False},
    #                             {'autostart': True},
    #                             {'node_names': lifecycle_nodes}
    #                             ]
    #             )
    #         ]
    #     )
    # )

    return LaunchDescription([
        diff_launch,
        start_map_server,
        map_server_lifecycle,
        start_amcl,
        # start_controller_server,
        # start_planner_server,
        # start_bt_navigator,
        # start_lifecycle_manager,
    ])
