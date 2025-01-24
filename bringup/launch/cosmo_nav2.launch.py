from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
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
     
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('hoverboard_driver'),
            'map',
            'map.yaml'))
    print(map_dir)
    param_file_name = 'hover.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('hoverboard_driver'),
            'param',
            param_file_name))
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'))
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'))
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'))

    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items()
                ))
    



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
