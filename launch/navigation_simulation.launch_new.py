import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


""" Launch file functions
    - Run navigation node with parameters.
    - Run fake lidar node and fake map node.
    - Run trajectory recorder node.
    - Spawn turtlebot in Gazebo with initial pose arguments.
"""
def stringify_dict_values(d):
    return {k: str(v) for k, v in d.items()}

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Load config
    pkg_semnav = get_package_share_directory('semnav')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    semnav_world_dir = os.path.join(pkg_semnav, 'worlds')

    config_path = os.path.join(pkg_semnav, 'config', 'launch_args.yaml')
    robots_path = os.path.join(pkg_semnav, 'config', 'robots.yaml')

    configs = load_yaml(config_path)
    robot_list = load_yaml(robots_path).get('robots', [])

    planner_config = configs.get('planner_config', {})
    risk_map_config = configs.get('risk_map_config', {})
    recording_config = configs.get('recording_config', {})

    # Launch Gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
		launch_arguments={
            'world': os.path.join(semnav_world_dir, 'slow_empty.sdf'),
            'verbose': 'true'
        }.items()  # world with different physics from default
    )

    # Topic names for planner
    planner_args = [
        DeclareLaunchArgument('pub_twist_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('pub_behaviorID_topic', default_value='/minitaur/command/behaviorId'),
        DeclareLaunchArgument('pub_behaviorMode_topic', default_value='/minitaur/command/behaviorMode'),

        DeclareLaunchArgument('sub_robot_topic', default_value='/odom'),
        DeclareLaunchArgument('sub_laser_topic', default_value='/scan'),
        DeclareLaunchArgument('sub_semantic_topic', default_value='/semantic_map'),

        DeclareLaunchArgument('world_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('laser_frame_id', default_value='laser')
    ]

    # Robot groups
    robot_groups = []

    for robot in robot_list:
        name = robot['name']
        start = robot['start']
        goal = robot['goal']

        # Combine planner params with robot-specific params
        combined_params = dict(planner_config)
        combined_params.update({
            'Goal_x': goal['x'],
            'Goal_y': goal['y'],

            # Topic and frame names
            """
            'pub_twist_topic':  f'/{name}/cmd_vel',
            'sub_robot_topic':  f'/{name}/odom',
            'sub_laser_topic':  f'/{name}/scan',
            """
            'pub_twist_topic': '/cmd_vel',
            'sub_robot_topic': '/odom',
            'sub_laser_topic': '/scan',
            'sub_semantic_topic': '/semantic_map',
            'world_frame_id': 'map',
            'odom_frame_id': 'odom',
            'laser_frame_id': 'laser',

            # Additional numerical params
            'RobotRadius': 1.0,         # Radius of circle containing robot (m)
            'Tolerance': 0.4,
            'ForwardLinCmdLimit': 0.3,
            'BackwardLinCmdLimit': 0.0,
            'AngCmdLimit': 0.7,
            'LinearGain': 0.2,
            'AngularGain': 0.4,
            'DebugFlag': False
        })

        group = GroupAction([
            PushRosNamespace(name),

            # Spawn TurtleBot in Gazebo
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb_launch_dir, 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': str(start['x']),
                    'y_pose': str(start['y']),
                    'z_pose': '0.0',
                }.items()
            ),

            # Robot state publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb_launch_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                }.items()
            ),

            # Navigation/planner node
            Node(
                package='semnav',
                executable='navigation',
                name='navigation_node',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[combined_params]
            ),

            # Fake Lidar for planner
            Node(
                package='semnav',
                executable='fake_lidar_publisher',
                name='fake_lidar_publisher',
                output='screen',
                parameters=[{
                    'pub_lidar_topic': '/fake_lidar_scan',
                }]
            )
        ])

        robot_groups.append(group)

    # Launch Foxglove simulation
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    return LaunchDescription(
        [gzserver_cmd] + robot_groups + [foxglove_bridge_node]
    )
