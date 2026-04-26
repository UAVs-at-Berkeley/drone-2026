from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('uav_mission'),
            'missions',
            'example_mission.yaml'
        ])
    )

    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    run_take_off_server = Node(
        package='uav_mission',
        executable='offboard_takeoff_server',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    run_central_command_node = Node(
        package='uav_mission',
        executable='central_command_node',
        output='screen',
        parameters=[
            {'mission_file': LaunchConfiguration('mission_file')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    run_time_trial_node = Node(
        package='uav_mission',
        executable='time_trial_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    run_RTH_node = Node(
        package='uav_mission',
        executable='return_to_home_server',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_mission_file_arg,
        declare_use_sim_time_arg,
        run_take_off_server,
        run_central_command_node,
        run_time_trial_node,
        run_RTH_node,
    ])