import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = os.path.join(
        get_package_share_directory('ur5_description'),
        'urdf', 'ur5_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    controllers_yaml = os.path.join(
        get_package_share_directory('ur5_controller'),
        'config', 'ros2_controllers.yaml')

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}])

    # ── 2. ros2_control node ─────────────────────────────────────────────────
    # On passe le yaml ICI pour que le controller_manager
    # connaisse les types — mais on NE spawn PAS via lui directement
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time},
            controllers_yaml
        ])

    # ── 3. Spawners avec délais croissants ───────────────────────────────────
    # joint_state_broadcaster en premier (immédiat)
    jsb_spawner = TimerAction(period=1.5, actions=[Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'])])

    # ur5_arm_controller après jsb
    arm_spawner = TimerAction(period=3.0, actions=[Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['ur5_arm_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'])])

    # hand_controller en dernier
    hand_spawner = TimerAction(period=4.0, actions=[Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['hand_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'])])

    # ── 4. RViz (après que tout soit actif) ──────────────────────────────────
    rviz_node = TimerAction(period=5.0, actions=[Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('ur5_description'),
            'config', 'view_config.rviz')])])

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
        hand_spawner,
        rviz_node,
    ])
