import os
import yaml
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path), 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    urdf_path     = os.path.join(
        get_package_share_directory("ur5_description"),
        "urdf", "ur5_robot.urdf.xacro"
    )
    ur5_moveit_pkg = get_package_share_directory("ur5_moveit")
    controllers_yaml = os.path.join(
        get_package_share_directory("ur5_moveit"),
        "config", "ros2_controllers.yaml"
    )

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )

    # ── ros2_control node (mock hardware) ────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    # ── Spawner des contrôleurs ───────────────────────────────────────────────
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ── MoveIt2 ───────────────────────────────────────────────────────────────
    ompl_planning = load_yaml("ur5_moveit", "config/ompl_planning.yaml")

    moveit_config = (
        MoveItConfigsBuilder("moveit2", package_name="ur5_moveit")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/ur5_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ompl_planning,
            {"use_sim_time": True},
            {"publish_robot_description_semantic": True},
            {"trajectory_execution.allowed_execution_duration_scaling": 10.0},
            {"trajectory_execution.allowed_goal_duration_margin": 5.0},
            {"trajectory_execution.execution_duration_monitoring": False},
            {"move_group.trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"plan_execution.record_trajectory_state_frequency": 10.0},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ur5_moveit_pkg, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Délai pour laisser ros2_control démarrer avant les spawners
    delayed_controllers = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster, arm_controller, hand_controller]
    )

    return LaunchDescription([
        ros2_control_node,
        delayed_controllers,
        move_group_node,
        rviz_node,
    ])
