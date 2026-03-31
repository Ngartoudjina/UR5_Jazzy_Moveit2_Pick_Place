import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="false")

    urdf_path = os.path.join(
        get_package_share_directory("ur5_description"),
        "urdf", "ur5_robot.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False
        }]
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}]
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit2", package_name="ur5_moveit")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/ur5_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"trajectory_execution.allowed_goal_duration_margin": 0.5},
            {"trajectory_execution.execution_duration_monitoring": False},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config = os.path.join(
        get_package_share_directory("ur5_moveit"),
        "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ])
