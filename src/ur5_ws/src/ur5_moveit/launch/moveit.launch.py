import os
import yaml
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg, file_path)
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory("ur5_description"),
        "urdf", "ur5_robot.urdf.xacro"
    )

    ur5_moveit_pkg = get_package_share_directory("ur5_moveit")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}]
    )

    # Notre publisher avec position initiale valide
    joint_state_publisher = Node(
        package="ur5_pick_place",
        executable="joint_state_pub",
        output="screen",
    )

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
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config = os.path.join(ur5_moveit_pkg, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ])
