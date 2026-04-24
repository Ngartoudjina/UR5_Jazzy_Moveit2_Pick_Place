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

    urdf_path = os.path.join(
        get_package_share_directory("ur5_description"),
        "urdf", "ur5_robot.urdf.xacro"
    )
    ur5_moveit_pkg   = get_package_share_directory("ur5_moveit")
    controllers_yaml = os.path.join(ur5_moveit_pkg, "config", "ros2_controllers.yaml")

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )

    # ── ros2_control (mock hardware) ─────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            controllers_yaml,
        ],
        output="screen",
    )

    # ── Spawners contrôleurs ─────────────────────────────────────────────────
    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["ur5_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    hand_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ── MoveIt2 config ───────────────────────────────────────────────────────
    ompl_planning = load_yaml("ur5_moveit", "config/ompl_planning.yaml")

    moveit_config = (
        # "ur5_robot" = nom dans <robot name="ur5_robot"> du URDF et du SRDF.
        # MoveItConfigsBuilder utilise ce nom pour résoudre robot_description.
        MoveItConfigsBuilder("ur5_robot", package_name="ur5_moveit")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/ur5_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── move_group node ───────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ompl_planning,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            # allowed_execution_duration_scaling : facteur multiplicatif sur la durée
            # de trajectoire prévue. 10.0 = permet 10x la durée théorique avant abort.
            # Nécessaire avec mock_components dont le timing n'est pas garanti.
            {"trajectory_execution.allowed_execution_duration_scaling": 10.0},
            {"trajectory_execution.allowed_goal_duration_margin": 5.0},
            # execution_duration_monitoring: False désactive le monitoring de durée
            # au niveau move_group (le contrôleur gère ses propres contraintes via
            # goal_time dans ros2_controllers.yaml).
            {"trajectory_execution.execution_duration_monitoring": False},
            {"plan_execution.record_trajectory_state_frequency": 10.0},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ur5_moveit_pkg, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ── Séquence de démarrage ─────────────────────────────────────────────────
    # Ordre obligatoire :
    #   t=0s  : ros2_control_node démarre, charge le URDF et les contrôleurs
    #   t=3s  : spawners activent les contrôleurs (nécessite ros2_control actif)
    #   t=5s  : move_group démarre APRÈS que joint_state_broadcaster publie
    #           /joint_states — sinon move_group logue "no robot state received"
    #           et refuse les premiers goals.
    delayed_controllers = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster, arm_controller, hand_controller]
    )
    delayed_move_group = TimerAction(
        period=5.0,
        actions=[move_group_node]
    )

    return LaunchDescription([
        ros2_control_node,
        delayed_controllers,
        delayed_move_group,
        rviz_node,
    ])
