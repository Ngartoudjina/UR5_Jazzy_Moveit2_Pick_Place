import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ur5_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ur5_controller"),
            "launch",
            "controller.launch.py"
        )
    )


    moveit_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ur5_moveit"),
            "launch",
            "moveit.launch.py"
        )
    )

    
    return LaunchDescription([
        gazebo,
        controller,
        moveit_launch
    ])