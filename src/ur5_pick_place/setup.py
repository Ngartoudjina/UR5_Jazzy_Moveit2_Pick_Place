from setuptools import setup, find_packages

package_name = 'ur5_pick_place'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files in share directory (accessible by installed package)
        ('share/' + package_name + '/config',
            ['ur5_pick_place/scene_config.yaml', 'config/ur5_pick_place_config.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/pick_place.launch.py',
             'launch/pick_place_v2.launch.py',
             'launch/pick_place_moveit2.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ngartoudjina',
    maintainer_email='todo@todo.com',
    description='UR5 pick and place with Robotiq 85',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_sender       = ur5_pick_place.pose_sender:main',
            'add_object        = ur5_pick_place.add_object:main',
            'fake_object_pose  = ur5_pick_place.fake_object_pose:main',
            'joint_state_pub   = ur5_pick_place.joint_state_publisher:main',
            'pick_place_node   = ur5_pick_place.pick_place_node:main',
            'joint_wave_demo   = ur5_pick_place.joint_wave_demo:main',
            'ik_solver              = ur5_pick_place.ik_solver:main',
            'moveit_cartesian_demo  = ur5_pick_place.moveit_cartesian_demo:main',
            'test_gripper = ur5_pick_place.test_gripper:main',
            'test_joints = ur5_pick_place.test_joints:main',
            'joint_wave_moveit = ur5_pick_place.joint_wave_moveit:main',
            'test_ik_solver = ur5_pick_place.test_ik_solver:main',
            'test_moveit = ur5_pick_place.test_moveit:main',
            'test_moveit_levels = ur5_pick_place.test_moveit_levels:main',
            'test_moveit_full_pipeline = ur5_pick_place.test_moveit_full_pipeline:main',
            'test_moveit_scene = ur5_pick_place.test_moveit_scene:main',
            'hand_arm_controller = ur5_pick_place.hand_arm_controller:main',
            'pick_place_moveit = ur5_pick_place.pick_place_moveit:main',
            'pick_place_moveit2 = ur5_pick_place.pick_place_moveit2:main',
            'pick_place_ros2control = ur5_pick_place.pick_place_ros2control:main',
            'moveit_diagnostics = ur5_pick_place.moveit_system_diagnostics:main',
            'test_scene_initialization = ur5_pick_place.test_scene_initialization:main',
            'test_corrections = ur5_pick_place.test_corrections:main',
            'pick_place_hybrid = ur5_pick_place.pick_place_hybrid:main',
            'pick_place_node_v2 = ur5_pick_place.pick_place_node_v2:main'
        ],
    },
)
