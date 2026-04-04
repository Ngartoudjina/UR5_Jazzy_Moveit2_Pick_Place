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
            'ik_solver         = ur5_pick_place.ik_solver:main',
            'test_gripper = ur5_pick_place.test_gripper:main',
            'test_joints = ur5_pick_place.test_joints:main',
        ],
    },
)
