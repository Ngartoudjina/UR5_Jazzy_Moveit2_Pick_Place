from setuptools import find_packages, setup

package_name = 'ur5_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beingar',
    maintainer_email='beingar@example.com',
    description='UR5 Pick and Place with MoveIt2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'add_object        = ur5_pick_place.add_object:main',
            'pose_sender       = ur5_pick_place.pose_sender:main',
            'fake_object_pose  = ur5_pick_place.fake_object_pose:main',
            'joint_state_pub   = ur5_pick_place.joint_state_publisher:main',
        ],
    },
)
