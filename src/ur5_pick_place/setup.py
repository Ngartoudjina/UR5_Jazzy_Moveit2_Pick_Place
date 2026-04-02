from setuptools import setup, find_packages

package_name = 'ur5_pick_place'

setup(
    name=package_name,
    version='2.0.0',  # v2.0 with complete refactor
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install configuration files
        ('share/' + package_name + '/config',
            ['config/ur5_pick_place_config.yaml']),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'dataclasses-json',
    ],
    zip_safe=True,
    maintainer='Ngartoudjina',
    maintainer_email='todo@todo.com',
    description='UR5 pick and place with Robotiq 85 — modernized v2.0 with async, validation, and config-driven architecture',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── LEGACY: Original implementations (deprecated) ───────────
            'pose_sender_legacy   = ur5_pick_place.pose_sender:main',
            'add_object_legacy    = ur5_pick_place.add_object:main',
            'fake_object_pose_legacy = ur5_pick_place.fake_object_pose:main',
            'joint_state_pub_legacy = ur5_pick_place.joint_state_publisher:main',
            'pick_place_node_legacy = ur5_pick_place.pick_place_node:main',
            
            # ── MODERNIZED v2.0: New implementations ───────────────────
            'pick_place_node      = ur5_pick_place.pick_place_node_v2:main',
            'pose_sender          = ur5_pick_place.pose_sender_v2:main',
        ],
    },
)
