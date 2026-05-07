import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'jetcobot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='Hybrid vision pipeline for Jetcobot (ROS 2 Jazzy)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'coord_transform_node            = jetcobot_vision.coord_transform_node:main',
            'vision_pick_place_node          = jetcobot_vision.vision_pick_place_node:main',
            'pick_place_coordinator_node     = jetcobot_vision.pick_place_coordinator_node:main',
            # Deprecated (use vision_pick_place_node instead):
            # 'vision_pick_node             = jetcobot_vision.vision_pick_node:main',
            # 'vision_place_node            = jetcobot_vision.vision_place_node:main',
        ],
    },
)
