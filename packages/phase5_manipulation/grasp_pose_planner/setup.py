from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'grasp_pose_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyujun',
    maintainer_email='hyujun@todo.todo',
    description='Grasp pose planning action server',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'grasp_planner_node = grasp_pose_planner.grasp_planner_node:main',
        ],
    },
)
