from glob import glob

from setuptools import find_packages, setup

package_name = 'perception_launch_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/host_profiles',
            glob('host_profiles/*.yaml') + ['host_profiles/README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyujun',
    maintainer_email='hyujun@todo.todo',
    description='Shared launch helpers for perspective_grasp packages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
