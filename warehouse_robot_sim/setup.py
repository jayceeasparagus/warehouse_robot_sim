import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'warehouse_robot_sim'


def package_files(directory):
    files = glob(os.path.join(directory, '*'))
    return [file for file in files if os.path.isfile(file)]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', package_files('launch')),
        ('share/' + package_name + '/worlds', package_files('worlds')),
        ('share/' + package_name + '/maps', package_files('maps')),
        ('share/' + package_name + '/config', package_files('config')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayce',
    maintainer_email='jayceeasparagus@gmail.com',
    description='ROS 2 warehouse robot simulation with TurtleBot navigation, SLAM, Nav2, and task dispatching.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'delivery_task_node = warehouse_robot_sim.delivery_task_node:main',
            'job_dispatcher_node = warehouse_robot_sim.job_dispatcher_node:main',
        ],
    },
)


