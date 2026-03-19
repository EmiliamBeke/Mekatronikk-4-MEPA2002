import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mekk4_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='gruppe5pi5@users.noreply.github.com',
    description='Bringup and launch tooling for the MEKK4 robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bno085_node = mekk4_bringup.bno085_node:main',
            'mega_driver_node = mekk4_bringup.mega_driver_node:main',
            'zero_joint_state_publisher = mekk4_bringup.zero_joint_state_publisher:main',
        ],
    },
)
