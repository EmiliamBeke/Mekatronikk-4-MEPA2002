from setuptools import find_packages, setup

package_name = 'robot_sim_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olav',
    maintainer_email='olavdrage@gmail.com',
    description='Simulation-only control adapters for the tracked robot.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tracked_cmd_vel_adapter = robot_sim_control.tracked_cmd_vel_adapter:main',
            'gripper_sim_adapter = robot_sim_control.gripper_sim_adapter:main',
            'robotarm_gui = robot_sim_control.robotarm_gui:main',
            'robotarm_state_adapter = robot_sim_control.robotarm_state_adapter:main',
            'dist_sensor_bridge = robot_sim_control.dist_sensor_bridge:main',
        ],
    },
)
