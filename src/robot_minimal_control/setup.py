from setuptools import find_packages, setup

package_name = 'robot_minimal_control'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_loop = robot_minimal_control.cmd_vel_loop:main',
            'sim_keyboard_teleop = robot_minimal_control.sim_keyboard_teleop:main',
        ],
    },
)
