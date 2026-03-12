from setuptools import find_packages, setup

package_name = 'mekk4_perception'

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
    maintainer='ubuntu',
    maintainer_email='gruppe5pi5@users.noreply.github.com',
    description='Perception nodes for teddy detection using YOLO/NCNN.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teddy_detector = mekk4_perception.teddy_detector:main',
        ],
    },
)
