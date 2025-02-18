from setuptools import setup
import os
from glob import glob

package_name = 'husky_commander'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'launch', 'launch_ros'],
    zip_safe=True,
    maintainer='Ainhoa Arnaiz',
    maintainer_email='ainhoaarnaiz@gmail.com',
    description='Hand teleoperation node using Mediapipe and MoveIt2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_teleop = husky_commander.hand_teleop:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),  # If you have a launch file
    ],
)
