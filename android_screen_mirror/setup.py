from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'android_screen_mirror'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'opencv-python',
        'cv_bridge',
        'numpy',
    ],
    zip_safe=True,
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'screen_mirror_node = android_screen_mirror.screen_mirror_node:main',
            'subscribe_node = android_screen_mirror.subscribe_node:main',
        ],
    },
)
