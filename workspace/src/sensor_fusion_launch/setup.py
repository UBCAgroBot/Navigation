from setuptools import setup
import os
from glob import glob

package_name = 'sensor_fusion_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],  # <-- IMPORTANT: no Python packages
    data_files=[
        # Required for ROS 2 package indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='ahandhingra@gmail.com',
    description='Launch files for sensor fusion bringup',
    license='Apache-2.0',
)