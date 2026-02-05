from setuptools import find_packages, setup

package_name = 'tasks12'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tasks12_launch.py']),
        ('share/' + package_name + '/launch', ['launch/gap_follower_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='lakshyam4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = tasks12.wall_follower:main',
            'wall_follower_lidar = tasks12.wall_follower_lidar:main',
            'demo_lidar = tasks12.demo_lidar:main',
            'gap_follower_node = tasks12.gap_follower_node:main',
            'explorer = tasks12.explorer:main'
        ],
    },
)
