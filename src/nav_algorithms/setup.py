from setuptools import find_packages, setup

package_name = 'nav_algorithms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_follower_launch.py']),
        ('share/' + package_name + '/launch', ['launch/gap_follower_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='lakshyam4@gmail.com',
    description='Reactive navigation nodes: gap followers, wall followers, frontier explorer',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = nav_algorithms.wall_follower:main',
            'wall_follower_lidar = nav_algorithms.wall_follower_lidar:main',
            'demo_lidar = nav_algorithms.demo_lidar:main',
            'gap_follower_node = nav_algorithms.gap_follower_node:main',
            'explorer = nav_algorithms.explorer:main',
            'gap_follower_ackermann = nav_algorithms.gap_follower_ackermann:main'
        ],
    },
)
