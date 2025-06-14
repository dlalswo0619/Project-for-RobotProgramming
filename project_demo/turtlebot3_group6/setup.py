from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_group6'

setup(
    name=package_name,
    version='2.3.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools', 'launch', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@todo.todo',
    keywords=['ROS', 'ROS2', 'group6', 'rclpy'],
    classifiers=[
    	'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO: Package description',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'turtlebot3_astar_move = ' 			
        	'turtlebot3_group6.turtlebot3_astar_move.'
        	'turtlebot3_astar_move:main',
        	'cam_shape_detector = '
        	'turtlebot3_group6.cam_shape_detector.'
        	'cam_shape_detector:main',
        	'path_node = '
        	'turtlebot3_group6.path_node.'
        	'path_node:main',
        	'object_to_goal = '
        	'turtlebot3_group6.object_to_goal.'
        	'object_to_goal:main'
        ],
    },
)
