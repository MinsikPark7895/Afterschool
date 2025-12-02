# setup.py
import os, glob
from setuptools import setup

package_name = 'turtlebot3_multi_robot'

setup(
    name=package_name,
    version='2.2.6',  # Match package.xml
    packages=[package_name],  # Include the package directory
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install other share files (launch, models, etc.) if needed
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arshad Mehmood',
    maintainer_email='arshadm78@yahoo.com',
    description='Multi robot support TurtleBot3 in Gazebo',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'coverage_planner = turtlebot3_multi_robot.coverage_planner:main',
            'fsm = turtlebot3_multi_robot.fsm:main',
            'person_detector = turtlebot3_multi_robot.person_detector:main',
            'person_tracker = turtlebot3_multi_robot.person_tracker:main',
            'trail_publisher = turtlebot3_multi_robot.trail_publisher:main',
        ],
    },
)