# setup.py
from setuptools import setup
from glob import glob

package_name = 'mqtt_ros_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjohn0714',
    maintainer_email='kjohn0714@example.com',
    description='ROS2 ↔ MQTT bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mqtt_ros_bridge = mqtt_ros_bridge.mqtt_ros_bridge:main',
            'mqtt_cmd_to_ros = mqtt_ros_bridge.mqtt_cmd_to_ros:main',
        ],
    },
)
