import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'person_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # 이 줄 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minju',
    maintainer_email='minjukim01110204@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_detector = person_detection.person_detector:main',
            'person_detect_only = person_detection.person_detect_only:main',
            'person_tracking_only = person_detection.person_tracking_only:main',
            
            # tb1
            'detect_fsm_tb1 = person_detection.detect_fsm_tb1:main',
            'tracking_fsm_tb1 = person_detection.tracking_fsm_tb1:main',
            
            # tb2
            'detect_fsm_tb2 = person_detection.detect_fsm_tb2:main',
            'tracking_fsm_tb2 = person_detection.tracking_fsm_tb2:main',
        ],
    },
)
