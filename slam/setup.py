from setuptools import setup
import os
from glob import glob

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'),
         glob('params/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aero',
    maintainer_email='aero@todo.todo',
    description='SLAM package with QR code detection and RTAB-Map integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detection_node = slam.scripts.qr_detection_node:main',
            'rangefinder = slam.scripts.rangefinder:main',
            'imu_diagnostics = slam.scripts.imu_diagonstic:main',
            'multi_camera_vio = slam.scripts.multi_camera_vio:main',
            'landing = slam.scripts.landing:main',
            'height_monitor = slam.scripts.height_monitor:main',
            'odom_converter = slam.scripts.odom_converter:main',
        ],
    },
    package_data={
        package_name: ['resource/*'],
    },
)