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
            'rgbd_images_sync = slam.scripts.rgbd_images_sync:main',
            'imu_diagnostics = slam.scripts.imu_diagonstic:main',
            'height_fusion_node = slam.height_fusion_node:main',
            'ground_plane_publisher = slam.ground_plane_publisher:main',
            'height_monitor = slam.height_monitor:main',
        ],
    },
    package_data={
        package_name: ['resource/*'],
    },
)