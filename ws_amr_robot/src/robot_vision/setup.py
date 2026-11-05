from setuptools import setup
import os
from glob import glob

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raenaldiardiansyah',
    maintainer_email='raenaldi.ardiansyah30@gmail.com',
    description='Kamera Kinect v1: deteksi objek (YOLO11n), depth, AprilTag checkpoint',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
