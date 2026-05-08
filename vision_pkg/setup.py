import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='richard',
    maintainer_email='richard.solheim21@gmail.com',
    description='USB webcam capture and HSV-based coloured cube detection',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node    = vision_pkg.camera_node:main',
            'detection_node = vision_pkg.detection_node:main',
        ],
    },
)
