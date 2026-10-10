from setuptools import setup
import os
from glob import glob

package_name = 'airsim_bridge'

setup(
    name=package_name,
    version='2.0.1',
    packages=[package_name, 'airsim'],
    package_dir={
        package_name: 'src',
        'airsim': 'src/airsim',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nuno Marques',
    maintainer_email='nuno.marques@dronesolutions.io',
    description='AirSim bridge for ROS2',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transform = transform_node:main',
        ],
    },
)
