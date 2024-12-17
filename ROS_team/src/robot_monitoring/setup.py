from setuptools import setup
import os
from glob import glob

package_name = 'robot_monitoring'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YSH',
    maintainer_email='your_email@example.com',
    description='Robot monitoring and control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_monitoring_node = robot_monitoring.robot_monitoring_node:main',
        ],
    },
)
