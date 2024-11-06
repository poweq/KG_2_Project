from setuptools import setup
import os
from glob import glob

package_name = 'agv_control_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Include the 'package.xml' file
        (os.path.join('share', package_name), ['package.xml']),
        # Include resource marker file
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        # Include any launch files (if you have them)
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Python-based AGV control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_contorl = agv_control_py.agv_controller_node:main',
        ],
    },
)
