from setuptools import setup

package_name = 'camera_udp_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysh',
    maintainer_email='poweq68@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

#        'camera_publisher = camera_udp_communication.camera_publisher:main',
        'camera_subscriber = camera_udp_communication.camera_subscriber:main',
    
        ],
    },
)
