from glob import glob
from setuptools import setup

package_name = 'nturt_bag_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
        ('share/' + package_name + '/bags', glob('bags/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='QuantumSpawner',
    maintainer_email='jet22854111@gmail.com',
    description='ROS2 package for recording ros message into a bag.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nturt_bag_recorder_node = nturt_bag_recorder.nturt_bag_recorder_node:main'
        ],
    },
)
