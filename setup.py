from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'blueberrypi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isabella',
    maintainer_email='isabella@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_disarm = blueberrypi.bluerov2_arm:yay',
            'dance = blueberrypi.bluerov2_dance:main',
            'depth_pid = blueberrypi.depth_pid:main',
            'pressure_depth = blueberrypi.pressure_depth:main',
            'target_depths = blueberrypi.target_depths:main',
            'heading_pid = blueberrypi.heading_pid:main',
            'move_forward = blueberrypi.translational_movement:main',
            'movement = blueberrypi.movement:main',
        ],
    },
)
