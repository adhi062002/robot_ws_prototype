from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='Teleoperation package for UGV with smooth control and hardware interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    data_files=[
        # This is the package index marker (required by colcon)
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Install the package.xml in the package share directory
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'smooth_teleop = ugv_teleop.smooth_teleop_node:main',
            'teleop = ugv_teleop.teleop_node:main',
            'hardware_interface = ugv_teleop.hardware_interface_node:main',
            'forward_kinematics = ugv_teleop.forward_kinematics_node:main',
            'ef_logger = ugv_teleop.hp60c_logger_node:main',
            'orb_logger = ugv_teleop.orbslam3_logger_node:main',
            'rtab_logger = ugv_teleop.rtab_logger_node:main',
            'rgbdi_logger = ugv_teleop.rgbdi_logger_node:main',
            'vins_logger = ugv_teleop.vins_logger_node:main',
            'odom_pub = ugv_teleop.vio_odom_pub:main',
            'odom_cmp = ugv_teleop.prototype:main',
            'compress = ugv_teleop.compression_node:main',
            'dist = ugv_teleop.distance_calculator:main',
            'dc_motor = ugv_teleop.dc_motor_teleop:main'
        ],
    },
)

