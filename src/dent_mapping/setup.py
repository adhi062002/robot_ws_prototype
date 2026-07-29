from setuptools import setup, find_packages
from glob import glob

package_name = 'dent_mapping'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py') + glob('launch/*.xml')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='ROS2 dent mapping node using SuperGlue + TEASER++ + Colored ICP',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_node = dent_mapping.realsense_frame_capture_node:main',
        ],
    },
)
