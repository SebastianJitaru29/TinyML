from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'braccio_hardware_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'pyserial', 'rclpy'],
    zip_safe=True,
    # CHANGE Maintainer, Email, License
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Hardware interface standalone node for Braccio arm using ROS 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        # Use console_scripts for the standalone node
        'console_scripts': [
            'braccio_interface_node = braccio_hardware_interface.braccio_interface:main',
        ],
         # Make sure hardware_interface plugin entry point is REMOVED or commented out
         # 'hardware_interface.system_interface': [ ... ],
    },
)