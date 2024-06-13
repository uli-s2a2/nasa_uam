from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'crazyflie_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all param files.
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*params.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='llanesc',
    maintainer_email='christian.llanes@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = crazyflie_bridge.crazyflie_bridge_odometry_publisher:main'
        ],
    },
)
