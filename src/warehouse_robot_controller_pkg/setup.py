import os # Operating system library
from glob import glob # Handles file path names
from setuptools import setup

package_name = 'warehouse_robot_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Path to the launch file      
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu-ros',
    maintainer_email='figoowen2003@126.com',
    description='Control the motion of a warehouse mobile robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'robot_controller = warehouse_robot_controller_pkg.robot_controller:main',
          'robot_estimator = warehouse_robot_controller_pkg.robot_estimator:main',         
        ],
    },
)
