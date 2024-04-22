from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name, package_name_in],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pengkung',
    maintainer_email='chetsokhpanha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dstar = robot_planner.dstarlite_scan_V2:main',
            'nmpc = robot_planner.nmpc:main',
            'pps = robot_planner.pure_pursuit:main',
        ],
    },
)
