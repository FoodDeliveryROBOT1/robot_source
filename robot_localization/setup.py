from setuptools import find_packages, setup

package_name = 'robot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            # 'ekf_v1 = robot_localization.EKF_v1:main',
            'ekf_v2 = robot_localization.EKF_v2:main',
            'get_yaw_ekf = robot_localization.get_yaw:main',
        ],
    },
)
