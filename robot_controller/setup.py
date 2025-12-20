from setuptools import setup
from glob import glob
import os

package_name = 'robot_controller'

setup(
    # Tên distribution (dùng trong importlib.metadata.distribution)
    # Giữ dấu gạch ngang như script đang gọi: 'robot-controller'
    name='robot-controller',
    version='0.0.0',
    # Tên package Python (thư mục robot_controller/)
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # cài các file launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='minh',
    maintainer_email='minhphamnm@gmail.com',
    description='Bilateral teleoperation controllers for mobile robot and haptic device',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fictitious_force_node = robot_controller.fictitious_force_node:main',
            'master_controller = robot_controller.master_controller:main',
            'slave_controller = robot_controller.slave_controller:main',
            'delay_relay_node = robot_controller.delay_relay_node:main',
            'data_logger = robot_controller.data_logger:main',
            'check_limit_vel = robot_controller.check_limit_vel:main',
            'cal_delay = robot_controller.cal_delay:main',
            'talker = robot_controller.talker:main',
        ],
    },
)
