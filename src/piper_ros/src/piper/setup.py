from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'piper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'numpy',
        'piper-sdk==0.6.1',
        'python-can==4.3.1',
        'scipy',
        'setuptools',
    ],
    zip_safe=True,
    maintainer='AgileX Robotics',
    maintainer_email='agilex@agilex.ai',
    description='ROS 2 driver and MoveIt trajectory bridge for the Piper arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_single_ctrl = piper.piper_ctrl_single_node:main',
            'piper_read_slave_joint = piper.piper_read_slave_joint:main',
        ],
    },
)
