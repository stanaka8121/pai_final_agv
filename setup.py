import os
from glob import glob

from setuptools import setup

package_name = 'agv_factory'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai_student',
    maintainer_email='user@example.com',
    description='AI-assisted factory transport AGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_key = agv_factory.teleop_key:main',
        ],
    },
)
