from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'moonpie_lari'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonpie',
    maintainer_email='moonpie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_control = moonpie_lari.arduino_control:main',
            'arduino_comms = moonpie_lari.arduino_comms:main',
            'beacon_control = moonpie_lari.beacon_control:main'
        ],
    },
)
