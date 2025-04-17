from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'moonpie_osamu'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonpie',
    maintainer_email='nath.h.jensen@gmail.com',
    description='Package for the Osamu computer for Mines Lunabotics Team Moonpie 2024',
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
