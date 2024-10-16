from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'tire_roller_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, glob('resource/*.dbc')),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koceti',
    maintainer_email='jpkim@koceti.re.kr',
    description='Tire Roller Controller',
    # license='koceti',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
