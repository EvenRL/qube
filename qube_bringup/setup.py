from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'qube_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*.xacro')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='even',
    maintainer_email='52103879+EvenRL@users.noreply.github.com',
    description='Package for connecting qube_driver and qube_description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
