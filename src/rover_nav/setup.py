from glob import glob
from setuptools import setup

package_name = 'rover_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edward',
    description='Mapping, localization and exploration',
    license='MIT',
    entry_points={'console_scripts': [
        'laser_odometry = rover_nav.laser_odometry:main',
        'global_localizer = rover_nav.global_localizer:main',
        'explorer = rover_nav.explorer:main',
        'rover_sim = rover_nav.sim:main',
    ]},
)
