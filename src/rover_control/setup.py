from glob import glob
from setuptools import setup

package_name = 'rover_control'

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
    description='ESP32 serial bridge + keyboard teleop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp32_bridge = rover_control.esp32_bridge:main',
            'rover_teleop = rover_control.rover_teleop:main',
        ],
    },
)
