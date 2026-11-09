from glob import glob
from setuptools import setup

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edward',
    description='AutoNav bring-up',
    license='MIT',
    entry_points={'console_scripts': [
        'setup_serial_ports = rover_bringup.setup_serial_ports:main',
        'preflight = rover_bringup.preflight:main',
    ]},
)
