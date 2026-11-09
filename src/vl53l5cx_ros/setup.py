from glob import glob
from setuptools import setup

package_name = 'vl53l5cx_ros'

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
    description='VL53L5CX 8x8 ToF driver and matrix viewers',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vl53l5cx_node = vl53l5cx_ros.vl53l5cx_node:main',
            'tof_matrix_viewer = vl53l5cx_ros.matrix_viewer:main',
            'tof_heatmap_viewer = vl53l5cx_ros.heatmap_viewer:main',
            'tof_hazard_node = vl53l5cx_ros.hazard_node:main',
            'tof_geometry = vl53l5cx_ros.geometry_tool:main',
        ],
    },
)
