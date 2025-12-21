from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you create them in the future
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'smbus2',
        'adafruit-circuitpython-ads1115', # Required for the Voltage Sensor ADC
    ],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='omar.ahmed0122966@gmail.com',
    description='ROS 2 package for AMR sensors integration.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "load_cell         = sensors.load_cell:main",
            "battery_LEDS      = sensors.battery_LEDS:main",
            "voltage_sensor    = sensors.voltage_sensor:main"
        ],
    },
)