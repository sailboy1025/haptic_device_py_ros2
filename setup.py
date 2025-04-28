from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hd_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install device_cal folder and its contents
        (os.path.join('share', package_name, 'device_cal'), glob('hd_py/device_cal/*')),
    ],
    install_requires=['setuptools', 'websockets', 'orjson'],
    zip_safe=True,
    maintainer='erie_lab',
    maintainer_email='sxj749@case.edu',
    description='A ROS2 Python package to publish data from all haptic devices in the Erie Lab.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inv3_web = hd_py.inverse3.inverse3_websocket:main',
            'inv3_calibration = hd_py.inverse3.inverse3_calibration:main'
        ],
    },
)
