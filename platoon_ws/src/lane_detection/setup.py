from setuptools import setup
import os
from glob import glob

package_name = 'lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JunSpring',
    maintainer_email='wjdwnsqja08@gmail.com',
    description='Lane Detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detection = lane_detection.main:main',
        ],
    },
)
