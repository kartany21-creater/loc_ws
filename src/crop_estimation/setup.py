from setuptools import setup
import os
from glob import glob

package_name = 'crop_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='your@email.com',
    description='Z velocity and yaw estimation with RealSense and UM7',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estimate_node = crop_estimation.estimate_node:main',
            'um7_node = crop_estimation.um7_node:main',
            'odom_node = crop_estimation.odom_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)

