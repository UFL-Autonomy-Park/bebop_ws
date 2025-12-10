import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rho_filter_eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgardenswartz@ad.ufl.edu',
    maintainer_email='max.gardenswartz@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'signal_publisher = rho_filter_eval.signal_publisher:main',
            'error_monitor = rho_filter_eval.error_monitor:main',
        ],
    },
)
