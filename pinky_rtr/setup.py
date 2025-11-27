from setuptools import find_packages, setup
import os, glob

package_name = 'pinky_rtr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.*'))),
        ('share/' + package_name + '/params', glob.glob(os.path.join('params', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pl3',
    maintainer_email='kyung133851@pinklab.art',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'amcl_publisher = pinky_rtr.amcl_publisher:main',
            'drive = pinky_rtr.drive:main',
            'waypoint = pinky_rtr.waypoint:main',
            'vel_smoother = pinky_rtr.vel_smoother:main',
        ],
    },
)
