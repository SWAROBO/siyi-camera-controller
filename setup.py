import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'a8camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, 'siyi_sdk', 'siyi_sdk.*']),  # siyi_sdk 포함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    package_data={package_name: ['siyi_sdk/*']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aimas',
    maintainer_email='aimas.lund@gmail.com',
    description='',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stream = a8camera.camera_stream:main',
            'controller = a8camera.camera_controller:main',
        ],
    },
)
