import os
from glob import glob
from setuptools import setup

package_name = 'jetracer_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'waveshare'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'traitlets',
        'adafruit-circuitpython-servokit'],
    zip_safe=True,
    maintainer='yuerino',
    maintainer_email='50553280+Yuerino@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = jetracer_py.controller:main',
            'teleop = jetracer_py.teleop:main'
        ],
    },
)
