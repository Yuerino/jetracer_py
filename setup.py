from setuptools import setup

package_name = 'jetracer_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'controller = jetracer_py.subscriber_member_function:main'
            'teleop = jetracer_py.publisher_member_function:main'
        ],
    },
)
