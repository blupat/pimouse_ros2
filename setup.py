from setuptools import setup

package_name = 'pimouse_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blupat',
    maintainer_email='oosmyss@gmail.com',
    description='Pimouse driver package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lightsensors = ' + package_name + '.lightsensors:main'
        ],
    },
)
