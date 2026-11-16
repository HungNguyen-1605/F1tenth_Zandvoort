from setuptools import setup

package_name = 'lidar_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'lidar_test = lidar_test.lidar_test:main',
	    'autonomous_drive = lidar_test.autonomous_controller:main',
        ],
    },
)
