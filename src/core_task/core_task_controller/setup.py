from glob import glob

from setuptools import find_packages, setup

package_name = 'core_task_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sutharsan',
    maintainer_email='sutharsanmail311@gmail.com',
    description='Python control node that drives Nav2 for the Omokai core task.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_limiter = core_task_controller.cmd_vel_limiter:main',
            'operation_controller = core_task_controller.Operation_controller:main',
        ],
    },
)
