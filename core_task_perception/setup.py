from setuptools import find_packages, setup

package_name = 'core_task_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sutharsan',
    maintainer_email='sutharsanmail311@gmail.com',
    description='Vision perception (YOLO target detection) for the Omokai core task.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_detector = core_task_perception.target_detector:main',
            'target_mover = core_task_perception.target_mover:main',
        ],
    },
)
