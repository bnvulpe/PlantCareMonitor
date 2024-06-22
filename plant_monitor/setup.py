from setuptools import find_packages, setup
from glob import glob

package_name = 'plant_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/conf/', glob('conf/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'rosbag2_py'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_manager = plant_monitor.sensor_manager:main',
            'decision_maker = plant_monitor.decision_maker:main',
            'heat_act = plant_monitor.heat_act:main',
            'light_act = plant_monitor.light_act:main',
            'ph_act = plant_monitor.ph_act:main',
            'water_act = plant_monitor.water_act:main',
            'simple_bag_recorder = plant_monitor.simple_bag_recorder:main',
        ],
    },
)
