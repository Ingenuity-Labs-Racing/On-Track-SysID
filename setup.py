import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'on_track_sys_id'

# Collect model files preserving subdirectory structure
model_data_files = []
for model_dir in ['NUC2', 'NUC5', 'NUC6', 'NUC7', 'SIM']:
    model_path = os.path.join('models', model_dir)
    if os.path.isdir(model_path):
        files = glob(os.path.join(model_path, '*'))
        model_data_files.append(
            (os.path.join('share', package_name, model_path), files)
        )

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
    ] + model_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onur Dikici',
    maintainer_email='odikici@ethz.ch',
    description='The on_track_sys_id package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'on_track_sys_id_node = on_track_sys_id.on_track_sys_id_node:main',
        ],
    },
)
