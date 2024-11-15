import os
from glob import glob
from setuptools import setup

package_name = 'dragonfly_sim'

setup(
    name=package_name,
    version='1.0.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/*.xml')),
        (os.path.join('lib', package_name), glob('scripts/arducopter.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
)
