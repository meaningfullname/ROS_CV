from glob import glob
from setuptools import setup
setup(name='tabletop_robot', version='0.1.0', packages=['tabletop_robot'],
 data_files=[('share/ament_index/resource_index/packages', ['resource/tabletop_robot']),
 ('share/tabletop_robot', ['package.xml']),
 ('share/tabletop_robot/launch', glob('launch/*.launch.py'))],
 install_requires=['setuptools'], zip_safe=True,
 maintainer='Developer', maintainer_email='developer@example.com',
 description='Camera guided tabletop robot', license='MIT',
 entry_points={'console_scripts': [
 'simulator = tabletop_robot.simulator_node:main',
 'perception = tabletop_robot.perception_node:main',
 'task_manager = tabletop_robot.task_node:main']})
