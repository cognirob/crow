from setuptools import setup
import os
from glob import glob

package_name = 'crow_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/run_all.sh']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'crow_control/data'), glob('crow_control/data/*.yaml')),
        (os.path.join('share', package_name, 'data'), glob('crow_control/data/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syxtreme',
    maintainer_email='radoslav.skoviera@cvut.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logic = crow_control.logic:main',
            'dummy = crow_control.testing.dummy_action_robot:main',
            'dummy_nl_input = crow_control.testing.dummy_nl_input_curses:main',
            'ptest = crow_control.testing.test_param:main',
            'stest = crow_control.testing.test_param_server:main',
            'profiler = crow_control.utils.profiler_node:main',
            'aplanner = crow_control.assembly_planner:main',
            # 'amonitor = crow_control.assembly_monitor:main',
            'famonitor = crow_control.assembly_monitor_fake:main',
            'monitor = crow_control.monitor:main',
            'wxvis = crow_control.wx_visualizator:main'
        ],
    },
)
