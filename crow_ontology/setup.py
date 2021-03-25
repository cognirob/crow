from setuptools import setup
import os
from glob import glob

package_name = 'crow_ontology'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'data'), glob('data/*.*'))
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
            'tester = crow_ontology.examples.tester:main',
            'tester2 = crow_ontology.examples.tester2:main',
            'tester3 = crow_ontology.examples.tester3:main',
            'tester4 = crow_ontology.examples.tester4:main',
            'server = crow_ontology.crowracle_server:main_ros',
            'adder = crow_ontology.add_to_database:main'
        ],
    },
)
