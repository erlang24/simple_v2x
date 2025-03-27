from setuptools import setup, find_packages
from glob import glob
import  os

package_name = 'mqtt_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=["mqtt_bridge"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    # py_modules=['DefPosition_pb2',  
    #             'FaultStatus_pb2',  
    #             'MsgFrame_pb2',  
    #             'SPAT_pb2',  
    #             'VBM_pb2' , 
    #             'VehBrake_pb2',  
    #             'VehClass_pb2',  
    #             'VehSize_pb2' , 
    #             'VehStatus_pb2'],

    package_dir={'':'src' },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='docker',
    maintainer_email='cjiangxumin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        'mqtt_bridge = mqtt_bridge.main:main',
        'mqtt_pub = mqtt_bridge.emq_pub_node:main',
        'mqtt_sub = mqtt_bridge.emq_sub_node:main',
        ],
    },
)
