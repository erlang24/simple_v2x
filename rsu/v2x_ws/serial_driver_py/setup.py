from setuptools import setup, find_packages
from glob import glob
import  os

package_name = 'serial_driver_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=["serial_driver_py","md_serial"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + "serial_driver"]),
        ('share/' + "serial_driver_py", ['package.xml']),
        (os.path.join('share', "serial_driver_py", 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', "serial_driver_py", 'launch'), glob('launch/*.launch.py')),
    ],
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
        'serial_driver = serial_driver_py.serial_driver:main',
        ],
    },
)
