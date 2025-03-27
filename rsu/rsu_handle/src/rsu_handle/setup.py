from setuptools import setup

package_name = 'rsu_handle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erlang',
    maintainer_email='erlang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handle_topic_node = rsu_handle.handle_topic_node:main',
            'rsu_send_node = rsu_handle.rsu_send_node:main',
            'rsu_map_node = rsu_handle.rsu_map_node:main',
        ],
    },
)
