from setuptools import find_packages, setup

package_name = 'mqttTasn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/asn',["asn/BSM.asn","asn/DefMotion.asn","asn/DefPositionOffset.asn","asn/Map.asn","asn/MapLink.asn","asn/MapPoint.asn","asn/MsgFrame.asn","asn/RSM.asn","asn/SPATIntersectionState.asn","asn/VehClass.asn",
                      "asn/VehSafetyExt.asn","asn/VehStatus.asn","asn/DefAcceleration.asn","asn/DefPosition.asn","asn/DefTime.asn","asn/MapLane.asn","asn/MapNode.asn","asn/VehSize.asn","asn/VehEmgExt.asn","asn/MapSpeedLimit.asn",
                      "asn/RSI.asn","asn/SignalPhaseAndTiming.asn","asn/VehBrake.asn"])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='promote',
    maintainer_email='2391701547@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'mqttTasn = mqttTasn.main:main',
        ],
    },
)
