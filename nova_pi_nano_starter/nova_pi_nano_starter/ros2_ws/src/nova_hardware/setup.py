from setuptools import setup

setup(
    name='nova_hardware',
    version='0.1.0',
    packages=['nova_hardware'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/nova_hardware']),
        ('share/nova_hardware', ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='NOVA project',
    maintainer_email='maintainer@example.com',
    description='Dry-run Nano 33 BLE Sense Rev2 IMU and USB serial bridge',
    license='MIT',
    entry_points={'console_scripts': ['hardware_bridge = nova_hardware.bridge:main']},
)
