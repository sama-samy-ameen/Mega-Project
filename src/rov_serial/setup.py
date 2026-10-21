from setuptools import find_packages, setup

package_name = 'rov_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'ahrs>=0.4.0'],
    zip_safe=True,
    maintainer='aliaa',
    maintainer_email='aliaaslsayed745@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rov_serial_bridge = rov_serial.rov_serial_bridge:main',
            'rov_attitude_ekf = rov_serial.attitude_ekf:main',
        ],
    },
)
