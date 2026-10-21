from setuptools import find_packages, setup

package_name = 'RovGui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # إضافة ملف الواجهة للـ installation
        ('share/' + package_name + '/RovGui',
            ['RovGui/rov.ui']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hazemyoussef',
    maintainer_email='hazem.peny@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rovgui = RovGui.rovgui:main',
        ],
    },
)