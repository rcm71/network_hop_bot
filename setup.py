from setuptools import find_packages, setup

package_name = 'multi_hop_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/create_points.py',
            package_name+'/drone_controller.py',
            package_name+'/lemme_drive.py',
            package_name+'/drone_server.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='rcm71@pitt.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'processes = multi_hop_controller.processes:main'
            
            

        ],
    },
)
