from setuptools import setup

package_name = 'ur5_pick_place_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aakash Kavi',
    maintainer_email='your.email@example.com',
    description='MoveIt! 2 pick-and-place execution node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_place_node = ur5_pick_place_manipulation.pick_place_node:main',
        ],
    },
)
