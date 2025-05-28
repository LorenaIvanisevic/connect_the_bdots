from setuptools import find_packages, setup
from glob import glob

package_name = 'connect_the_bdots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui.launch.xml', 'launch/prarob_manipulator.launch.py', 'launch/prarob_manipulator.rviz']),
        ('share/' + package_name + '/images', ['images/roboticArm.png']),
        ('share/' + package_name + '/urdf', glob("urdf/*")),
        ('share/' + package_name + '/meshes', glob("meshes/*")),
        ('share/' + package_name + '/scripts', ['scripts/move_robot.py']),
 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='lorenaivanisevic@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = connect_the_bdots.my_node:main',
            'gui_prarob_node = connect_the_bdots.gui_plugin:main'
        ],
         
    },
)
