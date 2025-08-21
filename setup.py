from setuptools import find_packages, setup

package_name = 'create3_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sugandh Mittal',
    maintainer_email='sugandhka2k@gmail.com',
    description='This node prevents the Create3 from any obstacle big or small',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_controller = create3_obstacle_avoidance.reactive_controller:main'
        ],
    },
)
