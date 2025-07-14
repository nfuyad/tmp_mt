from setuptools import find_packages, setup

package_name = 'mt_controller'

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
    maintainer='nafisfuyad',
    maintainer_email='nafisfuyad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard= mt_controller.keyboard:main',
            'cmddtoserial= mt_controller.cmdToSerial:main',
            'joy_controller= mt_controller.joy_controller:main',
            'arm_controller= mt_controller.arm_control_joy:main'
        ],
    },
)
