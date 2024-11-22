from setuptools import setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/thrust_mapping.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Volubepossum',
    maintainer_email='domj.roli@gmail.com',
    description='The motor_controller package. Maps requested thrust to pwm values, intended to work with the i2c_pwm_board package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = motor_controller.motor_controller_node:main',
        ],
    },
)
print('setup.py executed')