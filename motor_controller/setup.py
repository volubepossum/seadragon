from setuptools import setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    description='The motor controller package. map thrust to pwm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = motor_controller.motor_controller_node:main',
        ],
    },
)