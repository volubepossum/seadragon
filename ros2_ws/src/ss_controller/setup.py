from setuptools import setup

package_name = 'ss_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roland Domján',
    maintainer_email='domj.roli@gmail.com',
    description='State feedback controller node for ROS2',
    license='will do it later',
    entry_points={
        'console_scripts': [
            'controller_maker_node = ss_controller.controller_maker_node:main',
            'controller_node = ss_controller.controller_node:main',
            'model_estimator_node = ss_controller.model_estimator_node:main',
            'observer_maker_node = ss_controller.observer_maker_node:main',
            'observer_node = ss_controller.observer_node:main',
        ],
    },
)