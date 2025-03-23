from setuptools import setup

package_name = 'simple_chat'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pratham Shah',
    maintainer_email='prathamshah0506@gmail.com',
    description='a simple chat using ROS2 publisher and subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chat_publisher = simple_chat.chat_publisher:main',
            'chat_subscriber = simple_chat.chat_subscriber:main',
        ],
    },
)
