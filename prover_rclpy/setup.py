from setuptools import find_packages
from setuptools import setup

package_name = 'prover_rclpy'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Tomoya Fujita',
    author_email='Tomoya.Fujita@sony.com',
    maintainer='Tomoya Fujita',
    maintainer_email='Tomoya.Fujita@sony.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'rclcpy test sample to reproduce and investigate issues, easy to distribute.'
    ),
    license='Apache License, Version 2.0',
    tests_require=[''],
    entry_points={
        'console_scripts': [
            'talker = src.talker:main',
            'listener = src.listener:main',
            #'rclpy_585 = src.rclpy_585:main',
            #'rclpy_760 = src.rclpy_760:main',
            'rclpy_792 = src.rclpy_792:main',
            'rclpy_client_822 = src.rclpy_client_822:main',
            'rclpy_server_822 = src.rclpy_server_822:main',
            'rclpy_827 = src.rclpy_827:main',
            'ros2_1173 = src.ros2_1173:main',
        ],
    },
)
