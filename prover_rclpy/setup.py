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
            'rclpy_857 = src.rclpy_857:main',
            'rclpy_879 = src.rclpy_879:main',
            'rclpy_881 = src.rclpy_881:main',
            'rclpy_911 = src.rclpy_911:main',
            'rclpy_912 = src.rclpy_912:main',
            'rclpy_944 = src.rclpy_944:main',
            'rclpy_952 = src.rclpy_952:main',
            'rclpy_955 = src.rclpy_955:main',
            'rclpy_983 = src.rclpy_983:main',
            'rclpy_985 = src.rclpy_985:main',
            'rclpy_1000_pub = src.rclpy_1000_pub:main',
            'rclpy_1000_sub = src.rclpy_1000_sub:main',
            'rclpy_1016_service = src.rclpy_1016_service:main',
            'rclpy_1016_client = src.rclpy_1016_client:main',
            'rclpy_1018_server = src.rclpy_1018_server:main',
            'rclpy_1018_client = src.rclpy_1018_client:main',
            'rclpy_doctor_940 = src.rclpy_doctor_940:main',
            'rclpy_foo_940 = src.rclpy_foo_940:main',
            'rclpy_bar_940 = src.rclpy_bar_940:main',
            'ros2_1173 = src.ros2_1173:main',
            'rclpy_1030 = src.rclpy_1030:main',
        ],
    },
)
