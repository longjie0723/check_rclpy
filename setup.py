from setuptools import setup

package_name = 'check_rclpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tajima',
    maintainer_email='tajima.ryosuke@techmagic.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_service_from_service = check_rclpy.check_service_from_service:main',
            'check_service_from_timer = check_rclpy.check_service_from_timer:main',
            'check_service_from_task = check_rclpy.check_service_from_task:main'
        ]
    },
)
