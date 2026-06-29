from setuptools import setup

package_name = 'line_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='yourname@example.com',
    description='Line Manager ROS2 Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_manager = line_manager.line_manager:main',
        ],
    },
)
