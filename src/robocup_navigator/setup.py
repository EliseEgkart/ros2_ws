import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robocup_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'params'),
            glob('params/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonshot',
    maintainer_email='ky942400@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robocup_navigator = robocup_navigator.navigator:main',
            'robocup_navigator_nav2 = robocup_navigator.navigator_nav2:main',
            'robocup_current_pose = robocup_navigator.current_pose:main',
            'waypoint_editor = robocup_navigator.waypoint_editor:main',
        ],
    },
)
