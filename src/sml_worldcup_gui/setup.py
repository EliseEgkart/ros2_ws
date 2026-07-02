from setuptools import find_packages, setup

package_name = 'sml_worldcup_gui'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SML Team',
    maintainer_email='team@example.com',
    description='World Cup 2026 arena and EAI task visualization GUI (simulation + real-world monitoring).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sml_worldcup_gui = sml_worldcup_gui.app:main',
            'worldcup_gui = sml_worldcup_gui.app:main',
        ],
    },
)
