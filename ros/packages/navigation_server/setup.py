import os
from setuptools import setup

package_name = 'navigation_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'weights'), ['share/weights/navigation.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csokap',
    maintainer_email='peter.csoka@innovitech.hu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_server = navigation_server.navigation_server:main'
        ],
    },
)
