from setuptools import setup, find_packages

package_name = 'sc_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SmartCart Team (GUI 담당)',
    maintainer_email='team@smartcart.local',
    description='Smart Cart GUI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cart_gui = sc_gui.cart_gui:main',
        ],
    },
)
