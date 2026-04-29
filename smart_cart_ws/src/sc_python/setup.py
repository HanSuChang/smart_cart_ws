from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sc_python'


model_files = glob('sc_python/models/*.pt')
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if model_files:
    data_files.append(
        (os.path.join('share', package_name, 'models'), model_files)
    )

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='Han Su-chang',
    maintainer_email='hsc0724321@gmail.com',
    description='Smart Cart Python AI 노드 (YOLOv8n + DeepSORT)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_tracker  = sc_python.person_tracker:main',
            'item_classifier = sc_python.item_classifier:main',
            
        ],
    },
)