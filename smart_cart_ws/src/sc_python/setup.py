from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sc_python'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 모델 폴더 내의 모든 .pt 파일을 빌드 시 포함하도록 추가
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Han Su-chang', # 파이썬 담당 (나중에 이름 교체 예정)
    maintainer_email='hsc0724321@gmail.com.',
    description='Smart Cart Python AI 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_tracker   = sc_python.person_tracker:main',
            'item_classifier  = sc_python.item_classifier:main',
        ],
    },
)