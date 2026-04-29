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
    install_requires=[
        'setuptools',
        # GUI 의존성 (pip install로도 미리 설치 필요)
        # PyQt6, paramiko, roslibpy, requests, qrcode, flask, pyyaml
    ],
    zip_safe=True,
    maintainer='SmartCart Team (GUI 담당)',
    maintainer_email='team@smartcart.local',
    description='Smart Cart GUI (PyQt6 + Flask 결제 + Nav2 연동)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 메인 GUI (관제 + Flask 결제 동시 실행)
            'frictionless_gui = sc_gui.main:main',
            # Flask 결제 서버만 단독 실행 (테스트용)
            'cart_server = sc_gui.cart_gui:main',
        ],
    },
)