import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'legged_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['lib/*.so', 'policy/*.pt']   # 安装 lib 中的 .so 文件；安装策略文件
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 安装 config 文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tang-yucheng',
    maintainer_email='3143961287@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "legged_control = legged_control.legged_control:main"
        ],
    },
)
