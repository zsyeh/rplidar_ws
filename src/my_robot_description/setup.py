import os  # <--- 这就是我们上次遗漏的关键一行！
from glob import glob
from setuptools import setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 urdf 文件夹下的所有 .urdf 文件
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # 安装 launch 文件夹下的所有 .launch.py 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eh',
    maintainer_email='eh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },scripts=['scripts/tf_checker.py'],
)
