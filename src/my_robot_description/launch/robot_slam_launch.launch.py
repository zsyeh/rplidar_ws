import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 定位您的 URDF 文件
    urdf_file_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'hjy_car.urdf'
    )

    # 2. 定义 robot_state_publisher 节点
    # 这个节点唯一的任务就是读取 URDF 文件并发布 TF 静态变换
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # 使用 xacro 命令来加载URDF文件，这是一种通用且健壮的方法
            'robot_description': Command(['xacro ', urdf_file_path])
        }]
    )

    # 3. 返回只包含这一个节点的 LaunchDescription
    return LaunchDescription([
        robot_state_publisher_node
    ])
