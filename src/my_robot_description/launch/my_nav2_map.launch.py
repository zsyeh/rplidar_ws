# launch/start_robot_navigation.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # --- 声明 Launch 参数 ---
    # 声明一个名为 'map_path' 的启动参数，用于手动指定地图文件的绝对路径
    # 这里可以给一个默认值，或者留空强制用户在启动时提供
    declare_map_argument = DeclareLaunchArgument(
        'map_path',
        description='Full path to the map file to load.'
    )

    # --- 定位其他功能包的路径 ---
    # 获取 my_robot_description 功能包的共享目录路径
    my_robot_description_pkg = get_package_share_directory('my_robot_description')
    
    # 获取 nav2_bringup 功能包的共享目录路径
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    # 获取 rf2o_laser_odometry 功能包的共享目录路径
    rf2o_laser_odometry_pkg = get_package_share_directory('rf2o_laser_odometry')

    # --- 配置要包含的 Launch 文件 ---

    # 1. 配置 SLAM Toolbox 的启动文件
    # 这是我们要屏蔽输出的
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_description_pkg, 'launch', 'robot_slam_launch.launch.py')
        )
        # 注意：屏蔽输出的最佳方法在下面的文字说明中
    )

    # 2. 配置 Nav2 的启动文件
    # 这是我们要显示输出的
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        # 传递参数给 nav2 的 bringup_launch.py
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True', # 通常主文件启动时会设为True来自动启动Nav2
            'map': LaunchConfiguration('map_path')
        }.items()
    )

    # 3. 配置 RF2O 激光里程计的启动文件
    # 这也是我们要屏蔽输出的
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rf2o_laser_odometry_pkg, 'launch', 'rf2o_laser_odometry.launch.py')
        )
        # 注意：屏蔽输出的最佳方法在下面的文字说明中
    )

    # --- 组合并返回 LaunchDescription ---
    return LaunchDescription([
        declare_map_argument,
        
        # 依次启动
        slam_launch,
        rf2o_launch,
        nav2_launch
    ])
