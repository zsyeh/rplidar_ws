import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. 包含 rf2o_laser_odometry
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rf2o_laser_odometry'),
                'launch',
                'rf2o_laser_odometry.launch.py'
            )
        )
    )

    # 2. 包含 nav2_bringup (导航系统)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        )
    )
    
    # 3. 包含 slam_toolbox (SLAM建图)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        )
    )

    # 4. 包含您自己的 robot_slam_launch.launch.py
    my_robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_description'),
                'launch',
                'robot_slam_launch.launch.py'
            )
        )
    )


    # 将所有要启动的子系统组合起来
    return LaunchDescription([
        rf2o_launch,
        nav2_launch,
        slam_toolbox_launch,
        my_robot_description_launch,
    ])
