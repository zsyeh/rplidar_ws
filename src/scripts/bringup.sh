nohup ros2 launch my_robot_description robot_slam_launch.launch.py & sleep 1
nohup ros2 launch  rf2o_laser_odometry rf2o_laser_odometry.launch.py & sleep 1
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=False map:=/home/eh/map3.yaml

