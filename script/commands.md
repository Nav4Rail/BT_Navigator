ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch nav2_bringup localization_launch.py use_sim_time:=True map:=/home/mlatoundji/studies/dev/nav4rails/BT_Navigator/maps/exploration_map.yaml

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/mlatoundji/studies/dev/nav4rails/BT_Navigator/params/nav2_params.yaml

ros2 launch nav2_bringup rviz_launch.py