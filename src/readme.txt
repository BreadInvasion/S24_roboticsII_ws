ros2 launch maze maze.launch.py
ros2 bag play lidar_scan/
ros2 launch sllidar_ros2 sllidar_launch.py
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
ros2 run yahboomcar_ctrl yahboom_keyboard