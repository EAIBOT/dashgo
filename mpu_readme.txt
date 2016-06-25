roslaunch dashgo_bringup minimal.launch
roslaunch mpu6050_serial_to_imu demo.launch
roslaunch dashgo_bringup  robot_pose_ekf.launch

rosrun tf tf_echo /odom_combined /base_footprint

rosrun dashgo_bringup nav_grid_combined_1.py


Reducing  mpu node frequency,from 100 to 10
