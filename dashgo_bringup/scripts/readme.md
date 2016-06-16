test mpu6050

roslaunch dashgo_bringup minimal.launch
roslaunch mpu6050_serial_to_imu demo.launch
roslaunch dashgo_bringup  robot_pose_ekf.launch
rosrun dashgo_bringup nav_grid_combined_1.py
