#! /usr/bin/python

import PyKDL
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import *
import threading
import os
import subprocess
import yaml


def quat_to_angle(quat):
  rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
  return rot.GetRPY()[2]
def normalize_angle(angle):
  res = angle
  while res > pi:
    res -= 2.0*pi
  while res < -pi:
    res += 2.0*pi
  return res


class CalibrateRobot1:
  def __init__(self):
    self.lock = threading.Lock()
    self.sub_imu = rospy.Subscriber('odom', Odometry, self.imu_cb)
    self.last_imu_angle = 0
    self.imu_angle = 0
    while not rospy.is_shutdown():
      rospy.sleep(0.3)
      rospy.loginfo("imu_angle:"+str((self.imu_angle)*180/3.1415926))


  def imu_cb(self, msg):
    with self.lock:
      angle = quat_to_angle(msg.pose.pose.orientation)
      self.imu_angle = angle
      self.imu_time = msg.header.stamp

def main():
  rospy.init_node('scan_to_angle1')
  CalibrateRobot1()


if __name__ == '__main__':
  main()
