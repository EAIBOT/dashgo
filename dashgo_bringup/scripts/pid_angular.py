#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign
import PyKDL
from math import pi


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class CalibrateAngular():
    def __init__(self):
        # Give the node a name
        rospy.init_node('calibrate_angular', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)
        
        # The test angle is 360 degrees
        self.test_angle = radians(rospy.get_param('~test_angle', 90.0))

        self.speed = rospy.get_param('~speed', 0.5) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
 
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        #self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom_combined')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        
        reverse = 1

        preOutput = 0
        iterm = 0
        goal_angle = 3.1415926
        current_angle = 0
        kp = 0
        ki = 0
        kd = 0
        MAX_CMD_Z = 0.4
        
        while not rospy.is_shutdown():       
            self.current_angle = self.get_odom_angle()
            move_cmd = Twist()
            move_cmd.angular.z = self.doPID(3.14)
            rospy.loginfo("3")
            self.cmd_vel.publish(move_cmd)
            r.sleep()
 

    def doPID(self,my_goal_angle):
        rospy.loginfo("3.1")
        rospy.loginfo("goal_angle:"+str(my_goal_angle))
        perror = my_goal_angle - self.current_angle
        rospy.loginfo("4")
        output = self.kp*perror + self.iterm
        output = output + self.preoutput
        rospy.loginfo("5")
        if output >= self.MAX_CMD_Z:
            output = self.MAX_CMD_Z
        elif output <= -1*self.MAX_CMD_Z:
            output = -1*self.MAX_CMD_Z
        else:
            self.iterm = self.iterm+self.ki*perror
        rospy.loginfo("6")
        self.preoutput = output
        return output

        
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))
            
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        CalibrateAngular()
    except:
        rospy.loginfo("Calibration terminated.")

