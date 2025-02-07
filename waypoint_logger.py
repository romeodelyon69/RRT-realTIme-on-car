#!/usr/bin/env python3
import rospy
import numpy as np
import atexit
import tf
import os
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

home = expanduser('~')
file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

def save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
    if data.twist.twist.linear.x>0.:
        print(data.twist.twist.linear.x)

    file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     euler[2],
                                     speed))

def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    if os.environ['ROS_MASTER_URI']=="http://"+os.environ['ROS_HOSTNAME']+":11311":
        print("local")
        sub_topic = 'odom'
    else:
        print("remote")
        sub_topic = 'pf/pose/odom'
    print("sub_topic chosen : "+sub_topic)
    rospy.Subscriber(sub_topic, Odometry, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
