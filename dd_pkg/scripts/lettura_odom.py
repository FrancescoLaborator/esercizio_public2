#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

global x, y
x=[]
y=[]

def read_callback(msg):
	global x,y
	x.append(msg.pose.pose.position.x)
	y.append(msg.pose.pose.position.y)

rospy.init_node('reader')
sub = rospy.Subscriber('odom', Odometry, read_callback)
rospy.spin()

plt.figure()
plt.plot(x,y)
plt.ylabel('ODOM')
plt.show()
