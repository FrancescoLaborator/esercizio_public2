#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import numpy as np
import matplotlib.pyplot as plt
import math
import pymap3d as pm
from std_msgs.msg import Float64MultiArray

global lat, lon, alt, tt, X, Y, pub
lat=[]
lon=[]
alt=[]
tt=[]
X=[]
Y=[]


def read_callback(msg):
	global lat,lon,alt, tt, X, Y, pub
	lat.append(msg.latitude)
	lon.append(msg.longitude)
	alt.append(msg.altitude)
	
	tt.append(msg.header.stamp.secs+msg.header.stamp.nsecs*(1e-9))
	print(tt[-1])
	
	y,x,z=pm.geodetic2enu(lat[-1],lon[-1],alt[-1],lat[0],lon[0],alt[0])

	gps_message=Float64MultiArray()
	gps_message.data=[x,-y,tt[-1]]
	pub.publish(gps_message)

	X.append(x)
	Y.append(-y)

rospy.init_node('reader_gps')
pub = rospy.Publisher('data_gps', Float64MultiArray)
sub = rospy.Subscriber('gps', NavSatFix, read_callback)
rospy.spin()


plt.figure()
plt.plot(X,Y)
plt.ylabel('GPS')
plt.show()
