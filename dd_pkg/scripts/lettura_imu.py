#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Float64MultiArray

global ax, ay, tt, w, yaw, axr, ayr, pub
ax=[]
ay=[]
tt=[]
w=[]
yaw=[]
axr=[]
ayr=[]

def read_callback(msg):
	global ax,ay, tt, w, yaw, axr, ayr, pub 
	ax.append(msg.linear_acceleration.x)
	ay.append(msg.linear_acceleration.y)
	print(ay[-1])
	tt.append(msg.header.stamp.secs+msg.header.stamp.nsecs*(1e-9))
	print(tt[-1])
	w.append(msg.angular_velocity.z)
	if len(tt)>1:
		Tc=tt[-1]-tt[-2]
	else:
		Tc=0
	yaw.append(yaw[-1]+Tc*w[-1])
	axr.append(ax[-1]*math.cos(yaw[-1])-ay[-1]*math.sin(yaw[-1]))
	ayr.append(ax[-1]*math.sin(yaw[-1])+ay[-1]*math.cos(yaw[-1]))

	imu_message=Float64MultiArray()
	imu_message.data=[axr[-1],ayr[-1],yaw[-1],tt[-1],w[-1]]
	pub.publish(imu_message)

rospy.init_node('reader_imu')
yaw.append(0)
pub = rospy.Publisher('data_imu', Float64MultiArray)
sub = rospy.Subscriber('imu', Imu, read_callback)
rospy.spin()

os=tt[0]
for k in range(len(tt)):
	tt[k]=tt[k]-os
	
vx=[]
vx.append(0)
vy=[]
vy.append(0)
for k in range(len(tt)):
	if k!=0:
		Tc=tt[k]-tt[k-1]
		vx.append(vx[-1]+Tc*axr[k])
		vy.append(vy[-1]+Tc*ayr[k])
	
x=[]
x.append(0)
y=[]
y.append(0)
for k in range(len(tt)):
	if k!=0:
		Tc=tt[k]-tt[k-1]
		x.append(x[-1]+Tc*vx[k])
		y.append(y[-1]+Tc*vy[k])

plt.figure()
plt.plot(x,y)
plt.ylabel('IMU')
plt.show()

plt.figure()
plt.plot(yaw)
plt.show()
