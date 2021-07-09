#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Float64MultiArray

global axr, ayr, tt_imu, tt_gps, pub, yaw, xkh, ykh, vxkh, vykh, x, y, P, tt, w, yawkh
tt_imu=[]
tt_gps=[]
tt=[]
yaw=[]
axr=[]
ayr=[]
xkh=[]
ykh=[]
vxkh=[]
vykh=[]
w=[]
yawkh=[]
xkh.append(0)
ykh.append(0)
x=[]
y=[]
P=[
[1,0,0,0,0],
[0,1,0,0,0],
[0,0,1,0,0],
[0,0,0,1,0],
[0,0,0,0,0.1]
]

def read_callback(msg):
	global yaw, axr, ayr, pub, xkh, ykh, vxkh, vykh, P, tt_imu, tt, w, yawkh
	temp=msg.data 
	axr.append(temp[0])
	ayr.append(temp[1])
	tt_imu.append(temp[3])
	tt.append(tt_imu[-1])
	w.append(temp[4])
	print(tt_imu[-1])
	V1=np.array([
	[1,0,0,0,0],
	[0,1,0,0,0],
	[0,0,1,0,0],
	[0,0,0,1,0],
	[0,0,0,0,0.1],
	])
	if len(tt_imu)>1:
		Tc=tt_imu[-1]-tt_imu[-2]		
		vxkh.append(vxkh[-1]+Tc*axr[-1])
		vykh.append(vykh[-1]+Tc*ayr[-1])
		v=np.sqrt(pow(vxkh[-1],2)+pow(vykh[-1],2))
		if w[-1]==0:
			xkh.append(xkh[-1]+v*Tc*np.cos(yawkh[-1]))
			ykh.append(ykh[-1]+v*Tc*np.sin(yawkh[-1]))
			yawkh.append(yawkh[-1])
			F=[
			[1,Tc*np.cos(yawkh[-1])*(1/v)*vxkh[-1],0,Tc*np.cos(yawkh[-1])*(1/v)*vykh[-1],-v*Tc*np.sin(yawkh[-1])],
			[0,1,0,0,0],
			[0,Tc*np.sin(yawkh[-1])*(1/v)*vxkh[-1],1,Tc*np.sin(yawkh[-1])*(1/v)*vykh[-1],v*Tc*np.cos(yawkh[-1])],
			[0,0,0,1,0],
			[0,0,0,0,1]
			]

		else :
			xkh.append(xkh[-1]+v*(np.sin(yawkh[-1]+w[-1]*Tc)-np.sin(yawkh[-1]))/w[-1])
			ykh.append(ykh[-1]+v*(np.cos(yawkh[-1]+w[-1]*Tc)-np.cos(yawkh[-1]))/w[-1])
			yawkh.append(yawkh[-1]+w[-1]*Tc)
			F=[
			[1,((np.sin(yawkh[-1]+w[-1]*Tc)-np.sin(yawkh[-1]))/w[-1])*(1/v)*vxkh[-1],0,((np.sin(yawkh[-1]+w[-1]*Tc)-np.sin(yawkh[-1]))/w[-1])*(1/v)*vykh[-1],v*(np.cos(yawkh[-1]+w[-1]*Tc)-np.cos(yawkh[-1]))/w[-1]],
			[0,1,0,0,0],
			[0,((np.cos(yawkh[-1]+w[-1]*Tc)-np.cos(yawkh[-1]))/w[-1])*(1/v)*vxkh[-1],1,((np.cos(yawkh[-1]+w[-1]*Tc)-np.cos(yawkh[-1]))/w[-1])*1/v*vykh[-1],v*(-np.sin(yawkh[-1]+w[-1]*Tc)+np.sin(yawkh[-1]))/w[-1]],
			[0,0,0,1,0],
			[0,0,0,0,1]
			]
		P=np.dot(np.dot(F,P),np.transpose(F))+V1
		
	else :
		Tc=0
		vxkh.append(0)
		vykh.append(0)
		yawkh.append(0)
	EK_message=Float64MultiArray()
	EK_message.data=[xkh[-1],ykh[-1],tt[-1]]
	pub.publish(EK_message)

def read_callback2(msg):
	global x,y, P, xkh, ykh, vxkh, vykh, tt_gps, tt
	temp=msg.data 
	x.append(temp[0])
	y.append(temp[1])
	tt_gps.append(temp[2])
	tt.append(tt_gps[-1])
	#print(tt[-1])
	V2=[
	[1,0],
	[0,1]
	]
	H=[
	[1, 0, 0, 0, 0],
	[0, 0, 1, 0, 0]
	]
	cov=np.dot(np.dot(H,P),np.transpose(H))+V2
	print("cov")
	print(cov)
	K=np.dot(np.dot(P,np.transpose(H)),np.linalg.inv(cov))
	print("K")
	print(K)
	e=[0,0]
	e[0]=-xkh[-1]+temp[0]
	e[1]=-ykh[-1]+temp[1]
	print("e")
	print(e)
	temp2=np.dot(K,e)
	print("Ke")
	print(temp2)
	xkh.append(xkh[-1]+temp2[0])
	vxkh.append(vxkh[-1]+temp2[1])
	ykh.append(ykh[-1]+temp2[2])
	vykh.append(vykh[-1]+temp2[3])
	yawkh.append(yawkh[-1]+temp2[4])
	P=P-np.dot(np.dot(K,H),P)
	print("P")
	print(P)
	EK_message=Float64MultiArray()
	EK_message.data=[xkh[-1],ykh[-1],tt[-1]]
	pub.publish(EK_message)


	
rospy.init_node('kalman_ext')
pub = rospy.Publisher('data_ekf', Float64MultiArray)
sub = rospy.Subscriber('data_imu', Float64MultiArray, read_callback)
sub2= rospy.Subscriber('data_gps', Float64MultiArray, read_callback2)
rospy.spin()
file_uno = open("C:\\file_imu.txt", "w")
contenuto=str(axr)+str(ayr)+str(tt_imu)+str(w)
file_uno.write(contenuto)
file_uno.close() 

file_due = open("C:\\file_gps.txt", "w")
contenuto=str(x)+str(y)+str(tt_gps)
file_due.write(contenuto)
file_due.close() 

plt.figure()
plt.plot(xkh,ykh)
plt.ylabel('Kalman')
plt.show()

