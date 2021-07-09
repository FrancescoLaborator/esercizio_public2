#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Float64MultiArray

global xkh, ykh, tt
tt=[]
xkh=[]
ykh=[]

def read_callback(msg):
	global xkh, ykh, tt
	temp=msg.data 
	xkh.append(temp[0])
	ykh.append(temp[1])
	tt.append(temp[2])
#	EK_message=Float64MultiArray()
#	EK_message.data=[xkh[-1],ykh[-1],tt[-1]]
#	pub.publish(EK_message)

	
rospy.init_node('processamento')
#pub = rospy.Publisher('data_ekf', Float64MultiArray)
sub = rospy.Subscriber('data_ekf', Float64MultiArray, read_callback)
rospy.spin()
#file_uno = open("C:\\file_imu.txt", "w")
#contenuto=str(axr)+str(ayr)+str(tt_imu)
#file_uno.write(contenuto)
#file_uno.close() 

plt.figure()
plt.plot(xkh,ykh)
plt.ylabel('Kalman')
plt.show()

