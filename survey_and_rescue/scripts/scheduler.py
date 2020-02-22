#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from survey_and_rescue.msg import *
import json
from time import time
import math

class sr_scheduler():
	def __init__(self):
		#rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr',SRDroneStats,self.stats_callback)
		rospy.Subscriber('/alt_error', Float64, self.alt_error_callback)
		rospy.Subscriber('/pitch_error', Float64, self.pitch_error_callback)
		rospy.Subscriber('/roll_error', Float64, self.roll_error_callback)
		
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)
		self.decided_msg = SRInfo()
		self.decided_msg_prev=SRInfo()
		self.decided_msg.location=self.decided_msg_prev.location="D3"
		self.decided_msg.info=self.decided_msg_prev.info="BASE"
		self.beacons={}
		self.servicing=False
		self.info=["FOOD","MEDICINE","RESCUE"]
		self.time=time()
		self.food=0
		self.medicine=0
		self.stat=False
		self.error=[None,None,None]
		self.timer=0
		
	def detection_callback(self, msg):
		pass

	def alt_error_callback(self,msg):
		self.error[2]=msg.data
		
	def pitch_error_callback(self,msg):
		self.error[1]=msg.data
		
	def roll_error_callback(self,msg):
		self.error[0]=msg.data

	def serviced_callback(self,msg):
		if msg.location!='D3':
			self.beacons[msg.location][0]="OFF"
			self.beacons[msg.location][1]=None
		if(msg.location==self.decided_msg_prev.location):
			if self.decided_msg_prev.info=="RESCUE" and msg.info=="SUCCESS":
				self.decided_msg_prev.location="D3"
				self.decided_msg_prev.info="BASE"
				self.decision_pub.publish(self.decided_msg_prev)
				self.timer=0
				self.servicing=True
			else:
				self.servicing=False

	def shutdown_hook(self):
		pass

	def stats_callback(self,msg):
		self.food=msg.foodOnboard
		self.medicine=msg.medOnboard
		lit=[msg.currentLit.FOOD,msg.currentLit.MEDICINE,msg.currentLit.RESCUE]
		for i in range(3):
			for j in lit[i]:
				cell=chr(ord(j)/10+64)+str(ord(j)%10)
				try:
					if self.beacons[cell][0]!=self.info[i]:
						self.beacons[cell]=[self.info[i],time()-self.time]
				except KeyError:
					self.beacons[cell]=[self.info[i],time()-self.time]
					self.stat=True
					
def main(args):
	sched = sr_scheduler()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)	
	rate = rospy.Rate(20)
	while not sched.stat:
		pass
	while not rospy.is_shutdown():
		cur_col=ord(sched.decided_msg_prev.location[0])-64
		cur_row=int(sched.decided_msg_prev.location[1])
		priority=0
		for location,info in sched.beacons.items():
			if info[0]=="OFF" or location==sched.decided_msg_prev.location:
				continue
			col=ord(location[0])-64
			row=int(location[1])
			timespent=(time()-sched.time-info[1])+0.000001
			dist=math.sqrt( (cur_row-row)**2 + (cur_col-col)**2 )
			if info[0]=="FOOD" and timespent<=25:
				if priority < (10 * sched.food / (dist*timespent)):
					sched.decided_msg.location=location
					sched.decided_msg.info=info[0]
					priority=10 * sched.food / (dist*timespent)
											
			elif info[0]=="MEDICINE" and timespent<=25:
				if priority < (10 * sched.medicine / (dist*timespent)):
					sched.decided_msg.location=location
					sched.decided_msg.info=info[0]
					priority=10 * sched.medicine / (dist*timespent)
				
			elif info[0]=="RESCUE" and timespent<=3:
				if priority < (50 / (dist*timespent)):
					sched.decided_msg.location=location
					sched.decided_msg.info=info[0]
					priority=50 / (dist*timespent)
		if priority==0:
			sched.decided_msg.location="D3"
			sched.decided_msg.info="BASE"
			
		if not sched.servicing or (sched.decided_msg.info=="RESCUE" and sched.decided_msg_prev.info in ["FOOD","MEDICINE"] and sched.timer<1.5):
			sched.decision_pub.publish(sched.decided_msg)
			sched.decided_msg_prev.location=sched.decided_msg.location
			sched.decided_msg_prev.info=sched.decided_msg.info
			sched.timer=0
			sched.servicing=True
			
		elif -0.5<=sched.error[0]<=0.5 and -0.5<=sched.error[1]<=0.5 and -1<=sched.error[2]<=1:
			sched.timer=sched.timer+0.05
			
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
