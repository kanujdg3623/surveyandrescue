#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
import json
from rospy_message_converter import message_converter
from time import time
from math import floor

class sr_scheduler():
	def __init__(self):
		#rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr',SRDroneStats,self.stats_callback)
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)
		self.decided_msg = SRInfo()
		self.beacons={}
		self.servicing=False
		self.info=["FOOD","MEDICINE","RESCUE"]
		self.time=time()
		
	def detection_callback(self, msg):
		pass

	def serviced_callback(self,msg):
		self.beacons[msg.location]="OFF"
		if(msg.location==self.decided_msg.location):
			self.servicing=False

	def shutdown_hook(self):
		pass

	def stats_callback(self,msg):
		lit=[msg.currentLit.FOOD,msg.currentLit.MEDICINE,msg.currentLit.RESCUE]
		for i in range(3):
			for j in lit[i]:
				cell=chr(ord(j)/10+64)+str(ord(j)%10)
				try:
					if self.beacons[cell][0]!=self.info[i]:
						self.beacons[cell]=[self.info[i],floor(time()-self.time)]
						self.detection_callback([cell,self.info[i]])
				except KeyError:
					self.beacons[cell]=[self.info[i],floor(time()-self.time)]
					self.detection_callback([cell,self.info[i]])
					
def main(args):
	sched = sr_scheduler()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)	
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
