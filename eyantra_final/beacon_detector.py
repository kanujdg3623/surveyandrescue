#!/usr/bin/env python

'''
	* Team ID:		4438
	* Author List:	Aayushi Gautam, Dikshita Jain, Kanuj Das Gupta, Mohit Soni
	* FileName:		beacon_detector.py
	* Theme:		Survey And Rescue
	* Functions:	loadrois(), image_callback(), serviced_callback(), detect_colour_contour_centers() 
'''

#============================================================================================================
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import random
import pickle
import imutils
import copy
import os
from time import time

#============================================================================================================

class sr_determine_colors():
	"""To determine colours using image processing"""
	def __init__(self):
		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		#self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
 		self.img=None
 		self.rect_list=None
 		self.color_bound=np.array([[255,0,0],[0,255,0],[0,0,255]]) #Setting colour bound of BGR
 		self.info=("MEDICINE","FOOD","RESCUE")	#defining meaning of each colour
 		self.beacons={}
 		self.time=time()
 		rospy.sleep(0.5)

 #============================================================================================================
	"""
	* Function Name: load_rois
	* Input: file path - rect_info.pkl
	* Logic: Assigning rect list information of each roi from rect_info.pkl which is created by roi_detector.py
	"""
	def load_rois(self, file_path = '/home/asus/rect_info.pkl'):
		try:
			with open(file_path, 'rb') as input:
   				self.rect_list = pickle.load(input)
		except IOError, ValueError:
			print("File doesn't exist or is corrupted")

#==============================================================================================================

 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 		except CvBridgeError as e:
 			print(e)

#==============================================================================================================
	
	def detect_color_contour_centers(self, color_str):
		#thresholding self.img and assign it to img_copy
		img_copy=cv2.threshold(self.img, 200, 255, cv2.THRESH_BINARY)[1]  
		k=0
		for i in self.rect_list:
			for j in i:
				#assigning the roi by slicing the threshold image
				cell=img_copy[ j[1]:j[1]+j[3] ,j[0]:j[0]+j[2] ]	
				location=chr(k%6+65)+str(k/6+1)  #setting location as A1,A2 etc.
				
				#switching off the beacon to avoid redundancy
				if self.beacons.get(location) :
					if (self.beacons.get(location)[0]=="RESCUE" and (time()-self.time-self.beacons.get(location)[1])>=10.5) or (self.beacons.get(location)[0] in ["FOOD","MEDICINE"] and (time()-self.time-self.beacons.get(location)[1])>=30.5):
						self.beacons[location][0]="OFF"
						self.beacons[location][1]=None

				for l in range(0,3):
					#returns a mask, specifying which pixels fall into  specified upper and lower range
					mask=cv2.inRange(cell,self.color_bound[l],self.color_bound[l])
					if np.sum(mask==255)>100:
						#Publishing the location and info of the corresponding color detected
						try:
					     		if  self.beacons[location]!=self.info[l]:
								self.detect_info_msg.location=location
								self.detect_info_msg.info=self.info[l]
								self.detect_pub.publish(self.detect_info_msg)
								self.beacons[location]=self.info[l]
						except KeyError:
							self.detect_info_msg.location=location
							self.detect_info_msg.info=self.info[l]
							self.detect_pub.publish(self.detect_info_msg)
							self.beacons[location]=self.info[l]
						break
				k=k+1

#==============================================================================================================

def main(args):	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		rate = rospy.Rate(20)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.detect_color_contour_centers(None)
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


