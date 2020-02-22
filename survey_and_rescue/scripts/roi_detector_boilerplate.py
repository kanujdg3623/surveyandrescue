#!/usr/bin/env python

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle
import imutils
import copy
import numpy as np
import itertools
import pickle

class sr_determine_rois():
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect",Image,self.image_callback)
		self.img = None		
		self.cells=[]
		
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)

	def detect_rois(self):
		blur = cv2.GaussianBlur(self.img[120:900,260:],(5,5),0)
		ret,thresh=cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
		img_copy, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		contours.pop(0)
		i=0 
		del self.cells[:]
		for cnt in contours:
			cnt=cv2.convexHull(cnt)
			x,y,w,h = cv2.boundingRect(cnt)
			x=x+260
			y=y+120
			if 60<w<80 and 60<h<80:
				self.cells.insert(0,[x,y,w,h])
				img_copy = cv2.rectangle(self.img,(x,y),(x+w,y+h),(255,255,255),2)
				cv2.putText(self.img,str(i),(x+40,y+65), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
			i=i+1
		cv2.imshow("Detected ROIs", img_copy);
		if(cv2.waitKey(100)==ord('q')):
			cv2.destroyAllWindows()
			exit()

	def sort_rois(self):
		for i in range(0,6):
			self.cells.append(sorted(self.cells[6*i:6*i+6], key=lambda k:[k[0]]))
		del self.cells[0:36]		

	def query_yes_no(self, question, default=None):
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None:
			prompt = " [Y/N]:\t"
		elif default == "yes":
			prompt = " [Y/N]:\t"
		elif default == "no":
			prompt = " [Y/N]:\t"
		else:
			raise ValueError("Invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = raw_input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("\nPlease respond with 'yes' or 'no' ""(or 'y' or 'n').\n")

	def save_rois(self):
		outfile=open('rect_info.pkl','wb')
		pickle.dump(self.cells,outfile)
		outfile.close()
		print("Recording Successful")
					
	def draw_cell_names(self, img):
		with open("roi.json") as file:
			data=json.load(file)
		for i in ('A','B','C','D','E','F'):
			for j in ('1','2','3','4','5','6'):
				x,y,w,h=data.get(i+j)
				img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,255),2)
				cv2.putText(img,str(i+j),(x+40,y+65), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
		cv2.imshow(" ROIs", img);
		cv2.waitKey(100)

def main(args):
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		while not rospy.is_shutdown():
			if r.img is not None:
				r.detect_rois()
				if len(r.cells)!=36:
					new_thresh_flag = r.query_yes_no("36 cells were not detected, do you want to change?")
					if(new_thresh_flag):
						pass
					else:
						cv2.destroyAllWindows()
						break
				else:	
					satis_flag = r.query_yes_no("Are you satisfied with the currently detected ROIs?")
					if(satis_flag):
						r.sort_rois()
						r.save_rois()
						cv2.destroyAllWindows()
						break
					else:
						pass	
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
