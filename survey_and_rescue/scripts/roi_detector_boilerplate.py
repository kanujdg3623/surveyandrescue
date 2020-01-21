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

class sr_determine_rois():

	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect",Image,self.image_callback)
		self.img = None
		
		self.cells=[]
	# CV_Bridge acts as the middle layer to convert images streamed on rostopics to a format that is compatible with OpenCV
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)


	'''You will need to implement an image processing algorithm to detect the Regions of Interest (RoIs)
	The standard process flow is:
	i)		Standard pre-processing on the input image (de-noising, smoothing etc.)
	ii)		Contour Detection
	iii)	Finding contours that are square, polygons with 4 vertices
	iv)		Implementing a threshold on their area so that only contours that are the size of the cells remain'''
	def detect_rois(self):
		# Add your Code here
		# You may add additional function parameters
		# cv2.imshow("Detected ROIs", img_copy) #A copy is chosen because self.img will be continuously changing due to the callback function
		# cv2.waitKey(100)
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
			self.cells.insert(0,[x,y,w,h])
			img_copy = cv2.rectangle(self.img,(x,y),(x+w,y+h),(255,255,255),2)
			cv2.putText(self.img,str(i),(x+40,y+65), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
			i=i+1
		cv2.imshow("Detected ROIs", img_copy);
		if(cv2.waitKey(100)==ord('q')):
			cv2.destroyAllWindows()
			exit()


	'''	Please understand the order in which OpenCV detects the contours.
		Before saving the files you may sort them, so that their order corresponds will the cell order
		This will help you greatly in the next part. '''
	def sort_rois(self):
		# Add your Code here
		for i in range(0,6):
			self.cells.append(sorted(self.cells[6*i:6*i+6], key=lambda k:[k[0]]))
		del self.cells[0:36]
			

		

	def query_yes_no(self, question, default=None):
		"""Ask a yes/no question via raw_input() and return their answer.

		"question" is a string that is presented to the user.
		"default" is the presumed answer if the user just hits <Enter>.
		It must be "yes" (the default), "no" or None (meaning
		an answer is required of the user).

		The "answer" return value is True for "yes" or False for "no".
		"""
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

	'''	You may save the list using anymethod you desire
	 	The most starightforward way of doing so is staright away pickling the objects
		You could also save a dictionary as a json file, or, if you are using numpy you could use the np.save functionality
		Refer to the internet to find out more '''
	def save_rois(self):
		cell_dict={}
		col=('A','B','C','D','E','F')
		l=0
		for row in self.cells:
			k=0
			for cell in row:
				cell_dict[col[k]+str(l+1)]=cell
				k=k+1
			l=l+1
		with open("roi.json", mode='w') as outfile:
			json.dump(cell_dict, outfile)
		print("Recording Successful")
					

	#You may optionally implement this to display the image as it is displayed in the Figure given in the Problem Statement
	def draw_cell_names(self, img):
		#Add your code here
		pass

def main(args):
	#Sample process flow
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		while not rospy.is_shutdown():
			if r.img is not None:
				r.detect_rois()
				if len(r.cells)!=36:
					new_thresh_flag = r.query_yes_no("36 cells were not detected, do you want to change ##Enter tweaks, this is not necessary##?")
					if(new_thresh_flag):
						#Change settings as per your desire
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
						#Change more settings
						pass
		# r.draw_cell_names(r.img) # Again, this is optional
		
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
