'''
	* Team ID     :		4438
	* Author List : 	Aayushi Gautam, Dikshita Jain, Kanuj Das Gupta, Mohit Soni
	* FileName    :		schedular.py
	* Theme       :		Survey And Rescue
	* Functions   :		detection_callback(), alt_error_callback(), pitch_error_callback(), roll_error_callback(), serviced_callback(), stats_callback(), main()
'''
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

#=========================================================================================================================================================================================================

class sr_scheduler():
	'''to determine beacon to be serviced using appropriate algorithm'''
	def __init__(self):		
		#Subscribing to /detection_info /serviced_info /stats_sr /alt_error /pitch_error /roll_error
		rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr',SRDroneStats,self.stats_callback)
		rospy.Subscriber('/alt_error', Float64, self.alt_error_callback)
		rospy.Subscriber('/pitch_error', Float64, self.pitch_error_callback)
		rospy.Subscriber('/roll_error', Float64, self.roll_error_callback)
		
		#Publishing /decision_info
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)

		self.decided_msg = SRInfo()                                             #stores msg details that is constantly updating      
		self.decided_msg_pub=SRInfo()                                           #stores msg details that is published latest  
		self.decided_msg.location=self.decided_msg_pub.location="E4"            #initializing both msgs location with base location
		self.decided_msg.info=self.decided_msg_pub.info="BASE"			#initializing both msgs info with "BASE"
		
		self.beacons={}								#dictionary that consist of key as beacon location & value as list of msg info whether it is "FOOD" or 
											#"MEDICINE" OR "RESCUE" AND time stamp with respect to bench mark(self.time) i.e. time period from 
											#which led configration  starts to glow
											
		self.servicing=False							#flag variable that stores whether a beacon is currently serviced or not
		
		self.time=time()							#bench mark that stores time at start of the run for further time variables
		
		self.food=0								#stores food_on_board & is initially assigned to zero
		self.medicine=0								#stores med_on_board & is initially assigned to zero
		
		self.stat=False								#flag variable of leds status for there start in glow and thus prevents from any drone movement before led lit 
		
		self.error=[None,None,None]						#contain error values of roll, pitch and altitude accordingly
		
		self.timer=0								#acts as cumulative counter 
		
		self.rescue=False							#flag variable to check whether rescue is achieved or not

#=========================================================================================================================================================================================================

'''
	*Function Name   :		detection_callback
	*Input           :		self-> reference to the object of class sr_scheduler, msg-> msg of type SRInfo
	*Logic           :		updates self.beacons upon led detection and sets self.stat to True		
'''
		
	def detection_callback(self, msg):
		self.beacons[msg.location]=[msg.info,time()-self.time]
		self.stat=True

#=========================================================================================================================================================================================================

'''
	*Function Name   :		alt_error_callback, pitch_error_callback, roll_error_callback
	*Input           :		self-> reference to the object of class sr_scheduler, msg-> msg of type Float64
	*Logic           :		sets altitute, pitch and roll error accordingly in self.error		
'''

	def alt_error_callback(self,msg):
		self.error[2]=msg.data
		
	def pitch_error_callback(self,msg):
		self.error[1]=msg.data
		
	def roll_error_callback(self,msg):
		self.error[0]=msg.data

#=========================================================================================================================================================================================================

'''
	*Function Name   :		serviced_callback
	*Input           :		self-> reference to the object of class sr_scheduler, msg-> msg of type SRInfo 
	*Logic           :		looks over few conditions after the msg is being serviced 		
'''

	def serviced_callback(self,msg):
		if msg.location!='E4':							        #update self.beacons[location] -> info to "OFF"  and error to "None" after it is serviced except "E4"
			self.beacons[msg.location][0]="OFF"
			self.beacons[msg.location][1]=None
		elif msg.location == 'E4' and self.rescue:					#else check is made if previous service was rescue and currently drone is on 'E4' and therefore
			self.rescue=False							#initialize self.rescue to 'False' because now rescue is done
			
		if(msg.location==self.decided_msg_pub.location):				#comparing msg being serviced and msg being published
			if self.decided_msg_pub.info=="RESCUE" and msg.info=="SUCCESS":		#direct drone to base after the rescue is being serviced	
				self.decided_msg_pub.location="E4"
				self.decided_msg_pub.info="BASE"
				self.decision_pub.publish(self.decided_msg_pub)		 		
				self.timer=0							#each time the msg is serviced, timer is set to zero	
				self.rescue=self.servicing=True					#set flags for rescue being published and for successfully serviced msg info rescue
			else:
				self.servicing=False						#set flag after the service is done whether 'SUCCESS' or 'FAILURE'
	  	if msg.info == 'END':								#exit if 'END' msg info encountered									
	  		exit()			

#=========================================================================================================================================================================================================

	def shutdown_hook(self):
		pass

#=========================================================================================================================================================================================================

'''
	*Function Name   :		stats_callback
	*Input           :		self-> reference to the object of class sr_scheduler, msg-> msg of type SRDroneStats 
	*Logic           :		sets self.food & self.medicine		
'''

	def stats_callback(self,msg):
		self.food=msg.foodOnboard
		self.medicine=msg.medOnboard

#=========================================================================================================================================================================================================

'''
	*Function Name   :		main
	*Input           :		args-> sys.argv 
	*Logic           :		algorithm for drone to service under provided configuration		
'''
					
def main(args):
	sched = sr_scheduler()	
	rate = rospy.Rate(20)                                                           #specifies rate in Hz
	while not sched.stat:   						  	#loops until no led is lit	
		pass
	while not rospy.is_shutdown():
		cur_col=ord(sched.decided_msg_pub.location[0])-64			#stores current col value in integer example 'A' as 1	
		cur_row=int(sched.decided_msg_pub.location[1])				#stores current row value
		priority=0								#select which cell to service based on distance from current position, marks for service and food or med on board
		for location,info in sched.beacons.items():				
			if info[0]=="OFF" or location==sched.decided_msg_pub.location:	#if info is off or location is already published go on reading the dictionary	
				continue
			col=ord(location[0])-64						#else get col 
			row=int(location[1])						#get row	
			timespent=(time()-sched.time-info[1])				#calculate time of how long led is being glowing
			dist=math.sqrt( (cur_row-row)**2 + (cur_col-col)**2 )		#calculate distance between current cell and cell which might be published

'''			
			if timespent exceeds 26 there is no need to publish as we need to service over food for 3 sec
			comparing the priority to get maximum one
			assign decided msg details
			set priority
'''
			
			if info[0]=="FOOD" and timespent<=26:				
				if priority < (10 * sched.food / dist):			
					sched.decided_msg.location=location		
					sched.decided_msg.info=info[0]
					priority=10 * sched.food / dist               \
'''
			if timespent exceeds 26 there is no need to publish as we need to service over food for 3 sec
			comparing the priority to get maximum one
			assign decided msg details
			set priority
'''																	
			elif info[0]=="MEDICINE" and timespent<=26:
				if priority < (10 * sched.medicine / dist):
					sched.decided_msg.location=location
					sched.decided_msg.info=info[0]
					priority=10 * sched.medicine / dist           \
'''
			if timespent exceeds 4 there is no need to publish as we need to service over food for 5 sec
			comparing the priority to get maximum one
			assign decided msg details
			set priority
'''				
			elif info[0]=="RESCUE" and timespent<=4:
				if priority < (50 / dist):
					sched.decided_msg.location=location
					sched.decided_msg.info=info[0]
					priority=50 / dist
		if priority==0:									    #if priority is still zero,we might go to base				
			sched.decided_msg.location="E4"
			sched.decided_msg.info="BASE"

'''
		there are 3 conditions combined in next if statement(here self.timer comes in use and it is incremented in line 223):
			1. if servicing in false, therefore we need to publish decided msg
			2. if rescue comes in between 
				i.   we have decided msg as rescue
				ii.  published msg is food or medicine
				iii. if drone has spent less than 1.5 sec on food or med it will move to service rescue otherwise not
			3. if priority is zero for few seconds only and we can make a choice to service another msg if detected within selected time limits
				i.   we have published msg as base
				ii.  we didn't had rescue,therefore it is not must to service base
				iii. decided msg info is not base
				iv.  if drone has spent less than 2.5 sec on base 	
'''
			
		if not sched.servicing or (sched.decided_msg.info=="RESCUE" and sched.decided_msg_pub.info in ["FOOD","MEDICINE"] and sched.timer<=1.5) or (sched.decided_msg_pub.info=="BASE" and not sched.rescue and sched.decided_msg.info!="BASE" and sched.timer<=2.5) :							
   			sched.decision_pub.publish(sched.decided_msg)
			sched.decided_msg_pub.location=sched.decided_msg.location
			sched.decided_msg_pub.info=sched.decided_msg.info
			sched.timer=0								    #each time the msg is serviced, timer is set to zero	
			sched.servicing=True							    #set flags for successfully serviced msg 
			
		elif -1<=sched.error[0]<=1 and -1<=sched.error[1]<=1 and -1<=sched.error[2]<=1:	    #incrementing the timer if all three error are between -1 to 1 thus fitting the cell dimensions range
			sched.timer=sched.timer+0.05			
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
