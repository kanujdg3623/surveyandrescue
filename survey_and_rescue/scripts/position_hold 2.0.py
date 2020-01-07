#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error			/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error			/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import sys, select, termios, tty
import json

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		with open("/home/"+pc_username+"/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json","r") as read_file:
			data=json.load(read_file)
		self.setpoint=[]
		for i in ('A','B','C','D','E','F'):
			for j in ('1','2','3','4','5','6'):
				self.setpoint.append(data.get(i+j))
		print(self.setpoint)
		
		# whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [15,15,50]
		self.Ki = [0.1,0.05,0.05]
		self.Kd = [225,274,290]


		#-----------------------Add other required variables for pid here ----------------------------------------------

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, throttle]
		# Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		# self.min_values = [1000,1000,1000] corresponding to [roll, pitch, throttle]
		self.prev_value=[0,0,0]
		self.min_values=[1000,1000,1000]
		self.max_values=[2000,2000,2000]
		self.error=[0,0,0]
		self.Pterm=[0,0,0]
		self.Iterm=[0,0,0]
		self.Dterm=[0,0,0]
		self.out=[0,0,0]
		self.visit=0

		# You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=5)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)






		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)



		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1400
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def land(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1100
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(2)
	
	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.01 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.001
		self.Kd[2] = alt.Kd * 0.1

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.01
		self.Ki[1] = pitch.Ki * 0.001
		self.Kd[1] = pitch.Kd * 0.1

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.01
		self.Ki[0] = roll.Ki * 0.001
		self.Kd[0] = roll.Kd * 0.1






	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------
	
	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
		if(self.visit==30):
			print(self.setpoint[30])
		for i in range(3):
			self.error[i]=self.drone_position[i]-self.setpoint[self.visit][i]

			self.Pterm[i] = self.error[i]
			self.Dterm[i] = self.drone_position[i] - self.prev_value[i]
			#if self.error[i]*(self.prev_value[i]-self.setpoint[self.visit][i])<0:
			#	self.Iterm[i]=0
			self.Iterm[i] = self.Iterm[i] + self.error[i]

			self.out[i] = self.Pterm[i] * self.Kp[i] + self.Iterm[i] * self.Ki[i] + self.Dterm[i] * self.Kd[i]		

			self.prev_value[i]=self.drone_position[i]

		self.cmd.rcRoll=1500 - self.out[0]
		self.cmd.rcPitch=1500 + self.out[1]
		self.cmd.rcThrottle=1500 + self.out[2]

		commands=[self.cmd.rcRoll,self.cmd.rcPitch,self.cmd.rcThrottle]
		i=0
		for rc in commands:
			if rc > self.max_values[i]:
				rc = self.max_values[i]
			if rc < self.min_values[i]:
				rc = self.min_values[i]
			i=i+1
	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		self.roll_error_pub.publish(self.error[0])
		self.pitch_error_pub.publish(self.error[1])
		self.alt_error_pub.publish(self.error[2])
			

def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	e_drone = Edrone()
	r = rospy.Rate(10) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
		key=getKey()
		if -1<e_drone.error[0]<1 and -1<e_drone.error[1]<1 and -1<e_drone.error[2]<1:
			if e_drone.visit==35:
				break
			time=rospy.get_time()
			while rospy.get_time()-time<3:
				e_drone.pid()
				r.sleep()
				key=getKey()
				if key== '\x03':
					break
			e_drone.visit=e_drone.visit+1
		if key== '\x03':
			break
	e_drone.land()
	e_drone.disarm()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
