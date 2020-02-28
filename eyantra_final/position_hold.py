#!/usr/bin/env python
'''
	* Team ID:		4438
	* Author List:	Aayushi Gautam, Dikshita Jain, Kanuj Das Gupta, Mohit Soni
	* FileName:		Position_hold.py
	* Theme:		Survey And Rescue
	* Functions:	getKey(), disarm(), arm(), land(), whycon_callback(), decision_callback(), pid()
'''

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
from survey_and_rescue.msg import *

#================================================================================================================

"""
	Function Name: getKey()
	Logic: Inbuilt function taken from drone_teleop.py(edrone_client)
"""
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
#================================================================================================================

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		rospy.init_node('drone_control')	# initializing ros node with name drone_control
		
		# This corresponds to the current position of drone which is updated each time in whycon callback function
		self.drone_position = [0.0,0.0,0.0]
		
		self.cell_coords={}
		with open('/home/asus/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json','r') as readfile:
			self.cell_coords=json.load(readfile)
		self.setpoint=self.cell_coords.get("E4")
		self.pid_setpoint=[0,0,0]
		
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
		
		#initial setting of Kp, Kd and ki for [roll, pitch, throttle].
		#=======================#
		self.Kp = [10,10,50]	#
		self.Ki1 = [4.5,4.5,2]	#
		self.Ki2 = [2,2,0.5]	#
		self.Kd = [250,250,250]	#
		#=======================#
		
		self.prev_value=[0,0,0]
		self.min_values=[1000,1000,1000]
		self.max_values=[2000,2000,2000]
		
		self.error=[0,0,0]
		self.Pterm=[0,0,0]
		self.Iterm1=[0,0,0]	#Iterm for error in range -1.5 to +1.5
		self.Iterm2=[0,0,0] #Iterm for error except the above range
		self.Dterm=[0,0,0]
		self.out=[0,0,0]

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=5)
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		
		# Subscribing to /whycon/poses /decision_info
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/decision_info',SRInfo,self.decision_callback)
				
		self.arm() # ARMING THE DRONE

#================================================================================================================

	"""
	* Function Name: disarm(), arm(), land()
	* Input: Reference to the object of class edrone
	* Logic: Setting the values of roll, pitch, throttle in order to arm, disarm and land the drone.
	"""
	def disarm(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)
		
	def arm(self):
		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def land(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1100
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

#================================================================================================================
	"""
	* Function Name: whycon_callback()
	* Input: ros msg
	* Logic: Set the drone position using ros msgs
	"""
	def whycon_callback(self,msg):
		self.drone_position = msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z
		
#================================================================================================================
	"""
	* Function Name: decision_callback
	* Input: ros msg
	* Logic: Search the value of msg.location in cell_coords.json file and assign it as a set point.
	"""
	def decision_callback(self,msg):
		self.out=[0,0,0]
		self.Iterm1=[0,0,0]
		self.Iterm2=[0,0,0]
		self.setpoint=self.cell_coords.get(msg.location)

#================================================================================================================

	'''
	* Function Name: pid()
	* Input: reference to the object of class edrone
	* Logic: consists of steps mentioned below
	 	1. Compute error in each axis.
		2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
		3. Calculate the pid output required for each axis. 
		4. Reduce or add this computed output value on the avg value ie 1500.
		5. Don't run the pid continously. Run the pid only at the a sample time.
		6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. 
		7. Update previous errors.
		8. When the error is changing from positive to negative or vice-versa make Iterm=0.
	'''


	def pid(self):
		for i in range(3):
			self.error[i]=self.drone_position[i]-self.setpoint[i]
			if i<2:
				#limiting the error to avoid overshooting(for roll and pitch)
				self.error[i]=min(3,max(-3,self.error[i])) 
			
			self.Pterm[i] = self.error[i]
			self.Dterm[i] = self.drone_position[i] - self.prev_value[i]
			
			if -1.5<=self.error[i]<=1.5:
				
				if self.error[i]*(self.prev_value[i]-self.setpoint[i])<=0:
					self.Iterm1[i]=self.Iterm2[i]=0
	#using two different values of Iterm in order to stabalize the drone in two different error ranges
				self.out[i]=self.Iterm1[i] = self.Iterm1[i] + self.error[i]
			
			else:
				self.out[i]=self.Iterm2[i] = self.Iterm2[i] + self.error[i]
			
			#Final output after applying PID
			self.out[i] = self.out[i] + self.Pterm[i] * self.Kp[i] + self.Dterm[i] * self.Kd[i]

			self.prev_value[i]=self.drone_position[i] #updating the value of prev_value of drone
			self.out[i]=min(500,max(-500,self.out[i])) #thresholding the value of output b/w range 1000 to 2000
		
		self.cmd.rcRoll, self.cmd.rcPitch, self.cmd.rcThrottle=1500 - self.out[0], 1500 + self.out[1], 1500+self.out[2]
		self.command_pub.publish(self.cmd)
		self.roll_error_pub.publish(self.error[0]), self.pitch_error_pub.publish(self.error[1]), self.alt_error_pub.publish(self.error[2])

#================================================================================================================		

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	e_drone = Edrone()
	r = rospy.Rate(20)	#specify rate in Hz based upon your desired PID sampling time
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
		key = getKey()	#If entered key in terminal is ctrl then land the drone
		if key == '\x03':
			break
	e_drone.land()
	e_drone.disarm()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


