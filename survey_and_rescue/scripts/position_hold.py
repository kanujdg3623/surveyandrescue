#!/usr/bin/env python
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

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
class Edrone():
	def __init__(self):
		rospy.init_node('drone_control')
		self.drone_position = [0.0,0.0,0.0]
		
		self.cell_coords={}
		with open('/home/kdg/cell_coords.json','r') as readfile:
			self.cell_coords=json.load(readfile)
		self.setpoint=self.cell_coords.get("D3")
		self.pid_setpoint=[0,0,0]
		
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		
		#=======================#
		self.Kp = [8,8,20]	#
		self.Ki = [0,0,0]	#
		self.Kd = [300,300,200]	#
		#=======================#
		
		self.prev_value=[0,0,0]
		self.min_values=[1000,1000,1000]
		self.max_values=[2000,2000,2000]
		
		self.error=[0,0,0]
		self.Pterm=[0,0,0]
		self.Iterm=[0,0,0]
		self.Dterm=[0,0,0]
		self.out=[0,0,0]

		self.rp=False

		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=5)
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/decision_info',SRInfo,self.decision_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
				
		self.arm()

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

	def whycon_callback(self,msg):
		self.drone_position = msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z
		
	def decision_callback(self,msg):
		self.out=[0,0,0]
		self.Iterm=[0,0,0]
		self.setpoint=self.cell_coords.get(msg.location)
		
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.01
		self.Ki[2] = alt.Ki * 0.001
		self.Kd[2] = alt.Kd * 0.1
		
	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.01
		self.Ki[1] = pitch.Ki * 0.001
		self.Kd[1] = pitch.Kd * 0.1

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.01
		self.Ki[0] = roll.Ki * 0.001
		self.Kd[0] = roll.Kd * 0.1

	def pid(self):
		for i in range(3):
			if i<2 and not self.rp:
				continue
			self.error[i]=self.drone_position[i]-self.setpoint[i]

			self.Pterm[i] = self.error[i]
			self.Dterm[i] = self.drone_position[i] - self.prev_value[i]
			self.Iterm[i] = self.Iterm[i] + self.error[i]
			
			if self.error[i]*(self.prev_value[i]-self.setpoint[i])<=0:
				if i<2
					self.Iterm[i]=0
				elif self.error[2]<=0:
					self.Iterm[2]=10
				else:
					self.Iterm[2]=-10
			self.out[i] = self.Pterm[i] * self.Kp[i] + self.Iterm[i] * self.Ki[i] + self.Dterm[i] * self.Kd[i]		

			self.prev_value[i]=self.drone_position[i]
				
			self.out[i]=min(500,max(-500,self.out[i]))
			
		self.cmd.rcThrottle=1500+self.out[2]
		self.command_pub.publish(self.cmd)
		
		if self.rp:
			self.cmd.rcRoll, self.cmd.rcPitch=1500 - self.out[0], 1500 + self.out[1]
			self.command_pub.publish(self.cmd)
		self.rp=not self.rp
		
		self.roll_error_pub.publish(self.error[0]), self.pitch_error_pub.publish(self.error[1]), self.alt_error_pub.publish(self.error[2])
		

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	e_drone = Edrone()
	r = rospy.Rate(20)
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
		key = getKey()
		if key == '\x03':
			break
	e_drone.land()
	e_drone.disarm()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
