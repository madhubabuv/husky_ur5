#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3
import numpy as np

class Husky:


	def __init__(self,model_name):

		self.model_name=model_name

		self.pubCmd=rospy.Publisher("/cmd_vel",Twist,queue_size=1)

		self.aggressive=0.75


	def set_aggressive(self,scale):

		self.aggressive=scale

	
	def Command(self,cmd,time=0.0): # command has to be size (x,y,z,yaw)
	
		cmd=cmd*self.aggressive

		if len(cmd)<4:
			
			cmd=np.append(cmd,0.0)
			
		command=Twist(Vector3(cmd[0],cmd[1],cmd[2]),Vector3(0,0,cmd[3]))

		self.pubCmd.publish(command)

		rospy.sleep(time)

	








