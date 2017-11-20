#!/usr/bin/env python

import numpy as np

class PDController:

	def __init__(self,gains):
		
		"""
		This controller For x,y,z controller

		this doesnt inclue yaw

		"""

		self.gains=gains

		self.max_in=1.0

		self.min_in=-1.0


	def get_control_input(self,error):

		"""	
		The error inclues states and velocities
		
		[x,vx,y,vy,z,vz]

		"""

		gains=np.array(self.gains).reshape(3,2)
	
		error=error.reshape(3,2)

		control=np.sum(gains*error,axis=1)

		for i in range(len(control)):
			if control[i]>self.max_in:
				control[i]=self.max_in
			elif control[i]<self.min_in:
				control[i]=self.min_in

		return control














