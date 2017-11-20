#!/usr/bin/env python

import rospy
import numpy as np

from Husky import Husky
from PIDController import PDController
from GazeboStateEstimation import GazeboStateEstimation


class WayPointNavigation:

	def __init__(self,model_obj):
		
		self.mobile_base=model_obj

		gains=np.array([[0.5,0.2],[0.7,0.2]])
		self.controller=PDController(gains)
			
		self.state_estimator=GazeboStateEstimation(self.mobile_base.model_name)


	def polar_cordinates(self,position):

		radius=np.sqrt((position[0]**2)+(position[1]**2))

		yaw=np.arctan2([position[1]],[position[0]])

		return radius,yaw


	def run(self):

		while True:

			print self.state_estimator.get_model_rotation(init_flag=True).reshape(3,2)


	'''def run(self,target):
	
		cur_pos_vel=self.state_estimator.get_model_state(init_flag=True).reshape(3,2)


		r_t,yaw_t=self.polar_cordinates(cur_pos_vel[:,0])

		
		r_t,_=self.polar_cordinates(target)

		cur_state=cur_pos_vel[:,0]

		yaw_flag=False

		r_flag=False

		finish_flag=False

		prev_yaw=0.0

		rate=rospy.Rate(100)

		while not r_flag:

			cur_state=cur_pos_vel[:,0]

			r_c,yaw_t=self.polar_cordinates(target-cur_state)

			#print r_c,yaw_t,cur_state[2]
			if abs(r_c)<0.05:
					r_flag=True

			yaw_error=yaw_t-cur_state[2]

			yaw_vel=2.5*yaw_error

			if yaw_vel<-1.0: yaw_vel=-1.0
			if yaw_vel>1.0: yaw_vel=1.0


			if abs(yaw_error)<0.1:
				yaw_flag=True



			if yaw_flag:
				r_vel=1.5*r_c

				if r_vel<-1.0: r_vel=-1.0
				if r_vel>1.0:r_vel=1.0

			else:r_vel=0.0


			action=np.array([r_vel,0,yaw_vel])

			cur_pos_vel=self.get_next_state(cur_pos_vel,action).reshape(3,2)


			prev_yaw=yaw_error

			rate.sleep()'''

if __name__=="__main__":

	
	rospy.init_node("Moving_base")

	base=Husky("mobile_base")

	obj=WayPointNavigation(base)

	obj.run()



