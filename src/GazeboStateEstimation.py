#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Vector3
from std_msgs.msg import Empty

from gazebo_msgs.srv import GetLinkState,SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Quaternion,Vector3,Twist,Point
from std_msgs.msg import Float64
import random
import numpy as np
import tf.transformations as tf




class GazeboStateEstimation:

	def __init__(self,model_name):


		self.link_name=model_name+"::base_link"
		self.ref_link="link"

		self.dt=0.05

		self.yaw=0.0

		self.avg_dt=[]
		
		self.start=rospy.get_rostime().to_sec()



	def get_model_state(self,init_flag=False):

		rospy.wait_for_service("/gazebo/get_link_state")

		try:
			get_link_state=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
			
			state=get_link_state(self.link_name,self.ref_link)
			
			pose=state.link_state.pose.position

			if init_flag:

				return np.atleast_1d([pose.x,0.0,pose.y,0.0,pose.z,0.0])
			
			return pose

		except rospy.ServiceException as e:
			
			print "Ros service exception",e.message
			return None

	def get_model_rotation(self,init_flag=False):

		rospy.wait_for_service("/gazebo/get_link_state")

		try:
			get_link_state=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
			
			state=get_link_state(self.link_name,self.ref_link)
			
			quaternion=state.link_state.pose.orientation

			eular_angles=tf.euler_from_quaternion((quaternion.x,quaternion.y,quaternion.z,quaternion.w))

			if init_flag:

				return np.atleast_1d([eular_angles[0],0.0,eular_angles[1],0.0,eular_angles[2],0])
			
			return np.array(eular_angels)

		except rospy.ServiceException as e:
			
			print "Ros service exception",e.message
			return None

		

	def get_next_state(self,cur_state,rotation_flag=False):


		cur_state=cur_state.reshape(3,2)

		state=self.get_drone_state()
	
		if rotation_flag:

			state=self.get_drone_rotation

		position=np.atleast_2d([state.x,state.y,state.z])
		
		dt=rospy.get_rostime().to_sec()-self.start
				
		if len(self.avg_dt)<10:

			self.avg_dt.append(dt)

		else:
			del self.avg_dt[0]

		avg_time=np.average(self.avg_dt)

		velocity=((position-cur_state[:,0]))/avg_time

		self.start=rospy.get_rostime().to_sec()

		next_state=np.append(position,velocity,axis=0)

		return next_state.T.ravel()
	






