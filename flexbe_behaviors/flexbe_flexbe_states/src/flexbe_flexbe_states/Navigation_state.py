#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import rospy

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from Planning.msg import PositionAction, PositionGoal
from Planning.msg import RandomPositionAction, RandomPositionGoal
from test_utils.msg import GenericSignalAction,GenericSignalGoal, GenericSignalActionResult, GenericSignalActionFeedback
import numpy as np

import geometry_msgs

class NavigationActionState(EventState):
	def __init__(self,positional_error):
		super(NavigationActionState, self).__init__(outcomes = ['target_detected','command_error'],
												 input_keys = ['pose_goal'],
												 output_keys = ['position_error'])

		self._positional_error = positional_error
		self.interrupt_navigation=False
		self.step=0
		self._topic = 'move_base'
		self._client = ProxyActionClient({self._topic: MoveBaseAction})
		self._topic_sig = 'generate_signal'
		self._client_sig = ProxyActionClient({self._topic_sig: GenericSignalAction})
		self._error = False
		self.rate = rospy.Rate(0.2)
		#                          x,y,z_r,w
		self.trajectory=np.array([[1,0,0,1],
                                    [2,0,0,1],
                                    [3,0,0,1],
                                    [3,1,0,1],
                                    [2,1,0,1],
                                    [1,1,0,1],
                                    [0,1,0,1],
                                    [1,1,0,1],
                                    [2,1,0,1],
                                    [3,1,0,1],
                                    [3,2,0,1],
                                    [2,2,0,1],
                                    [1,2,0,1],
                                    [0,2,0,1],
                                    ])
        


	def execute(self, userdata):
		if self._error:
			return 'command_error'

		#if self._client.has_result(self._topic):
			#result = self._client.get_result(self._topic)
			#rand_pose = self._client_pos.get_result(self._topic_pos)
            #Resultado de Vision Artificial

		#	print("error posicional")
		#	print(userdata.position_error)
#
		#	if userdata.position_error < self._positional_error:
		#		return 'goal'
		#	else:
		#		return 'no_goal'


		GenSig=GenericSignalGoal()
		GenSig.req=True
		#while self.interrupt_navigation==False:
		#	self._client_sig.send_goal(self._topic_sig, GenSig) #Sim
		#	print("sleep")
		#	self.rate.sleep()
		#	result = self._client_sig.get_result(self._topic_sig) #Sim
		#	#print(result.done)
		#	self.interrupt_navigation = result.done

		#self._client_sig.send_goal(self._topic_sig, GenSig) #Sim
		print("sleep")
		self.rate.sleep()
		result = self._client_sig.get_result(self._topic_sig) #Sim
		print("result")
		print(result.done)
		self.interrupt_navigation = result.done

		if self.interrupt_navigation==True:
			print("aborting")
			self._client.cancel(self._topic)
			print("cancled goal")
			return 'target_detected'
		



	def on_enter(self, userdata):
		goal = MoveBaseGoal()
		print(goal)
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		#goal.target_pose.pose.position.x = self.trajectory[self.step,0]
		#goal.target_pose.pose.position.y = self.trajectory[self.step,1]
		#goal.target_pose.pose.orientation.z = self.trajectory[self.step,2]
		#goal.target_pose.pose.orientation.w = self.trajectory[self.step,3]

		goal.target_pose.pose.position.x = 11
		goal.target_pose.pose.position.y = 1.5
		goal.target_pose.pose.orientation.z =0 
		goal.target_pose.pose.orientation.w = 1
		self._error = False
		try:
			self._client.send_goal(self._topic, goal)
			#self._client_pos.wait_for_result()
			self.rate.sleep()
		except Exception as e:
			Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')