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

from perception.msg import target_position_reportAction, target_position_reportGoal
from perception.msg import target_position_reportAction, target_position_reportActionResult, target_position_reportActionFeedback

import numpy as np

import geometry_msgs

class NavigationActionState(EventState):
	def __init__(self,positional_error):
		super(NavigationActionState, self).__init__(outcomes = ['target_detected','next_step','end','command_error'],
												 input_keys = ['nav_current_step_in'],
												 output_keys = ['nav_current_step_out'])

		self._positional_error = positional_error
		self.interrupt_navigation=False
		self.step=0
		self.goal_step=False
		self._topic = 'move_base'
		self._topic_report = 'report_target_position'
		self._topic_sig = 'generate_signal'
		self._client = ProxyActionClient({self._topic: MoveBaseAction, self._topic_report: target_position_reportAction})
		#self._client = ProxyActionClient({self._topic: MoveBaseAction, self._topic_sig: GenericSignalAction})
		#self._topic_sig = 'generate_signal'
		#self._client_sig = ProxyActionClient({self._topic_sig: GenericSignalAction})
		self._error = False
		self.rate = rospy.Rate(0.2)
		#                          x,y,z_r,w
#		self.trajectory=np.array([[1,-0.5,0,1],
#									[5.5,-0.5,0,1],
#									[11.8,-0.5,0,1],
#									[11.5,-1.0,0,0.5],#Se va acercando
#									[11.5,-1.2, 0, 0], 
#                                   [11.5,-1.5,1,0],
#									[5.5,-1.5,1,0],
#									[1,-1.5,1,0],
#									[5.5,-1.5,1,0],
#									[1,-2.5,0,1],
#									[11.5,-2.5,0,1],
#									[11.5,-3.5,1,0],
#                                   [1,-3.5,1,0],
#									[1,-4.5,0,1],
#									[11.5,-4.5,0,1],
#									[11.5,-5.5,1,0],
#                                   [1,-5.5,1,0],
#                                   ])

		self.trajectory=np.array([#[1,0.1,0,1],
									#[3,0,0,1],
									[6,0,0,1],
									[6,-0.5,0,0.5],#Se va acercando
									[6,-0.7, 0, 0], 
                                   [6,-1.0,1,0],
									[1,-1.0,1,0],
									[0,-1.5,1,-0.5],
									[3,-1.0,1,0],
									[1,-2.0,0,1],
									[6,-2.0,0,1],
									[6,-3.0,1,0],
                                   [1,-3.0,1,0],
									[1,-4.0,0,1],
									[6,-4.0,0,1],
									[6,-5.0,1,0],
                                   [1,-5.0,1,0],
                                    ])
        


	def execute(self, userdata):
		if self._error:
			return 'command_error'

		pose_goal=target_position_reportGoal()
		pose_goal.req=True

		self._client.send_goal(self._topic_report, pose_goal)

		client=self._client._clients.get(self._topic_report)
		client.wait_for_result()
		#print("result")
		result = self._client.get_result(self._topic_report)
		if result.range:
			print("new result!")
			self.interrupt_navigation = True

		elif self._client.has_result(self._topic):
			if self.step<11:
				print("next")
				self.step=self.step+1
				return 'next_step'
			else:
				print("fin")
				return 'end'		
		#elif self.goal_step==True:
		#	if self.step<11:
		#		print("next")
		#		self.step=self.step+1
		#		return 'next_step'
		#	else:
		#		print("fin")
		#		return 'end'
			
			self.goal_step=False

		if self.interrupt_navigation==True:
			print("aborting")
			self._client.cancel(self._topic)
			self.rate.sleep()
			print("cancled goal")
			self.interrupt_navigation = False
			#userdata.step=self.step
			userdata.nav_current_step_out=self.step
			return 'target_detected'
		



	def on_enter(self, userdata):
		#self.step=userdata.nav_current_step_in
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = self.trajectory[self.step,0]
		goal.target_pose.pose.position.y = self.trajectory[self.step,1]
		goal.target_pose.pose.orientation.z = self.trajectory[self.step,2]
		goal.target_pose.pose.orientation.w = self.trajectory[self.step,3]

		#goal.target_pose.pose.position.x = 11.5
		#goal.target_pose.pose.position.y = -0.5
		#goal.target_pose.pose.orientation.z =0 
		#goal.target_pose.pose.orientation.w = 1
		self._error = False
		try:
			self._client.send_goal(self._topic, goal)
			print("paso")
			print(self.step)
			client=self._client._clients.get(self._topic)
			#client.wait_for_result()
			#print("goal")
			#self.goal_step=True
			#if self.step<11:
			#	print("next")
			#	return 'next_step'
			#else:
			#	print("fin")
			#	return 'end'

		except Exception as e:
			Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):
		print("EXIT")
		#self.step=self.step+1
		userdata.nav_current_step_out=self.step
		print(self.step)
		userdata.nav_current_step_out=self.step
		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')