#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import rospy

#from goal_pose import MoveGroupPythonInterfaceTutorial

from Planning.msg import PositionAction, PositionGoal
from Planning.msg import RandomPositionAction, RandomPositionGoal
import numpy as np

import geometry_msgs

class PositionActionState(EventState):
	'''
	Actionlib actions are the most common basis for state implementations
	since they provide a non-blocking, high-level interface for robot capabilities.
	The example is based on the DoDishes-example of actionlib (see http://wiki.ros.org/actionlib).
	This time we have input and output keys in order to specify the goal and possibly further evaluate the result in a later state.

	-- dishes_to_do int 	Expected amount of dishes to be cleaned.

	># Pose_goal 	int 	ID of the dishwasher to be used.

	#> position_error 		int 	Amount of cleaned dishes.

	<= goal 		Only a few dishes have been cleaned.
	<= no_goal		Cleaned a lot of dishes.

	'''
	def __init__(self,positional_error):
		super(PositionActionState, self).__init__(outcomes = ['goal','no_goal','command_error'],
												 input_keys = ['pose_goal'],
												 output_keys = ['position_error'])



		self._positional_error = positional_error
		self._topic_pos = 'generate_position'
		self._topic = 'to_position'
		self._client_pos = ProxyActionClient({self._topic_pos: RandomPositionAction})
		self._client = ProxyActionClient({self._topic: PositionAction})
		self._error = False
		self.rate = rospy.Rate(0.2)


	def execute(self, userdata):
		if self._error:
			return 'command_error'

		if self._client.has_result(self._topic) and self._client_pos.has_result(self._topic):
			result = self._client.get_result(self._topic)
			rand_pose = self._client_pos.get_result(self._topic_pos)
			print(result)
			print(rand_pose)
			rcoord=np.array([
				result.real_goal.position.x,
				result.real_goal.position.y,
				result.real_goal.position.z
			])
			coord=np.array([
				rand_pose.real_goal.position.x,
				rand_pose.real_goal.position.y,
				rand_pose.real_goal.position.z
			])
			print(result)

			userdata.position_error = np.linalg.norm(rcoord-coord)
			print("error posicional")
			print(userdata.position_error)
			print(self._positional_error)

			if userdata.position_error < self._positional_error:
				return 'goal'
			else:
				return 'no_goal'

	def on_enter(self, userdata):
		#print(userdata.pose_goal)

		#Pose_goal = userdata.Pose

		Pose=PositionGoal()
		#Pose.goal = Pose_goal

		Randgen=RandomPositionGoal()
		Randgen.req=True

		self._error = False
		try:
			self._client_pos.send_goal(self._topic_pos, Randgen)
			#self._client_pos.wait_for_result()
			self.rate.sleep()
			rand_pose = self._client_pos.get_result(self._topic_pos)
			Pose.goal = rand_pose.real_goal
			self._client.send_goal(self._topic, Pose)
		except Exception as e:
			Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')