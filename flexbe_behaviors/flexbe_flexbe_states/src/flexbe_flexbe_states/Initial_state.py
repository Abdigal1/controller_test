#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import rospy

from Planning.msg import PositionAction, PositionGoal
from Planning.msg import RandomPositionAction, RandomPositionGoal
import numpy as np

import geometry_msgs

class InitActionState(EventState):
	def __init__(self):
		super(InitActionState, self).__init__(outcomes = ['ready','command_error'],input_keys = [],output_keys = [])
		self._topic = 'to_position'
		self._client = ProxyActionClient({self._topic: PositionAction})
		self._error = False
		self.rate = rospy.Rate(1)
	def execute(self, userdata):
		if self._error:
			return 'command_error'
		else:
			return "ready"
	def on_enter(self, userdata):
		Pose=PositionGoal()
		Pose.command="home"
		self._client.send_goal(self._topic, Pose)
	def on_exit(self, userdata):
		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')