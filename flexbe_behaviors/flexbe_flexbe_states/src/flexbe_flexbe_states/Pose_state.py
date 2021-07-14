#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionFeedback

from Planning.msg import PositionAction, PositionGoal
from Planning.msg import RandomPositionAction, RandomPositionGoal

from perception.msg import target_position_reportAction, target_position_reportGoal
from perception.msg import target_position_reportAction, target_position_reportActionResult, target_position_reportActionFeedback

import numpy as np

import geometry_msgs

import tf2_ros

class PositionActionState(EventState):
	def __init__(self,positional_error,shift=0.3):
		super(PositionActionState, self).__init__(outcomes = ['goal','no_goal','out_of_range','command_error'],
												 input_keys = ['pos_current_step_in'],
												 output_keys = ['pos_current_step_out'])



		self.shift=shift
		self.tfBuffer=tf2_ros.Buffer()
		self.tf_listener=tf2_ros.TransformListener(self.tfBuffer)
		self._positional_error = positional_error
		self._topic_pos = 'generate_position'
		self._topic_report = 'report_target_position'
		self._topic = 'to_position'
		#self._client_pos = ProxyActionClient({self._topic_pos: RandomPositionAction})
		self._client = ProxyActionClient({self._topic: PositionAction, self._topic_report: target_position_reportAction})
		#self._client = ProxyActionClient({self._topic: PositionAction,self._topic_pos: RandomPositionAction})
		self._error = False
		self.rate = rospy.Rate(0.2)
		self.trajectory=np.array([[1,-0.5,1,2*np.pi],
									[11.5,-0.5,1,2*np.pi],
                                    [11.5,-1.5,1,0],
									[1,-1.5,1,0],
									[1,-2.5,1,np.pi],
									[11.5,-2.5,1,np.pi],
									[11.5,-3.5,1,0],
                                    [1,-3.5,1,0],
									[1,-4.5,1,np.pi],
									[11.5,-4.5,1,np.pi],
									[11.5,-5.5,1,0],
                                    [1,-5.5,1,0],
                                    ])
	def execute(self, userdata):
		if self._error:
			return 'command_error'

		if self._client.has_result(self._topic) and self._client.has_result(self._topic_pos):
			result = self._client.get_result(self._topic)
			rand_pose = self._client.get_result(self._topic_pos)
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
		print("ENTER")
		#print(userdata.pose_goal)

		#Pose_goal = userdata.Pose

		Pose=PositionGoal()
		#Pose.goal = Pose_goal

		#Randgen=RandomPositionGoal() #Sim
		#Randgen.req=True			 #Sim

		pose_goal=target_position_reportGoal()
		pose_goal.req=True
		self._client.send_goal(self._topic_report, pose_goal)

		self._error = False
		#try:
		#	self._client_pos.send_goal(self._topic_pos, Randgen)
		#	self.rate.sleep()
		#	self.rate.sleep()
		#	#if self._client_pos.has_result(self._topic_pos):
		#	rand_pose = self._client_pos.get_result(self._topic_pos)
		#	Pose.command="position"
		#	Pose.goal = rand_pose.real_goal
		#	self._client.send_goal(self._topic, Pose)
		#except Exception as e:
		#	Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
		#	self._error = True

		try:
			client=self._client._clients.get(self._topic_report)
			print("wating")
			client.wait_for_result()
			print("result ready")
			rand_pose = self._client.get_result(self._topic_report)
			if rand_pose.range:
				Pose.command="position"
				Pose.goal = rand_pose.real_goal
				self._client.send_goal(self._topic, Pose)
			else:
				return 'out_of_range'
		except Exception as e:
			Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):
		print("EXIT")
		Pose=PositionGoal()
		Pose.command="home"
		self._client.send_goal(self._topic, Pose)

		#Salida navigation
		base_current_pose=self.tfBuffer.lookup_transform("map", 'base_arm', rospy.Time())
		print(base_current_pose)
		step=userdata.pos_current_step_in
		print("paso")
		print(step)
		pt1=self.trajectory[step-1,:1]
		pt2=self.trajectory[step,:1]
		l=pt2-pt1
		print(l)
		current_pt=np.array([base_current_pose.transform.translation.x,base_current_pose.transform.translation.y])
		print(current_pt)
		current_pt_proy=(np.dot(l,current_pt-pt1))*(l/np.linalg.norm(l))
		goal_proy=current_pt_proy+(l/np.linalg.norm(l))*self.shift
		goal_o=goal_proy+pt1


		goal = MoveBaseGoal()
		print(goal)
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = goal_o[0]
		goal.target_pose.pose.position.y = goal_o[1]
		goal.target_pose.pose.orientation.z = base_current_pose.transform.rotation.z
		goal.target_pose.pose.orientation.w = base_current_pose.transform.rotation.w

		try:
			self._client.send_goal(self._topic, goal)
			#self._client_pos.wait_for_result()
			self.rate.sleep()
			print("paso")
			print(self.step)
			userdata.pos_current_step_out=step
		except Exception as e:
			Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
			self._error = True

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')