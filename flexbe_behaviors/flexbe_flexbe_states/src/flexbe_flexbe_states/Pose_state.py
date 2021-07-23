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

import tf2_geometry_msgs

import tf2_ros

from geometry_msgs.msg import Pose

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass


class PositionActionState(EventState):
	def __init__(self,positional_error,shift=1.3):
		super(PositionActionState, self).__init__(outcomes = ['goal','no_goal','out_of_range','command_error'],
												 input_keys = ['pos_current_step_in'],
												 output_keys = ['pos_current_step_out'])



		self.shift=shift
		self.tfBuffer=tf2_ros.Buffer()
		self.tf_listener=tf2_ros.TransformListener(self.tfBuffer)
		self._positional_error = positional_error
		self._topic_nav = 'move_base'
		self._topic_pos = 'generate_position'
		self._topic_report = 'report_target_position'
		self._topic = 'to_position'
		#self._client_pos = ProxyActionClient({self._topic_pos: RandomPositionAction})
		self._client = ProxyActionClient({self._topic_nav: MoveBaseAction, self._topic: PositionAction, self._topic_report: target_position_reportAction})
		#self._client = ProxyActionClient({self._topic: PositionAction,self._topic_pos: RandomPositionAction})
		self._error = False
		self.rate = rospy.Rate(0.2)
		self.target_reached=False
		#self.trajectory=np.array([[1,-0.5,1,2*np.pi],
		#							[11.5,-0.5,1,2*np.pi],
        #                            [11.5,-1.5,1,0],
		#							[1,-1.5,1,0],
		#							[1,-2.5,1,np.pi],
		#							[11.5,-2.5,1,np.pi],
		#							[11.5,-3.5,1,0],
        #                            [1,-3.5,1,0],
		#							[1,-4.5,1,np.pi],
		#							[11.5,-4.5,1,np.pi],
		#							[11.5,-5.5,1,0],
        #                            [1,-5.5,1,0],
        #                            ])
		self.trajectory=np.array([#[1,0.1,0,1],
									#[3,0,0,1],
									[6.5, 0, 0, 1], 
                                   [7.0, -0.5, np.sin(-np.pi/4), np.cos(-np.pi/4)],
								   [6.5, -1, 1, 0],
									[0.5,-1.0,1, 0],
									[0.0,-1.5,np.sin(-np.pi/4),np.cos(-np.pi/4)],
									[0.5,-2.0,0,1]])
	
	def execute(self, userdata):
		if self._error:
			return 'command_error'

		if self._client.has_result(self._topic) and self._client.has_result(self._topic_report) and self._client.has_result(self._topic_nav):
			return 'goal'
			#result = self._client.get_result(self._topic)
			#rand_pose = self._client.get_result(self._topic_report)
			#pose_stamped = tf2_geometry_msgs.PoseStamped()
			#pose_stamped.pose = rand_pose.real_goal
			#pose_stamped.header.frame_id = "optical"
    		##pose_stamped.header.stamp = rospy.Time.now()
			#target_pose=self.tfBuffer.transform(pose_stamped, 'base_arm', rospy.Duration(0.5))
			#target_pose.pose.position.z=0.25
#
			#print(result)
			#print(rand_pose)
			#rcoord=np.array([
			#	result.real_goal.position.x,
			#	result.real_goal.position.y,
			#	result.real_goal.position.z
			#])
			#coord=np.array([
			#	target_pose.pose.position.x,
			#	target_pose.pose.position.y,
			#	target_pose.pose.position.z
			#])
			#print(result)
#
			##userdata.position_error = np.linalg.norm(rcoord-coord)
			#print("error posicional")
			##print(userdata.position_error)
			#print(self._positional_error)
#
			#if np.linalg.norm(rcoord-coord) < self._positional_error:
			#	self.target_reached=True
			#	return 'goal'
			#else:
			#	self.target_reached=False
			#	return 'no_goal'

	def on_enter(self, userdata):
		print("ENTER")

		Pose=PositionGoal()

		pose_goal=target_position_reportGoal()
		pose_goal.req=True
		self._client.send_goal(self._topic_report, pose_goal)

		self._error = False

		try:
			client=self._client._clients.get(self._topic_report)
			print("wating")
			client.wait_for_result()
			print("result ready")

			rand_pose = self._client.get_result(self._topic_report)
			#TF optical to global

			#target_pose=transform_pose(rand_pose.real_goal,"optical","odom")
			pose_stamped = tf2_geometry_msgs.PoseStamped()
			pose_stamped.pose = rand_pose.real_goal
			pose_stamped.header.frame_id = "optical"
    		#pose_stamped.header.stamp = rospy.Time.now()
			target_pose=self.tfBuffer.transform(pose_stamped, 'odom', rospy.Duration(0.5))

			#Move_base
			#base_current_pose=self.tfBuffer.lookup_transform("odom", 'base_arm', rospy.Time())
			goal = MoveBaseGoal()
			#print(goal)
			self.step=userdata.pos_current_step_in
			print("paso")
			print(self.step)
			goal.target_pose.header.frame_id = "odom"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = target_pose.pose.position.x
			goal.target_pose.pose.position.y = self.trajectory[self.step,1]
			#goal.target_pose.pose.position.x = base_current_pose.transform.translation.x+0.5
			#goal.target_pose.pose.position.y = self.trajectory[self.step,1]
			goal.target_pose.pose.orientation.z = self.trajectory[self.step,2]
			goal.target_pose.pose.orientation.w = self.trajectory[self.step,3]

			self._client.send_goal(self._topic_nav, goal)
			client=self._client._clients.get(self._topic_nav)

			client.wait_for_result()

			#target_pose=transform_pose(target_pose,"odom","base_arm")

			pose_stamped = tf2_geometry_msgs.PoseStamped()

			pose_stamped.pose = target_pose.pose

			pose_stamped.header.frame_id = "odom"
    		#pose_stamped.header.stamp = rospy.Time.now()


			target_pose=self.tfBuffer.transform(pose_stamped, 'base_arm', rospy.Duration(0.5))

			target_pose.pose.position.z=0.25


			if rand_pose.range:
				Pose.command="position"
				Pose.goal = target_pose.pose
				self._client.send_goal(self._topic, Pose)
			else:
				return 'out_of_range'
		except Exception as e:
			Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):
		print("EXIT")
		#if self.target_reached:
		if True:
			#try:
			print("sending goal to position")
			Pose=PositionGoal()
			Pose.command="home"
			self._client.send_goal(self._topic, Pose)
			client=self._client._clients.get(self._topic)
			print("wating")
			client.wait_for_result()
			print("paso")
			print(self.step)

			userdata.pos_current_step_out=self.step


			#except Exception as e:
			#	Logger.logwarn('Failed to send the Position command:\n%s' % str(e))
			#	self._error = True

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')