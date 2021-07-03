#! /usr/bin/env python

import roslib
roslib.load_manifest("Planning")
import rospy
import actionlib

from Planning.msg import PositionAction, PositionGoal
import geometry_msgs

if __name__=='__main__':
    rospy.init_node('pose_client')
    client=actionlib.SimpleActionClient('to_position',PositionAction)
    client.wait_for_server()
    pose_goal=PositionGoal()
    #Fill goal
    print(pose_goal)
    #pose_goal = geometry_msgs.msg.Pose()
    print("pose")
    #pose_goal.orientation.z= -1
    pose_goal.goal.position.x = 0.1
    pose_goal.goal.position.y = -0.2
    pose_goal.goal.position.z = 0.25
    print("pose_goal")
    print(pose_goal)

    client.send_goal(pose_goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

