#! /usr/bin/env python

import roslib
roslib.load_manifest("Planning")
import rospy
import actionlib

from Planning.msg import PositionAction, RandomPositionGoal
from Planning.msg import RandomPositionAction, RandomPositionActionResult, RandomPositionActionFeedback
import geometry_msgs

if __name__=='__main__':
    rospy.init_node('generate_pose_client')
    client=actionlib.SimpleActionClient('generate_position',RandomPositionAction)
    client.wait_for_server()
    pose_goal=RandomPositionGoal()
    #Fill goal
    #print(pose_goal)
    #print("generate_pose")
    #pose_goal.goal.position.x = 0.1
    #pose_goal.goal.position.y = -0.2
    #pose_goal.goal.position.z = 0.25
    #print("pose_goal")
    #print(pose_goal)
    pose_goal.req=

    client.send_goal(pose_goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print(client.get_result())

