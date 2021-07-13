#! /usr/bin/env python

import roslib
roslib.load_manifest("perception")
import rospy
import actionlib

from perception.msg import target_position_reportAction, target_position_reportGoal
from perception.msg import target_position_reportAction, target_position_reportActionResult, target_position_reportActionFeedback
import geometry_msgs

if __name__=='__main__':
    rospy.init_node('get_pose_client')
    print("init_client")
    client=actionlib.SimpleActionClient('report_target_position',target_position_reportAction)
    client.wait_for_server()
    pose_goal=target_position_reportGoal()
    #Fill goal
    #print(pose_goal)
    #print("generate_pose")
    #pose_goal.goal.position.x = 0.1
    #pose_goal.goal.position.y = -0.2
    #pose_goal.goal.position.z = 0.25
    #print("pose_goal")
    #print(pose_goal)
    pose_goal.req=True

    client.send_goal(pose_goal)
    print("sended")
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print(client.get_result())