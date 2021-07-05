#! /usr/bin/env python

import roslib
roslib.load_manifest('Planning')
import rospy
import actionlib
#from goal_pose import MoveGroupPythonInterfaceTutorial
import geometry_msgs

from Planning.msg import RandomPositionAction, RandomPositionActionResult, RandomPositionActionFeedback
import numpy as np

class Generate_position_server(object):
  _feedback=RandomPositionActionFeedback()
  _result=RandomPositionActionResult()
  def __init__(self):
    self.server = actionlib.SimpleActionServer('generate_position', RandomPositionAction, self.execute, False)
    self.server.start()
    

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print(goal)
    if goal:
        np.random.uniform(low=0,high=0.5)
        pose=geometry_msgs.msg.Pose()
        pose.position.x=np.random.uniform(low=-0.5,high=0.5)
        pose.position.y=np.random.uniform(low=-0.5,high=0.5)
        pose.position.z=0.25
        print(pose)
        print(self._result)
        self._result.result.real_goal=pose
        print(self._result)
    #self.server.set_succeeded(self._result)
    self.server.set_succeeded(self._result.result)



if __name__ == '__main__':
  rospy.init_node('generate_position_server')
  server = Generate_position_server()
  rospy.spin()