#! /usr/bin/env python

import roslib
roslib.load_manifest('Planning')
import rospy
import actionlib
from goal_pose import MoveGroupPythonInterfaceTutorial

from Planning.msg import PositionAction, PositionActionResult, PositionActionFeedback

class To_position_server(object):
  _feedback=PositionActionFeedback()
  _result=PositionActionResult()
  def __init__(self):
    self.MG = MoveGroupPythonInterfaceTutorial('to_position_server')
    self.server = actionlib.SimpleActionServer('to_position', PositionAction, self.execute, False)
    self.server.start()
    

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("recieved in server")
    print(goal)

    if goal.command=="home":
      print("to home")
      self.MG.go_to_joint_state(j1=0,j2=0.6928,j3=0.9398)
      self.server.set_succeeded()
    elif goal.command=="position":
      self.MG.go_to_pose_goal(goal.goal)
      real_pose=self.MG.move_group.get_current_pose().pose
      self._result.result.real_goal=real_pose
      self.server.set_succeeded(self._result.result)
    else:
      print("invalid command")


if __name__ == '__main__':
  #rospy.init_node('to_position_server')
  server = To_position_server()
  rospy.spin()