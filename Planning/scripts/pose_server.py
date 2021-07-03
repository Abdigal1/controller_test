#! /usr/bin/env python

import roslib
roslib.load_manifest('Planning')
import rospy
import actionlib
from goal_pose import MoveGroupPythonInterfaceTutorial

from Planning.msg import PositionAction

class To_position_server:
  def __init__(self):
    self.MG = MoveGroupPythonInterfaceTutorial('to_position_server')
    self.server = actionlib.SimpleActionServer('to_position', PositionAction, self.execute, False)
    self.server.start()
    

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("recieved in server")
    print(goal)
    self.MG.go_to_pose_goal(goal.goal)
    #print(self.move_group.get_current_pose().pose)
    self.server.set_succeeded()


if __name__ == '__main__':
  #rospy.init_node('to_position_server')
  server = To_position_server()
  rospy.spin()