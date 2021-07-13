#! /usr/bin/env python

import roslib
roslib.load_manifest('test_utils')
import rospy
import actionlib

from test_utils.msg import GenericSignalAction, GenericSignalActionResult, GenericSignalActionFeedback

class Generate_signal_server(object):
  _feedback=GenericSignalActionFeedback()
  _result=GenericSignalActionResult()
  def __init__(self):
    self.server = actionlib.SimpleActionServer('generate_signal', GenericSignalAction, self.execute, False)
    self.server.start()
    

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("recieved goal")
    print(goal)
    if goal.req:
        Res=True
        print("b result")
        print(self._result)
        self._result.result.done=Res
        print("a result")
        print(self._result)
    #self.server.set_succeeded(self._result)
    self.server.set_succeeded(self._result.result)



if __name__ == '__main__':
  rospy.init_node('generate_signal_server')
  server = Generate_signal_server()
  rospy.spin()