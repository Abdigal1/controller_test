#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('perception')
import rospy
import actionlib
#from goal_pose import MoveGroupPythonInterfaceTutorial
import geometry_msgs

import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs import point_cloud2


from perception.msg import target_position_reportAction, target_position_reportActionResult, target_position_reportActionFeedback
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/rgb/process_1",Image)

    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/rgb/image_raw",Image)
    self.depth_sub = message_filters.Subscriber("/depth/points",PointCloud2)
    self.cv_image = None
    self.depth_image = None
    self.flag = True
    ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 50)
    ts.registerCallback(self.callback)




  def callback(self,data, ddata):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.depth_image = np.array(list(point_cloud2.read_points(ddata, field_names=("x", "y", "z"), skip_nans=False)))
      
      
    except CvBridgeError as e:
      print(e)
    self.do_stuff()
   
  def do_stuff(self):
    if self.flag:
      cv2.imwrite('/home/lambda/Downloads/avr.jpg', self.cv_image)
      self.flag  = False


    hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    #_, thresh1 = cv2.threshold(self.cv_image[:, :, 0], 0, 30, cv2.THRESH_BINARY)  #R
    #_, thresh2 = cv2.threshold(self.cv_image[:, :, 1], 200, 256, cv2.THRESH_BINARY)#G
    #_, thresh3 = cv2.threshold(self.cv_image[:, :, 2], 0, 10, cv2.THRESH_BINARY) #B
    #_, thresh4 = cv2.threshold(hsv[:, :, 1], 200, 255, cv2.THRESH_BINARY)          #S 
    #rthresh = cv2.bitwise_and(cv2.bitwise_and(thresh1, thresh2), cv2.bitwise_and(thresh3, thresh4))

    _, thresh1 = cv2.threshold(self.cv_image[:, :, 0], 0, 256, cv2.THRESH_BINARY)  #R
    _, thresh2 = cv2.threshold(self.cv_image[:, :, 1], 0, 256, cv2.THRESH_BINARY)#G
    _, thresh3 = cv2.threshold(self.cv_image[:, :, 2], 0, 256, cv2.THRESH_BINARY) #B
    _, thresh4 = cv2.threshold(hsv[:, :, 1], 254, 355, cv2.THRESH_BINARY)          #S 
    _, thresh5 = cv2.threshold(hsv[:, :, 2], 230, 355, cv2.THRESH_BINARY)          #V 
    rthresh = cv2.bitwise_and(thresh5,cv2.bitwise_not(thresh1))


    dilatation_size = 7
    dilation_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                       (dilatation_size, dilatation_size))
    
    thresh = cv2.dilate(rthresh, element)


    # Find contours:
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    N = len(contours)
    max_cont = sorted(contours, key=lambda x: cv2.contourArea(x))
    val = min(6, N)
    #Visualization
    for i in range(val):
      br1 = cv2.boundingRect(max_cont[-(i+1)])
      cv2.rectangle(self.cv_image, (int(br1[0]), int(br1[1])), \
          (int(br1[0]+br1[2]), int(br1[1]+br1[3])), (255, 0, 0), 2)
    cv2.imshow("Image window", np.hstack((cv2.bitwise_and(self.cv_image,self.cv_image, mask= rthresh), self.cv_image)))
    cv2.waitKey(3)
    
    if len(max_cont)>0:
      cnt = max_cont[-1]
      M = cv2.moments(cnt)
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      ix =cx + cy*480 #
      #rospy.loginfo("%f %f %f %d %d", self.depth_image[ix][0], self.depth_image[ix][1], self.depth_image[ix][2], cx, cy)
      self.target_position=self.depth_image[ix]

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

class Report_position_server(object):
  _feedback=target_position_reportActionFeedback()
  _result=target_position_reportActionResult()
  def __init__(self):
    print("init_server")
    self.server = actionlib.SimpleActionServer('report_target_position', target_position_reportAction, self.execute, False)
    self.server.start()
    self.ic = image_converter()
    self.th=0.7
    

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    #print("execute")
    print(goal)
    
    if goal:
      if np.linalg.norm(self.ic.target_position)<self.th and np.linalg.norm(self.ic.target_position)>(self.th-0.2):
        rospy.loginfo("Target detected")
        pose=geometry_msgs.msg.Pose()
        pose.position.x=self.ic.target_position[0]
        pose.position.y=self.ic.target_position[1]
        pose.position.z=0.25
        self._result.result.range=True
        self._result.result.real_goal=pose
      else:
        self._result.result.range=False
        print("distance")
        print(np.linalg.norm(self.ic.target_position))
        print("TARGET OUT OF RANGE")
        print(self._result.result)
    else:
      print("NO TARGET")
    #self.server.set_succeeded(self._result)
    self.server.set_succeeded(self._result.result)



if __name__ == '__main__':
  rospy.init_node('target_position_server')
  server = Report_position_server()
  rospy.spin()