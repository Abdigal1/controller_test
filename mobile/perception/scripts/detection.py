#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('perception')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/rgb/process_1",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.flag = True

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    if self.flag:
      cv2.imwrite('/home/lambda/Downloads/avr.jpg', cv_image)
      self.flag  = False
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    _, thresh1 = cv2.threshold(cv_image[:, :, 0], 0, 50, cv2.THRESH_BINARY)
    _, thresh2 = cv2.threshold(cv_image[:, :, 1], 180, 255, cv2.THRESH_BINARY)
    _, thresh3 = cv2.threshold(cv_image[:, :, 2], 0, 50, cv2.THRESH_BINARY)
    _, thresh4 = cv2.threshold(hsv[:, :, 1], 0, 255, cv2.THRESH_BINARY)
    thresh = cv2.bitwise_and(cv2.bitwise_and(thresh1, thresh2), cv2.bitwise_and(thresh3, thresh4))
    # Find contours:
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    N = len(contours)
    rospy.loginfo("%d", N)
    max_cont = sorted(contours, key=lambda x: cv2.contourArea(x))
    # Draw contours:
    #cv2.drawContours(a, max_cont, N-1, (255, 0, 0), 2)
    #cv2.drawContours(a, max_cont, N-2, (255, 0, 0), 2)
    val = min(6, N)
    for i in range(val):
      br1 = cv2.boundingRect(max_cont[-(i+1)])
      cv2.rectangle(cv_image, (int(br1[0]), int(br1[1])), \
          (int(br1[0]+br1[2]), int(br1[1]+br1[3])), (255, 0, 0), 2)


    #cv2.imshow("Image window", cv2.bitwise_and(cv_image,cv_image, mask= thresh))
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)