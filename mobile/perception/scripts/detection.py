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
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs import point_cloud2


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/rgb/process_1",Image)

    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/rgb/image",Image)
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

    _, thresh1 = cv2.threshold(self.cv_image[:, :, 0], 0, 30, cv2.THRESH_BINARY)
    _, thresh2 = cv2.threshold(self.cv_image[:, :, 1], 200, 255, cv2.THRESH_BINARY)
    _, thresh3 = cv2.threshold(self.cv_image[:, :, 2], 0, 10, cv2.THRESH_BINARY)
    _, thresh4 = cv2.threshold(hsv[:, :, 1], 200, 255, cv2.THRESH_BINARY)
    rthresh = cv2.bitwise_and(cv2.bitwise_and(thresh1, thresh2), cv2.bitwise_and(thresh3, thresh4))


    dilatation_size = 7
    dilation_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                       (dilatation_size, dilatation_size))
    
    thresh = cv2.dilate(rthresh, element)


    # Find contours:
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    N = len(contours)
    #rospy.loginfo("%d %d %d", N, self.depth_image.shape[0], self.depth_image.shape[1])
    max_cont = sorted(contours, key=lambda x: cv2.contourArea(x))
    # Draw contours:
    #cv2.drawContours(a, max_cont, N-1, (255, 0, 0), 2)
    #cv2.drawContours(a, max_cont, N-2, (255, 0, 0), 2)
    val = min(6, N)
    for i in range(val):
      br1 = cv2.boundingRect(max_cont[-(i+1)])
      cv2.rectangle(self.cv_image, (int(br1[0]), int(br1[1])), \
          (int(br1[0]+br1[2]), int(br1[1]+br1[3])), (255, 0, 0), 2)
    if len(max_cont)>0:
      cnt = max_cont[-1]
      M = cv2.moments(cnt)
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      ix =cx + cy*480 #
      rospy.loginfo("%f %f %f %d %d", self.depth_image[ix][0], self.depth_image[ix][1], self.depth_image[ix][2], cx, cy)




    cv2.imshow("Image window", np.hstack((cv2.bitwise_and(self.cv_image,self.cv_image, mask= rthresh), self.cv_image)))
    #cv2.imshow("Image window", cv2.bitwise_and(self.cv_image,self.cv_image, mask= thresh))
    #cv2.imshow("Image window", self.cv_image)
    #cv2.imshow("Image window", np.hstack((np.dstack((self.sdepth_image, np.zeros_like(self.sdepth_image), np.zeros_like(self.sdepth_image))), self.cv_image)))
    #cv2.imshow("Ra", self.depth_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
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