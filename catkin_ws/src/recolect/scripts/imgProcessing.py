#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import tf
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.fruitPoint = PointStamped()
    rospy.Subscriber("/recolect/camera1/image_raw", Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    
    gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        
    gray = cv.medianBlur(gray, 5)
    
    
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=20, maxRadius=100)
    
    if circles is not None:
      circles = np.int16(np.around(circles))
      for i in circles[0, :]:
        center = (i[0], i[1])
        # circle center
        cv.circle(cv_image, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = i[2]
        cv.circle(cv_image, center, radius, (255, 0, 255), 3)
    
      """ distance = np.subtract(circles[0][1],circles[0][0])
      self.fruitPoint.header = PointStamped().header
      self.fruitPoint.header.frame_id = "camera1_link"
      self.fruitPoint.point.x = float(distance[0]) /480
      self.fruitPoint.point.y = float(distance[1]) /640
      listener = tf.TransformListener()
      pointTrasformed = listener.transformPoint("base_link", self.fruitPoint)
      print(self.fruitPoint.point.x)
      print(self.fruitPoint.point.y) """

    cv.imshow("detected circles", cv_image)
    cv.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('houghTransform', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
