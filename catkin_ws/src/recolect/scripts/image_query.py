#!/usr/bin/env python

# Python Libraries
import rospy
import string
import cv2
import numpy as np
import argparse
import imutils
import time

# ROS Message Types
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Keras Models and Image Preprocessing
from keras import models
from keras.preprocessing import image

# To Interface ROS Image Data into CV2 Image Data
from cv_bridge import CvBridge, CvBridgeError

# Query Class
class Query():
    def __init__(self):
        print('Initialising')

        # Load Trained Model
        self.model = models.load_model('/home/prkizir/catkin_ws/src/recolect/scripts/rvfo.h5')

        # CvBridge Instance
        self.bridge = CvBridge()

        # Save Image from ROS
        self.cv_image = 0

        # Image Capture Subscriber
        rospy.Subscriber("/two_wheels_robot/camera1/image_raw", Image, self.video_capture)

        # ROS Loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.classifier()
            rate.sleep()

    # ROS Image Capture Callback Function
    def video_capture(self, data):
        try:
            # Convert ROS Image into RGB8 Image -> Save to cv_image
            self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

        except CvBridgeError as e:
            print(e)

        cv2.imshow("output", self.cv_image)
        cv2.waitKey(3)


    # Classifier Function
    def classifier(self):

        # ROS view path
        src = '/home/prkizir/catkin_ws/src/recolect/scripts/ros_view.png'

        intx = raw_input("Wanna take a photo? -> ")

        if(intx == '1'):
            cv2.imwrite(src,self.cv_image)
            print('Photo Taken')

            # Load and Resize Image from Source
            img = image.load_img(src, target_size = (150, 150, 3))

            # Transform Image to Array for Model Processing
            img_tensor = image.img_to_array(img)
            img_tensor = np.expand_dims(img_tensor, axis = 0)
            img_tensor /= 255.

            print('Analysing')

            # Predict Image Class (0 Fresh, 1 Rotten)
            result = self.model.predict_classes(img_tensor)

            # Validate Prediction
            if(result[0][0] == 1):
                print('Rotten')
                #print('>>> Remove Fruit')
            else:
                print('Fresh')
                #print('Picking Fruit...')

        else:
            print('I will keep looking')



if __name__ == '__main__':
    rospy.init_node("image_query",anonymous = True)
    try:
        Query()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
