#!/usr/bin/env python

# Sergio Isaac Mercado Silvano
# A01020382
# Closest-Object-Following Robot

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.st_st)

        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd_vel = Twist()
        self.angular_speed = 0.5
        self.linear_speed = 0.155

        rospy.Subscriber("base_scan", LaserScan, self.movement)
        self.ranges = []
        self.i = 0
        self.min_a = 0
        self.min_r = 0
        self.r_flag = True;
        self.l_flag = False;


        self.st_st()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.publisher.publish(self.cmd_vel)
            rate.sleep()

    def movement(self,data):
        self.ranges = data.ranges
        self.min_r = min(self.ranges)

        self.i = self.ranges.index(self.min_r)

        self.min_a = data.angle_increment * self.i * 180.0/math.pi

        if(self.min_a < 170.0):
            self.cmd_vel.angular.z = -self.angular_speed
            self.cmd_vel.linear.x = 0

        elif(self.min_a > 190.0):
            self.cmd_vel.angular.z = self.angular_speed
            self.cmd_vel.linear.x = 0

        else:
            self.cmd_vel.angular.z = 0
            self.cmd_vel.linear.x = 0

            if(self.min_r > 0.6):
                self.cmd_vel.angular.z = 0;
                self.cmd_vel.linear.x = self.linear_speed

            else:
                self.st_st()

        if(self.min_r == np.inf):
            self.st_st()

        print(self.min_r)
        print(self.min_a)

    def st_st(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0

        self.publisher.publish(self.cmd_vel)

if __name__ == '__main__':
    rospy.init_node("Sergio_Robot_Controller",anonymous = True)
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
