#!/usr/bin/env python

#Sergio Isaac Mercado Silvano
#A01020382
#base_scan subscriber

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

class BaseScanSubClass():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("base_scan", LaserScan, self.min_p_cb)
        rospy.Subscriber("base_scan", LaserScan, self.min_a_cb)

        self.ranges = []
        self.i = 0

        self.min_r = 0
        self.min_a = 0

        r = rospy.Rate(2)

        print "Node initialized at 2hz"

        while not rospy.is_shutdown():
            print "Minimum value in rages: [" + str(self.min_r) + "]\n"
            print "The angle for minimum value: [" + str(self.min_a) + "]\n"
            r.sleep()

    def min_a_cb(self, msg):
        self.i = self.ranges.index(self.min_r)
        self.min_a = msg.angle_increment * self.i * 180/math.pi
        pass

    def min_p_cb(self, msg):
        self.ranges = msg.ranges
        self.min_r = min(self.ranges)
        pass

    def cleanup(self):
        pass

if __name__ == "__main__":
    rospy.init_node("base_scan_sub", anonymous=True)
    try:
        BaseScanSubClass()
    except:
        rospy.logfatal("base_scan_sub died")
