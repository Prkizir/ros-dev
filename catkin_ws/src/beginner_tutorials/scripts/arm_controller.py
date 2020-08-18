#!/usr/bin/env python

# Sergio Isaac Mercado Silvano
# A01020382
# Text-controlled robotic arm with moement sequences

import rospy
import string
from std_msgs.msg import String
from std_msgs.msg import Float64

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.home)

        self.joint1 = rospy.Publisher('robot/joint1/command', Float64, queue_size=10)
        self.joint2 = rospy.Publisher('robot/joint2/command', Float64, queue_size=10)
        self.joint3 = rospy.Publisher('robot/joint3/command', Float64, queue_size=10)
        self.joint4 = rospy.Publisher('robot/joint4/command', Float64, queue_size=10)

        self.lgrip = rospy.Publisher('robot/lgrip/command', Float64, queue_size=10)
        self.rgrip = rospy.Publisher('robot/rgrip/command', Float64, queue_size=10)

        rospy.Subscriber("recognize/output", String, self.movement)

        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 9
        self.angle4 = 0

        self.linear_l = 0
        self.linear_r = 0

        self.ready = False

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if(self.ready == True):
                self.joint1.publish(self.angle1)
                self.joint2.publish(self.angle2)
                self.joint3.publish(self.angle3)
                self.joint4.publish(self.angle4)

                self.lgrip.publish(self.linear_l)
                self.rgrip.publish(self.linear_r)

            rate.sleep()

    def movement(self,data):
        if(data.data == "sequence 1"):
            self.sequence1()
        elif(data.data == "sequence 2"):
            self.sequence2()
        elif(data.data == "sequence 3"):
            self.sequence3()
        elif(data.data  == "home"):
            self.home()
        else:
            print("No such sequence")

    def sequence1(self):
        print("Setting: SEQUENCE 1")
        self.ready = True

        self.angle1 = -1
        rospy.sleep(2)
        self.angle2 = 1
        rospy.sleep(2)
        self.angle3 = -1
        rospy.sleep(2)

        self.home()

        self.ready = False

    def sequence2(self):
        print("Setting: SEQUENCE 2")
        self.ready = True

        self.angle1 = 2
        rospy.sleep(2)
        self.angle4 = 1.5
        rospy.sleep(2)

        self.home()

        self.ready = False

    def sequence3(self):
        print("Setting: SEQUENCE 3")
        self.ready = True

        self.angle2 = 0.2
        rospy.sleep(2)
        self.angle3 = -1.5
        rospy.sleep(2)
        self.angle4 = 1.6
        rospy.sleep(3)
        self.linear_l = -0.6
        self.linear_r = -0.6
        rospy.sleep(1)
        self.angle3 = -1
        rospy.sleep(2)
        self.angle1 = 0.7
        rospy.sleep(2)
        self.angle4 = 0
        rospy.sleep(1)
        self.angle3 = -1.5
        rospy.sleep(2)
        self.linear_l = 0.4
        self.linear_r = 0.4
        rospy.sleep(1)

        self.home()

        self.ready = False

    def home(self):
        print("Setting: HOME")
        self.ready = True

        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 0
        self.angle4 = 0
        self.linear_l = 0
        self.linear_r = 0
        rospy.sleep(2)

        self.ready = False


if __name__ == '__main__':
    rospy.init_node("robot_controller",anonymous = True)
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
