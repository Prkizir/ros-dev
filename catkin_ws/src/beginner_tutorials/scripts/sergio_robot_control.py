#!/usr/bin/env python

# Sergio Isaac Mercado Silvano
# A01020382
# Text-controlled Robot

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.st_st)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd_vel = Twist()
        self.angular_speed = 2
        self.linear_speed = 0.5
        rospy.Subscriber("recognize/output", String, self.movement)
        self.st_st()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.publisher.publish(self.cmd_vel)
            rate.sleep()

    def movement(self,data):
        if data.data == "rotate left":
            self.cmd_vel.angular.z = self.angular_speed
            self.cmd_vel.linear.x = 0

        elif data.data == "rotate right":
            self.cmd_vel.angular.z = -self.angular_speed
            self.cmd_vel.linear.x = 0

        elif data.data == "move forward":
            self.cmd_vel.linear.x = self.linear_speed
            self.cmd_vel.angular.z = 0

        elif data.data == "move backwards":
            self.cmd_vel.linear.x = -self.linear_speed
            self.cmd_vel.angular.z = 0

        elif data.data == "stop":
            self.st_st()

        else:
            pass

        print(data.data)

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
