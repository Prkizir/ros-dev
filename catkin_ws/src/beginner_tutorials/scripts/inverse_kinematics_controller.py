#!/usr/bin/env python

# Sergio Isaac Mercado Silvano - A01020382
# Franco Garcia Pedregal - A01273527
# Text-controlled robotic arm with moement sequences

# Import Python Libraries

import rospy
import string
import math

# Import ROS Libraries

from std_msgs.msg import String     # Just in case
from std_msgs.msg import Float64    # Float64 to Publish Angles
from geometry_msgs.msg import Vector3   # Vector3 To Receive X,Y,Z Vector

# Controller Class
class Controller():

    # Initial State
    def __init__(self):

        # Joint Publishers
        self.joint1 = rospy.Publisher('robot/joint1/command', Float64, queue_size=10)
        self.joint2 = rospy.Publisher('robot/joint2/command', Float64, queue_size=10)
        self.joint3 = rospy.Publisher('robot/joint3/command', Float64, queue_size=10)

        # Vector3 Subscriber
        rospy.Subscriber("geometry_msgs/Vector3", Vector3, self.movement)

        # Initial Angle Positions
        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 0

        # Table and Base Height (Z-Offset)
        self.z_offset = 1.5381

        # Arm Lengths
        self.arm1_length = 0.6
        self.arm2_length = 0.6

        # 10 Hz
        rate = rospy.Rate(10)

        # Continously Publish to Joints
        while not rospy.is_shutdown():
            self.joint1.publish(self.angle1)
            self.joint2.publish(self.angle2)
            self.joint3.publish(self.angle3)

            rate.sleep()

    # Vector3 Callback Function
    def movement(self,data):

        # Reset Angle Positions
        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 0

        # Obtain X,Y,Z Coordinates from Vector3 Subscriber
        x1 = data.x
        x2 = data.y
        x3 = data.z

        # Write Angles
        if(x1 == 0 and x2 == 0 and x3 == 0):
            self.angle1 = 0
            self.angle2 = 0
            self.angle3 = 0
        else:
            # Transform Z Coordinates with Offset
            x3 = data.z - self.z_offset

            # Calculate Inner Alpha and Beta Values for Validation
            beta_test = (-x1**2 - x2**2 - x3**2 + self.arm1_length**2 + self.arm2_length**2)/(2 * self.arm1_length * self.arm2_length)
            alpha_test = (x1**2 + x2**2 + x3**2 + self.arm1_length**2 - self.arm2_length**2)/(2 * self.arm1_length * math.sqrt(x1**2 + x2**2 + x3**2))

            # Validate Alpha and Beta Testing Values
            #   to avoid Calculation Errors
            if((beta_test >= -1 and beta_test <= 1) and (alpha_test >= -1 and alpha_test <= 1)):

                # Base Joint
                theta1 = math.atan2(x2, x1)
                o = math.degrees(theta1)
                self.angle1 = theta1
                #print(o)
                print(theta1)

                rospy.sleep(2)

                # Link 1-2 Joint
                beta = math.degrees(math.acos(beta_test))
                beta = beta - 180
                theta3 = math.radians(beta)
                self.angle3 = theta3
                #print(beta)
                print(theta3)

                rospy.sleep(2)

                # Base-Link 1 Joint
                alpha = math.acos(alpha_test)
                theta2 = math.atan2(x3,math.sqrt(x1**2 + x2**2)) + alpha
                self.angle2 = theta2
                #print(math.degrees(alpha))
                print(theta2)

            else:
                # Validation Failed
                print('Unreachable Point')

# Main
if __name__ == '__main__':
    # Initialise Node as "robot_controller"
    #   allowing Multiple Instances
    rospy.init_node("robot_controller",anonymous = True)
    try:
        # Call Node
        Controller()
    except rospy.ROSInterruptException:
        pass
