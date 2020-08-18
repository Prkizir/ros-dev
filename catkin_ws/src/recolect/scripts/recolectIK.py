#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from math import pi

class IKClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  
 
        ############ CONSTANTS ################  
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("IK_node", anonymous=True) 
        rospy.Subscriber("point", Vector3, self.pose_cb) 
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.eef_link = self.move_group.get_end_effector_link()
        self.robotJoints = self.move_group.get_joints()
        self.endEffector= Pose()
        self.endEffector.orientation.x = 0.517736
        self.endEffector.orientation.y = 0.517888
        self.endEffector.orientation.z = -0.481384
        self.endEffector.orientation.w = 0.481675
        
        #********** INIT NODE **********###  
        r = rospy.Rate(0.03) #1Hz 

        while not rospy.is_shutdown():  

            r.sleep()  

    def pose_cb(self, point): 
      self.endEffector.position = point
      self.move_group.set_pose_target(self.endEffector)
      self.move_group.go(wait=True)    
      self.move_group.stop()
      self.move_group.clear_pose_targets() 
      scale = 1
      waypoints = []
      wpose = self.move_group.get_current_pose().pose
      wpose.position.z -= scale * 0.25
      waypoints.append(copy.deepcopy(wpose))
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0) 
      self.move_group.execute(plan, wait=True)
      self.move_group.stop()


    def cleanup(self):  
      return

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   

    try:  

        IKClass()  

    except:  

        rospy.logfatal("ik_node died")  