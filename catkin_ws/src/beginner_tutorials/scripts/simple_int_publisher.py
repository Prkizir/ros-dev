#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('number_topic', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    num = 0;
    while not rospy.is_shutdown():
        rospy.loginfo("Num: %d" % num)
        pub.publish(num)
        num += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
