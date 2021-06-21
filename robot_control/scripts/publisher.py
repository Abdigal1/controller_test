#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64

def talkers():
    pub1 = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talkers', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    i = 0.0
    while not rospy.is_shutdown():
        pub1.publish(1.5*math.sin(i/10))
        pub2.publish(0.7*math.sin(i/10))
        pub3.publish(0.5*math.sin(i/10))
        i+=1.0
        if i==31415:
            i = 0.0

        rate.sleep()


if __name__ == '__main__':
    try:
        talkers()
    except rospy.ROSInterruptException:
        pass