#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from goal_pose import MoveGroupPythonInterfaceTutorial

def callback(data):
    rospy.loginfo("log", data.data)
    
def to_pose():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    MG = MoveGroupPythonInterfaceTutorial('to_pose')

    #rospy.init_node('to_pose', anonymous=True)

    #Pose=rospy.Subscriber("pose", geometry_msgs.msg.Pose, callback)
    MG.go_to_pose_goal(0.1,-0.2,0.25)

    



    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    try:
        to_pose()
    except rospy.ROSInterruptException:
        pass