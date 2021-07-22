#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf import TransformBroadcaster
from rospy import Time
import numpy as np


#rospy.init_node('odom_pub')

#odom_pub=rospy.Publisher ('/my_odom', Odometry)
while not rospy.is_shutdown():
    rospy.init_node('my_broadcaster')
    rospy.wait_for_service ('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    #odom=Odometry()
    #header = Header()
    #header.frame_id='/odom'

    model = GetModelStateRequest()
    model.model_name='robot'
    
    r = rospy.Rate(50)


    
    b = TransformBroadcaster()
    result = get_model_srv(model)
    v1 = result.pose.position
    v2 = result.pose.orientation
    
    translation = (v1.x+9.3, v1.y-8.5, 0.0)
    rotation = (v2.x, v2.y, v2.z, v2.w)
    b.sendTransform(translation, rotation, Time.now(), 'base_arm', '/odom')



    r.sleep()


