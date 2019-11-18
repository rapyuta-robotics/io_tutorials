#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

rospy.loginfo('waiting service')
rospy.init_node("spawn_products_in_bins")
rospy.wait_for_service("/gazebo/spawn_urdf_model")
spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
rospy.loginfo('service started')

#load params
urdf = rospy.get_param('/robot_description')
x = rospy.get_param('/init_x', 0.0)
y = rospy.get_param('/init_y', 0.0)
yaw = rospy.get_param('/init_yaw', 0.0)
q = quaternion_from_euler(0,0,yaw)

init_pose = Pose()
init_pose.position.x = x
init_pose.position.y = y
init_pose.orientation.x = q[0]
init_pose.orientation.y = q[1]
init_pose.orientation.z = q[2]
init_pose.orientation.w = q[3]

#call service
if(spawn_model('Turtle', urdf, rospy.get_namespace(), init_pose, '')):
    rospy.loginfo('spawn model succeeded')
else:
    rospy.logerr('spawn model failed')