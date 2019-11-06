#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('demo_app')

# Time sync
rospy.wait_for_message('/clock', Time)
rospy.loginfo('Time synced with Simulator')

# Load initial pose
init_quaternion = quaternion_from_euler(0.0, 0.0, float(rospy.get_param('/init_yaw')))

initalpose = PoseWithCovarianceStamped()
initalpose.pose.pose.position.x = float(rospy.get_param('/init_x'))
initalpose.pose.pose.position.y = float(rospy.get_param('/init_y'))
initalpose.pose.pose.position.z = 0.0
initalpose.pose.pose.orientation.x = init_quaternion[0]
initalpose.pose.pose.orientation.x = init_quaternion[1]
initalpose.pose.pose.orientation.x = init_quaternion[2]
initalpose.pose.pose.orientation.x = init_quaternion[3]

initpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

# Load locations 
locations = []
locations_num = rospy.get_param("~location_num", 0)
rospy.loginfo("locations num: %s", locations_num)

loc_x = []
loc_y = [] 
loc_yaw = []

for i in range(locations_num):
    location_name = '~loc'+str(i)    
    loc_x.append(rospy.get_param(location_name+'/x'))
    loc_y.append(rospy.get_param(location_name+'/y'))
    loc_yaw.append(rospy.get_param(location_name+'/yaw'))

# Movebase client
move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
rospy.loginfo("Waiting MoveBase action server")
if move_base_client.wait_for_server(timeout = rospy.Duration(60.0)):
    rospy.loginfo("Confirmed MoveBase action server is running")
    # Send initial pose
    initpose_pub.publish(initalpose)
    
    # Send goal
    for i in range(locations_num):
        quaternion = quaternion_from_euler(0.0, 0.0, loc_yaw[i])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = loc_x[i]
        goal.target_pose.pose.position.y = loc_y[i]
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo("send goal to : x:%s, y:%s, yaw:%s", loc_x[i], loc_y[i], loc_yaw[i])
        move_base_client.send_goal(goal)
        result = move_base_client.wait_for_result(timeout = rospy.Duration(30.0))
        if not result:
            rospy.logerr("Exceeded waiting time for result!")
        else:
            rospy.loginfo("Reached goal!")

else:
    rospy.logerr("MoveBase action server is not running")
    exit()
