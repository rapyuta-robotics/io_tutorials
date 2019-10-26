#!/usr/bin/env python
""" ROS action client for PickAndPlace
"""

import rospy
import actionlib
from kyoto_common.msg import PickAndPlaceTarget, PickAndPlaceAction, PickAndPlaceGoal

if __name__ == "__main__":

    rospy.init_node('pick_and_place_action_client')

    action_client = actionlib.SimpleActionClient('pick_and_place', PickAndPlaceAction)
    rospy.loginfo("Wait for action server named : " + action_client.action_client.ns)
    action_client.wait_for_server()
    rospy.loginfo("Confirmed action server started")

    markers = []
    marker_num = rospy.get_param("marker_num", 0)
    rospy.loginfo('marker num: %s',marker_num)
    for i in range(marker_num):
        marker = PickAndPlaceTarget()
        marker_name = 'marker'+str(i)
        marker.id = rospy.get_param(marker_name+'/id')
        trans = rospy.get_param(marker_name+'/trans')
        euler = rospy.get_param(marker_name+'/euler')
        marker.trans.x = trans[0]
        marker.trans.y = trans[1]
        marker.trans.z = trans[2]
        marker.euler.x = euler[0]
        marker.euler.y = euler[1]
        marker.euler.z = euler[2]

        markers.append(marker)

    for i in range(marker_num):
      rospy.loginfo('ar_marker_%s, %s, %s',markers[i].id, markers[i].trans, markers[i].euler)

    goal = PickAndPlaceGoal()
    goal.targets = markers
    rospy.loginfo("Send goal to the action server named: " +  action_client.action_client.ns)
    action_client.send_goal(goal)
    action_client.wait_for_result()

    result = action_client.get_result().status
    if result:
        rospy.loginfo("Action is completed")
    else :
        rospy.logwarn("Planning/Execution error:" + str(result))
        exit()