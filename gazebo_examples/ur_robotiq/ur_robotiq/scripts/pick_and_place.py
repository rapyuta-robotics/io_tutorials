#!/usr/bin/env python
import sys
import copy
import rospy
import numpy as np
from std_msgs.msg import String, Bool
import geometry_msgs.msg
import actionlib
from ur_robotiq_msgs.msg import PickAndPlaceAction, PickAndPlaceResult
import tf
from tf.transformations import *
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

STEP_BY_STEP = False#True#
SOOTBALL_HEIGHT = 0.865
APPROACH_HEIGHT = 0.08
QR_CODE_SEARCH_TIMEOUT = 10.0

def debug_input():
	if(STEP_BY_STEP):
		raw_input("Press Enter to continue...")

class pickAndPlace(object):
	
	def __init__(self):

		#moveit initialize
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface',
		                anonymous=True)

		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()
		group_name = "ur3"
		self._group = moveit_commander.MoveGroupCommander(group_name)
		self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)

		#robotiq initialization
		self._robotiq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', 
		                      outputMsg.Robotiq2FGripper_robot_output,
		                      queue_size=10)

		self.robotiq_reset()
		self.robotiq_close()
		self.robotiq_open()

		#camera
		self._use_camera = rospy.get_param('~camera')

		self._as = actionlib.SimpleActionServer('pick_and_place', PickAndPlaceAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, msg):
		result = PickAndPlaceResult()
		result.status = True
		result.picked_num = 0
		for target in msg.targets:
			trans = [0]*3
			euler = [0]*3
			trans[0] = target.trans.x
			trans[1] = target.trans.y
			trans[2] = target.trans.z
			euler[0] = target.euler.x
			euler[1] = target.euler.y
			euler[2] = target.euler.z
			if not node.pickAndPlace(target.id, trans, euler):
				result.status = False
				self._as.set_aborted(result)
				return
			else:
				result.picked_num += 1

		self._as.set_succeeded(result)

	def robotiq_reset(self):
		#reset
		command = outputMsg.Robotiq2FGripper_robot_output()
		command.rACT = 0 #reset
		# self._robotiq_pub.publish(command)
		rospy.sleep(1.0)
		self._robotiq_pub.publish(command)
		rospy.sleep(1.0)
		
	def robotiq_command(self, cmd):
		rospy.loginfo('Robotiq : %s', cmd)
		command = outputMsg.Robotiq2FGripper_robot_output();
		command.rACT = 1
		command.rGTO = 1
		command.rSP  = 255
		command.rFR  = 150
		command.rPR = cmd
		self._robotiq_pub.publish(command)
		rospy.sleep(0.5)

	def robotiq_close(self):
		self.robotiq_command(255)

	def robotiq_open(self):
		self.robotiq_command(0)

	def wait_ar_marker(self, marker_name):
		listener = tf.TransformListener()
		broadcaster = tf.TransformBroadcaster()
		searching = True

		rospy.loginfo('searching ar marker')
		rate = rospy.Rate(10.0)
		start_time = rospy.Time.now()
		if self._use_camera:
			while(searching):
			    try:

			        #tf.waitForTransform('/world', '/rgb_camera_link', rospy.Time(0), rospy.Duration(3.0))
			        (trans,rot) = listener.lookupTransform('/world', '/'+marker_name, rospy.Time(0))
			        euler = list(tf.transformations.euler_from_quaternion(rot))

			        #sanity check
	    			if (trans[0] > -0.2 or trans[0] < -1.0 or
	        			trans[1] > 0.5 or trans[1] < -0.5 or	 
	        			trans[2] > 1.0 or trans[2] < -0.5 ):
	    				continue
	    			else:
				        trans[2] = SOOTBALL_HEIGHT #center of cubes
				        euler[0] = 0.0
				        euler[1] = 0.0
				        orientation = quaternion_from_euler(0, 0, euler[2])
				        broadcaster.sendTransform(trans, orientation, rospy.Time.now(), marker_name+'_target', 'world')
				        searching = False
				        rospy.loginfo('Find '+marker_name+': %s %s', trans, euler)
				        return True, trans, euler
			        
			    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			    	if rospy.Time.now() - start_time > rospy.Duration(QR_CODE_SEARCH_TIMEOUT):
			    		return False, None, None
			    	rate.sleep()


		else:
			return True, [-0.65, 0.0, 0.8], [0,0,1.5707] #dummy marker position

	def goto(self, goal):
		result = True
		rospy.loginfo('Goto : %s', goal)
		self._group.set_pose_target(goal)
		#execute twice
		result = self._group.go(wait=True)
		self._group.stop()
		rospy.sleep(1.0)
		result = self._group.go(wait=True)
		self._group.stop()
		rospy.sleep(0.5)
		self._group.clear_pose_targets()

		rospy.loginfo(result)
		return result

	def cartesian(self, x, y, z):
		rospy.loginfo('Cartesian : %s, %s, %s', x, y, z)
		waypoints = []
		wpose = self._group.get_current_pose().pose
		wpose.position.x += x  
		wpose.position.y += y 
		wpose.position.z += z 
		waypoints.append(copy.deepcopy(wpose))
		(plan, fraction) = self._group.compute_cartesian_path(
		                                   waypoints,   # waypoints to follow
		                                   0.01,        # eef_step
		                                   0.0)         # jump_threshold
		self._group.execute(plan, wait=True)
		# rospy.sleep(1.0)
		# self._group.execute(plan, wait=True)
		# self._group.clear_pose_targets()

	def pick(self, target_trans, target_euler):
		pick_goal = geometry_msgs.msg.Pose()
		q = quaternion_from_euler(0.0, -4*np.pi/4.0, target_euler[2])
		pick_goal.orientation.x = q[0]
		pick_goal.orientation.y = q[1]
		pick_goal.orientation.z = q[2]
		pick_goal.orientation.w = q[3]

		pick_goal.position.x = target_trans[0] #+ 0.1
		pick_goal.position.y = target_trans[1]
		pick_goal.position.z = target_trans[2] + APPROACH_HEIGHT

		result = True
		self.robotiq_open()
		debug_input()
		result = self.goto(pick_goal)
		if not result:
			return result
		debug_input()
		self.cartesian(0,0,-APPROACH_HEIGHT)
		debug_input()
		self.robotiq_close()
		debug_input()
		self.cartesian(0,0, APPROACH_HEIGHT)

		return result

	def place(self, target_trans, target_euler):
		place_goal = geometry_msgs.msg.Pose()
		q = quaternion_from_euler(0.0, -4*np.pi/4.0, target_euler[2])
		place_goal.orientation.x = q[0]
		place_goal.orientation.y = q[1]
		place_goal.orientation.z = q[2]
		place_goal.orientation.w = q[3]

		place_goal.position.x = target_trans[0] #+ 0.1
		place_goal.position.y = target_trans[1]
		place_goal.position.z = target_trans[2] + APPROACH_HEIGHT

		result = self.goto(place_goal)
		if not result:
			return result
		debug_input()
		self.cartesian(0,0,-APPROACH_HEIGHT)
		debug_input()
		self.robotiq_open()
		debug_input()
		self.cartesian(0,0, APPROACH_HEIGHT)

		return result

	def pickAndPlace(self, ar_marker_num, place_trans, place_euler):
		marker_base = 'ar_marker_'
		result, trans, euler = self.wait_ar_marker(marker_base+str(ar_marker_num))
		if not result:
			return result
		debug_input()
		result = self.pick(trans, euler)
		if not result:
			return result
		debug_input()
		result = self.place(place_trans, place_euler)
		return result

if __name__ == '__main__':
    node = pickAndPlace()
    rospy.spin()