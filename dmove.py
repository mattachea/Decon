#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String, Bool, Float32MultiArray
import math
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion, rotation_matrix
from nav_msgs.msg import Odometry
import tf
import sys

class DeconNavigator:
	def __init__(self):
		rospy.init_node('decon_move')
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client.wait_for_server()
		self.move_rel_command_topic = rospy.Subscriber('move_rel_msg', Float32MultiArray, self.move_rel_callback)	
		self.move_rel_status_topic = rospy.Publisher('move_rel_status_msg', Bool, queue_size=1)
		self.move_abs_command_topic = rospy.Subscriber('move_abs_msg', Float32MultiArray, self.move_abs_callback)	
		self.move_abs_status_topic = rospy.Publisher('move_abs_status_msg', Bool, queue_size=1)
		
		self.current_pose = None
		#self.get_pose = rospy.Subscriber('odom', Odometry, self.get_pose_callback)
		self.tf_listener = tf.TransformListener()
		self.command = None
		rospy.loginfo("[Decon] Initiating Move...")
		rospy.spin()

	#def get_pose_callback(self, response):
	#	self.current_pose = response.pose.pose
			
	def get_current_position(self):
		(position, rotation) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		self.current_pose = Pose()
		self.current_pose.position.x = position[0]
		self.current_pose.position.y = position[1] 
		self.current_pose.position.z = position[2] 
		self.current_pose.orientation.x = rotation[0]
		self.current_pose.orientation.y = rotation[1]
		self.current_pose.orientation.z = rotation[2]
		self.current_pose.orientation.w = rotation[3]


	def move_rel_callback(self, response):
		rospy.loginfo("[Decon] Move request received relative to current position...")
		rel_position = response.data
		
		self.get_current_position()	
		print("TEMP Command: ", rel_position)
		

		if len(rel_position) == 3:
			self.command = "rel"
			print("TEMP CURRENT: ", self.current_pose)	
			quaternion = (	self.current_pose.orientation.x,
					self.current_pose.orientation.y,
					self.current_pose.orientation.z,
					self.current_pose.orientation.w)
		
                	euler_angle = euler_from_quaternion(quaternion)				# not sure if this is the correct way of converting from quaternion to euler...
        		#angle = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))       # the angle is a yaw (not sure if this is correct ^ continuation of conversion)
			angle = euler_angle[2]
        		rel_x = rel_position[0] * math.cos(angle) + rel_position[1] * math.sin(angle)
        		rel_y = rel_position[0] * math.sin(angle) + rel_position[1] * math.cos(angle)
       			abs_x = self.current_pose.position.x + rel_x
       			abs_y = self.current_pose.position.y + rel_y
			#if rel_position[0] != 0:
			#	abs_x = self.current_pose.position.x + rel_position[0] * math.cos(angle)
			#	abs_y = self.current_pose.position.y + rel_position[0] * math.sin(angle)
			#elif rel_position[1] != 0:
			#	abs_x = self.current_pose.position.x + rel_position[1] * math.sin(angle)
			#	abs_y = self.current_pose.position.y + rel_position[1] * math.cos(angle)
				

			print("TEMP AFTER: ", abs_x, abs_y)
       			self.movebase_client([abs_x, abs_y, self.current_pose.position.z, self.current_pose.orientation])
                else:
                	self.move_rel_status_topic.publish(False)

	def move_abs_callback(self, response):
		rospy.loginfo("[Decon] Request for move to absolute location received...")
		abs_position = list(response.data)
		if len(abs_position) == 4:
			self.command = "abs"
			quaternion = quaternion_from_euler(0.0, 0.0, abs_position[3]*math.pi/180, axes='sxyz')
			abs_position[3] = Quaternion(*quaternion)
			self.movebase_client(abs_position)
		else:
			self.move_abs_status_topic.publish(False)


	def active_cb(self):
		rospy.loginfo("Goal pose is now being processed by the Action Server...")
 
	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for goal pose received")

	def done_cb(self, status, result):
		shutdown = False

		if status == 2:
			rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")
			ret = False
		elif status == 3:
			rospy.loginfo("Goal pose reached")
			ret = True
		elif status == 4:
			rospy.loginfo("Goal pose was aborted by the Action Server")
			ret = False
			shutdown = True
			
			self.move_abs_status_topic.publish(False)                        
		elif status == 5:
			rospy.loginfo("Goal pose has been rejected by the Action Server")
			ret = False
			shutdown = True
		elif status == 8:
			rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")	
			ret = False

		if self.command == "abs":
			self.move_abs_status_topic.publish(ret)
		elif self.command == "rel":
			self.move_rel_status_topic.publish(ret)
		self.command = None
		
		if shutdown is True:
			rospy.signal_shutdown("Goal pose aborted, shutting down!")

	def movebase_client(self, abs_position):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = abs_position[0]
		goal.target_pose.pose.position.y = abs_position[1]
		goal.target_pose.pose.position.z = abs_position[2]
		goal.target_pose.pose.orientation = abs_position[3]
		rospy.loginfo("Sending goal pose to Action Server")
		rospy.loginfo(str(goal.target_pose.pose))
		self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

if __name__ == '__main__':
	try:
		dnav = DeconNavigator()
	except rospy.ROSInterruptException:
		log_string = "[Decon] Decon Navigator exiting at %s!" % rospy.get_time()	
		rospy.loginfo(log_string)


