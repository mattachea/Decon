#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Bool, Float32MultiArray
import tf
from moveit_msgs.msg import OrientationConstraint, Constraints
import math
from darm_interface import *

####################################################################
################# JOHNS HOPKINS UNIVERSITY #########################
################# INTERBOTIX WX200 WITH MOVEIT #####################
################# ARM node for DECON ###############################
####################################################################

class DeconARM():
	def __init__(self):

		#Topics that this node will interact with decon controller 
		self.arm_topic = rospy.Subscriber('arm_msg', Float32MultiArray, self.arm_cb)	
		self.arm_status_topic = rospy.Publisher('arm_status_msg', Bool, queue_size=1)
	
		#Basic initialisation for moveit
		moveit_commander.roscpp_initialize(sys.argv)
		self.scene = moveit_commander.PlanningSceneInterface() #ns="wx200"
        	self.robot = moveit_commander.RobotCommander(robot_description="wx200/robot_description")
        	group_name = "interbotix_arm"
        	self.group = moveit_commander.MoveGroupCommander(robot_description="wx200/robot_description",ns =    "wx200",name = group_name)

		rospy.init_node('decon_arm', anonymous=True, disable_signals=True)
		self.rate = rospy.Rate(1)
		
		#Save the starting pose to return to  
		self.start_pose = self.group.get_current_pose()
		self.start_pose.pose.position.x = 0.1
		self.start_pose.pose.position.y = 0.0
		self.start_pose.pose.position.z = 0.46
		
		quat = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(-28), math.radians(0))
		self.start_pose.pose.orientation.x = quat[0]
		self.start_pose.pose.orientation.y = quat[1]
		self.start_pose.pose.orientation.z = quat[2]
		self.start_pose.pose.orientation.w = quat[3]

		move_to_pose(self.group, [self.start_pose.pose])

		#Constraint to keep orientaion aligned with turtlebot
		self.constraints = Constraints()	
		self.set_orientation_constraints()
		self.group.set_path_constraints(self.constraints)
	
		rospy.loginfo("[Decon] Initiating ARM WX200...")
		rospy.spin()

	def set_orientation_constraints(self):
		oc = OrientationConstraint()
		oc.header.frame_id = 'world'
		oc.link_name = 'ee_arm_link'
		quat = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(-28), math.radians(0))
		oc.orientation.x = quat[0]
		oc.orientation.y = quat[1]
		oc.orientation.z = quat[2]
		oc.orientation.w = quat[3]
		oc.absolute_x_axis_tolerance = 0.5
		oc.absolute_y_axis_tolerance = 0.1
		oc.absolute_z_axis_tolerance = 0.5
		oc.weight = 1.0
		self.constraints.orientation_constraints.append(oc)

	def arm_cb(self, request):
		rospy.loginfo("[Decon] Request to move arm recieved...")
		if len(request.data) != 2 and len(request.data) != 3:
			self.arm_status_topic.publish(False)
			return

		if request.data == (0.0,0.0):
			move_to_pose(self.group, [self.start_pose.pose])
			self.arm_status_topic.publish(True)
			return
			
		[arm_x_offset, arm_y_offset, arm_z_offset] = request.data	
		move(self.group, arm_x_offset, 0, arm_z_offset)	
		self.arm_status_topic.publish(True)
		return
		
if __name__ == '__main__':
	try:
		dcontroller = DeconARM()
	except rospy.ROSInterruptException:
		log_string = "[Decon] Decon Arm exiting at %s!" % rospy.get_time()	
		rospy.loginfo(log_string)

