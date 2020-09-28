#!/usr/bin/env python
import sys
import math
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
import geometry_msgs.msg
import tf
from darm_interface import *

####################################################################
################# JOHNS HOPKINS UNIVERSITY #########################
################# INTERBOTIX WX200 WITH MOVEIT #####################
# To test the interbotix wx200 with basic move up, down, left, #####
# right, front, back with calibration for position and orientation #
####################################################################


class DeconArmTest:
	def __init__(self):
		
		#Basic initialisation for moveit
        	moveit_commander.roscpp_initialize(sys.argv)
        	self.scene = moveit_commander.PlanningSceneInterface() #ns="wx200"
        	self.robot = moveit_commander.RobotCommander(robot_description="wx200/robot_description")
        	group_name = "interbotix_arm"
        	self.group = moveit_commander.MoveGroupCommander(robot_description="wx200/robot_description",ns =    "wx200",name = group_name)

		rospy.init_node('decon_test', anonymous=True, disable_signals=True)
        	self.rate = rospy.Rate(1) # 1hz
		
		#Attach cone as colliding object to avoid the need to restart the ARM
		#self.attach_object()
		
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

		#Print basic moveit, scene, robot info
		self.print_basic_interface_info()
		self.print_basic_robot_info()

	#####################################################################################
	#Code taken from moveit package
	def print_basic_interface_info(self):
        	# We can get the name of the reference frame for this robot:
        	planning_frame = self.group.get_planning_frame()
        	print "============ Reference frame: %s" % planning_frame

        	# We can also print the name of the end-effector link for this group:
        	eef_link = self.group.get_end_effector_link()
        	print "============ End effector: %s" % eef_link

        	# We can get a list of all the groups in the robot:
        	group_names = self.robot.get_group_names()
        	print "============ Robot Groups:", group_names

        	# Sometimes for debugging it is useful to print the entire state of the
        	# robot:
        	print "============ Printing robot state"
        	print self.robot.get_current_state()
        	print ""
		#####################################################################################

	def print_basic_robot_info(self):
        	print ("Starting position: ", self.start_pose)
        	curr_joint_values = self.group.get_current_joint_values()
        	curr_joint_values_in_degrees = [el*180/math.pi for el in curr_joint_values]
        	print("Current Joint Values in radians: ", curr_joint_values)
        	print("Current Joint Values in degrees: ", curr_joint_values_in_degrees)
		print("Joint info: ", self.group.get_joints())

	def attach_object(self):
		box_pose = geometry_msgs.msg.PoseStamped()
		eef_link = self.group.get_end_effector_link()
		box_pose.header.frame_id = eef_link
		box_pose.pose.orientation.w = 1.0
		box_name = 'cone'
		self.scene.add_box(box_name, box_pose, size=(0.25,0.25,0.25))
		grasping_group = 'interbotix_arm'
		touch_links = [eef_link] 
		self.scene.attach_box(eef_link, name=box_name, touch_links=touch_links) 

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
		
	def start(self):
		while not rospy.is_shutdown():
			user_input = raw_input("Press 1 to increase x, 2 to decrease x, 3 to increase y, 4 to decrease y, 5 to increase z, 6 to decrease z, h to go to start position, e to exit: ")
 			if user_input == '1':
				inc_x(self.group, 0.1)
			elif user_input == '2':
				dec_x(self.group, 0.1)
			elif user_input == '3':
				inc_y(self.group, 0.1)
			elif user_input == '4':
				dec_y(self.group, 0.1)
			elif user_input == '5':
				inc_z(self.group, 0.1)
			elif user_input == '6':
				dec_z(self.group, 0.1)
			elif user_input == 'h':
				move_to_pose(self.group, [self.start_pose.pose])
			elif user_input == 'e':
				move_to_pose(self.group, [self.start_pose.pose])
				rospy.signal_shutdown("Exit Command Received...Good Bye!")
			else:
				print("Incorrect input")
			self.rate.sleep()

if __name__ == '__main__':
    try:
        arm_test = DeconArmTest()
	arm_test.start()
	
    except rospy.ROSInterruptException:
        pass

