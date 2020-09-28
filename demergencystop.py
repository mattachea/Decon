#!/usr/bin/env python
__author__ = 'matt'
import rospy
import json
import sys
import time
import math
from std_msgs.msg import String, Bool, Float32MultiArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class EmergencyStop:
	def __init__(self):
		rospy.init_node('emergency_stop', anonymous=True, disable_signals=True)

		self.move_base = actionlib.SimpleActionClient("move_base" ,MoveBaseAction) #create move base actionlib client
		while not self.move_base.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown(): # wait for move_base server to be ready
			rospy.loginfo(  "Waiting for Move Base Server")

		self.led_command_topic = rospy.Publisher('led_msg', Bool, queue_size=1) 				# UV LEDs
		self.pump_command_topic = rospy.Publisher('pump_msg', Bool, queue_size=1)				# Pump
		# self.arm_command_topic = rospy.Publisher('arm_msg', Float32MultiArray, queue_size=1)	# Arm

		self.emergency_topic = rospy.Subscriber('emergency_msg', Bool, self.stopAll) #calls stopAll() when message is received


	def move_arm(self, position): # moves arm to the given position
		rospy.loginfo("[Decon] Starting to position arm...")
		request = Float32MultiArray()
		request.data = position
		self.arm_command_topic.publish(request)

	def stopAll(self, data):
		if (data.data == True):
			self.move_base.cancel_all_goals()        # cancel all navigation goals
			self.led_command_topic.publish(False)    # turn off leds
			self.pump_command_topic.publish(False)   # turn off pump
			# self.move_arm([0.0,0.0]) 				 # move arm home #maybe change to stay still?
			rospy.loginfo("[Decon] Emergency: Stopping all functions...")
				
if __name__ == '__main__':
	try:
		demergencystop = EmergencyStop()
        
	except rospy.ROSInterruptException:
		log_string = "[Decon] Decon E-Stop exiting at %s!" % rospy.get_time()	
		rospy.loginfo(log_string)