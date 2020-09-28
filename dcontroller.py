#!/usr/bin/env python
import rospy
import json
import sys
import time
import math
from std_msgs.msg import String, Bool, Float32MultiArray

HANDSHAKE_FLAG, DETECTION_FLAG, MOVE_CLOSER_FLAG, ARM_FLAG, LOCATION_FLAG = 0, 1, 2, 3, 4

class DeconController:
	def __init__(self, obj_locations):
		
		self.obj_locations = obj_locations
		self.next_location_idx = 0
		self.state = "MOVE2NEXT"

		self.emergency_topic = rospy.Publisher('emergency_msg', Bool, queue_size=1)

		self.arm_command_topic = rospy.Publisher('arm_msg', Float32MultiArray, queue_size=1)
		self.arm_status_topic = rospy.Subscriber('arm_status_msg', Bool, self.arm_cb)

		self.move_closer_command_topic = rospy.Publisher('move_rel_msg', Float32MultiArray, queue_size=1)
		self.move_closer_status_topic = rospy.Subscriber('move_rel_status_msg', Bool, self.move_closer_cb)
		
		self.move2next_location_command_topic = rospy.Publisher('move_abs_msg', Float32MultiArray, queue_size=1)
		self.move2next_location_status_topic = rospy.Subscriber('move_abs_status_msg', Bool, self.move2next_location_cb)
	
		self.led_command_topic = rospy.Publisher('led_msg', Bool, queue_size=1)
		self.pump_command_topic = rospy.Publisher('pump_msg', Bool, queue_size=1)

		self.detection_command_topic = rospy.Publisher('detect_msg', Bool, queue_size=1)
		self.detection_status_topic = rospy.Subscriber('detect_status_msg', Float32MultiArray, self.detection_cb)

		rospy.init_node('decon_controller', anonymous=True, disable_signals=True)

		self.response_signal = [None, None, None, None, None]
		self.disinfection_completed = False
		
		self.rate = rospy.Rate(1) #Repeat detect and disinfect every 1 second

	def detect(self):
		rospy.loginfo("[Decon] Starting detection process...")
		self.detection_command_topic.publish(True)

	def move2next_location(self):
		if self.next_location_idx >= len(self.obj_locations):
			rospy.loginfo("[Decon] All locations covered...")
			return False
		next_location = self.obj_locations[self.next_location_idx]
		rospy.loginfo("[Decon] Moving to location number " + str(self.next_location_idx + 1) + "/" + str(len(self.obj_locations))+"...")

		request = Float32MultiArray()
		request.data = next_location
		self.move2next_location_command_topic.publish(request)
		self.next_location_idx += 1

	def move_closer(self):
		rospy.loginfo("[Decon] Starting to move...")
		request = Float32MultiArray()
		request.data = self.move_position
		self.move_closer_command_topic.publish(request)
	
	def move_arm(self, position):
		rospy.loginfo("[Decon] Starting to position arm...")
		request = Float32MultiArray()
		request.data = position
		self.arm_command_topic.publish(request)

	def detection_cb(self, data):
		self.response_signal[DETECTION_FLAG] = data.data
		if self.response_signal[DETECTION_FLAG] == (1,) and self.response_signal[HANDSHAKE_FLAG] == False:
			self.response_signal[HANDSHAKE_FLAG] = True
			self.response_signal[DETECTION_FLAG] = None
			rospy.loginfo("[Decon] Detection handshake completed...")
	
	def move2next_location_cb(self, data):
		self.response_signal[LOCATION_FLAG] = data.data

	def arm_cb(self, data):
		self.response_signal[ARM_FLAG] = data.data
		
	def move_closer_cb(self, data):
		self.response_signal[MOVE_CLOSER_FLAG] = data.data

	def turn_led_on(self):
		rospy.loginfo("[Decon] Turning led on...")
		self.led_command_topic.publish(True)

	def turn_led_off(self):
		rospy.loginfo("[Decon] Turning led off...")
		self.led_command_topic.publish(False)

	def activating_pump(self):
		rospy.loginfo("[Decon] Activating pump (It will turn off on its own after 30 seconds)...")
		self.pump_command_topic.publish(True)
	
	def timer_for(self, seconds):
		for remaining in range(seconds, 0, -1):
    			sys.stdout.write("\r")
    			sys.stdout.write("{:2d} seconds remaining.".format(remaining)) 
    			sys.stdout.flush()
    			time.sleep(1)

		sys.stdout.write("\rComplete!            \n")

	def disinfect(self):
		self.turn_led_on()
		self.timer_for(30)
		self.turn_led_off()
		self.activating_pump()
		self.timer_for(5)

	def handshake_detection(self):
		if self.response_signal[HANDSHAKE_FLAG] is not True:
			rospy.loginfo("[Decon] Initiating handshake...")
			self.detection_command_topic.publish(False)
		else:
			rospy.loginfo("[Decon] Handshake already established...")
	
	def reset_handshake(self):
		rospy.loginfo("[Decon] Resetting handshake...")
		self.response_signal[HANDSHAKE_FLAG] = False

	def wait_for (self, flag_idx):
		try:
			timer = -1
			while self.response_signal[flag_idx] is None:
				timer += 1
				sys.stdout.write("\r")
				sys.stdout.write("{:3d} seconds elapsed.".format(int(timer)))
				sys.stdout.flush()
				time.sleep(1)

				if timer >= 120:
					sys.stdout.write("\rTimer is Over! No response obtained. Releasing unit...")
					break
		except KeyboardInterrupt:
			rospy.loginfo("[Decon] Keyboard Interrupt received... Releasing unit!")
		sys.stdout.write("\n")
						
		ret_response = self.response_signal[flag_idx]
		self.response_signal[flag_idx] = None
		return ret_response

	def activate_emergency_switch(self):
		self.emergency_topic.publish(True)

	def start(self):
		rospy.loginfo("[Decon] System Activated...")
		while not rospy.is_shutdown():
			if self.state == "HANDSHAKE":
				self.handshake_detection()
				
				if self.wait_for(HANDSHAKE_FLAG) is False:
					rospy.loginfo("[Decon] Detection not working... Try again later!")
					self.state = "EXIT"
				else:
					self.state = "DETECT"

			elif self.state == "DETECT":
				self.detect()
				ret = self.wait_for(DETECTION_FLAG)
			
				if ret == (-1,) or ret is None:
					rospy.loginfo("\r[Decon] Camera could not capture. Try replugging camera cable...")
                                	self.reset_handshake()
					self.state = "HANDSHAKE"
				elif len(ret) > 1:
					rospy.loginfo("[Decon] %s, I heard %s" % (rospy.get_caller_id(), ', '.join([str(elem) for elem in ret])))	
				
					
					[w,h,d,s,k] = ret[:5]
					#objects: [label, x_center, y_center, width, height]
					labels = [ret[el] for el in range(5,len(ret),5)]
					#Converting pixels to meters by coefficients s (in x axis) and k (in y axis)
					objects = [[ret[el]*s, ret[el+1]*k, ret[el+2]*s, ret[el+3]*k] for el in range(6,len(ret),5)] 
					
					self.arm_offset4objects2disinfect = []
					for label, [obj_x_center, obj_y_center, obj_width, obj_height] in zip(labels, objects):
						#If the object is human
						#if label == 1: 
						#	stopAllMovement/Emergency Kill Switch Operation 
						
						#If the object is door handle or light switch
						#Cross section of disinfecting cone is 0.25X0.25, hence the number 0.2
						print("w,h,d,s,k: ",w,h,d,s,k)
						print("label: ",label)
						print("obj_x_center: ", obj_x_center)
						print("obj_y_center: ", obj_y_center)
						print("obj_width: ", obj_width)
						print("obj_height: ", obj_height)
						if label > 1 and obj_width < 0.2 and obj_height < 0.2 and abs(obj_x_center - (w*s/2)) < 0.4:
							obj_horizontal_offset_from_arm_ee = obj_x_center - (w*s/2)
							#Arm End Effector From Camera distance in sleep position is 0.26m
							obj_vertical_offset_from_arm_ee =  (h*k/2) - 0.42 - obj_y_center
							print("obj_horizontal_offset: ", obj_horizontal_offset_from_arm_ee)
							print("obj_vertical_offset: ",obj_vertical_offset_from_arm_ee)
						
							
							self.arm_offset4objects2disinfect.append([0.25, obj_horizontal_offset_from_arm_ee, obj_vertical_offset_from_arm_ee])
						elif label == 1:
							self.activate_emergency_switch()
							rospy.loginfo("[Decon] Human in proximity. Shutting all systems...")
							rospy.signal_shutdown("[Decon] Human proximity. Good bye!")
							
					self.state = "MOVE_CLOSER"
					self.move_position = [d-0.4, 0, 0] if d > 0.4 else [0, 0, 0]
				else:
					rospy.loginfo("[Decon] No objects found... Moving to another location...")
					self.state = "MOVE2NEXT"	
			elif self.state == "MOVE_CLOSER":
				self.move_closer()
				if self.wait_for(MOVE_CLOSER_FLAG) is False:
					rospy.loginfo("\n[Decon] Robot could not move closer to the object. Skipping the object...")
					self.state = "MOVE2NEXT"	
				else:				
					self.state = "MOVE_ARM"
					
			elif self.state == "MOVE_ARM":
				print("In state move_arm, self.arm_offset4objects2disinfect: ", self.arm_offset4objects2disinfect)
				if len(self.arm_offset4objects2disinfect) > 0:
					self.disinfection_completed = False
					#Move arm to home position
					self.move_arm([0.0,0.0]) 
					ret = self.wait_for(ARM_FLAG)
					ret2 = False
					if ret is True:
						#Move arm to position close to the object
						self.move_arm(self.arm_offset4objects2disinfect[0])
						ret2 = self.wait_for(ARM_FLAG)
					if ret is False or ret2 is False:
						rospy.loginfo("\n[Decon] Robot could not position arm. Skipping the object...")
					else:
						self.state = "DISINFECT"
					#self.arm_offset4objects2disinfect = self.arm_offset4objects2disinfect[1:]
					self.arm_offset4objects2disinfect = []
				else:
					self.move_arm([0.0, 0.0])
					ret = self.wait_for(ARM_FLAG)
					if ret is True:
						ret = False
						self.state = "MOVE2NEXT"
					else:
						rospy.loginfo("[Decon] Could not get ARM to reposition to home position. Manual reset of the arm needed...")
						rospy.signal_shutdown("[Decon] ARM Error... Shutting Down!")

			elif self.state == "DISINFECT":
				self.disinfect()
				self.state = "MOVE_ARM"				
				self.disinfection_completed = True
			
			elif self.state == "MOVE2NEXT":
				if self.move2next_location() is False:
					self.state = "EXIT"
				elif self.wait_for(LOCATION_FLAG) is True:
					self.state = "DETECT"
						
			elif self.state == "EXIT":
				rospy.loginfo("[Decon] Good Bye")
				break	

			else:
				rospy.loginfo("[Decon] [ERROR] Unknown state reached. Exitting system...")
				self.state = "EXIT"	
					

		self.rate.sleep()		

#Home coordinates: [2.7955,-0.41917,0,-24.27]				
if __name__ == '__main__':
	try:
		home_coord = [3.2278,-0.94701,0,62]
		#dcontroller = DeconController([[0.4,-1.06,0,0], [0.4,-0.532,0], [0.5,-1.5,0,-90]])
		#dcontroller = DeconController([[3.49093,-1.12527,0,-26.5],[2.883,-0.72133,0,65.91]])
		dcontroller = DeconController([[3.2278,-0.94701,0,62]])
		#[2.883,-0.72133,0,65.91]])
		dcontroller.start()
		dcontroller.move_arm([0.0, 0.0])
		ret = dcontroller.wait_for(ARM_FLAG)
		if ret is True:
			ret = False
			request = Float32MultiArray()
			request.data = home_coord 
			dcontroller.move2next_location_command_topic.publish(request)
			ret = dcontroller.wait_for(LOCATION_FLAG)
			if ret is not True:
				rospy.loginfo("[Decon] Error Occured going back to home position...")
			rospy.signal_shutdown("[Decon] Good Bye...")

	except rospy.ROSInterruptException:
		log_string = "[Decon] Decon Controller exiting at %s!" % rospy.get_time()	
		rospy.loginfo(log_string)

