
import rospy
import copy
scale = 0.05
min_z = 0.1
max_z = 1.0
min_x = -0.1
max_x = 0.5
min_y = -0.5
max_y = 0.5

def check_bounds(pose):
	if pose.position.x < min_x:
		print("[Decon]: X less than minX...")
		return False
	if pose.position.x > max_x:
		print("[Decon]: X greater than maxX...")
		return False
	if pose.position.y < min_y:
		print("[Decon]: y less than minY...")
		return False
	if pose.position.y > max_y:
		print("[Decon]: y greater than maxY...")
		return False
	if pose.position.z < min_z:
		print("[Decon]: z less than minZ...")
		return False
	if pose.position.z > max_z:
		print("[Decon]: z greater than maxZ...")
		return False
	return True

def move_to_pose(group, waypoints):
	(plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        group.execute(plan, wait=True)
	group.stop()
	group.clear_pose_targets()
	rospy.sleep(2)

def inc_z(group, offset):
	rospy.loginfo("[Decon] Increase ARM's z...")
	waypoints = []
	wpose = group.get_current_pose().pose
	while offset >= scale:
		wpose.position.z += scale
		if not check_bounds(wpose):
			break
		waypoints.append(copy.deepcopy(wpose))
		offset -= scale
	if len(waypoints) > 0: 
		move_to_pose(group, waypoints)

def dec_z(group, offset):
	rospy.loginfo("[Decon] Decrease ARM's z...")
	waypoints = []
	wpose = group.get_current_pose().pose
	while offset >= scale:
		wpose.position.z -= scale
		if not check_bounds(wpose):
			break
		waypoints.append(copy.deepcopy(wpose))
		offset -= scale
	if len(waypoints) > 0: 
		move_to_pose(group, waypoints)

def inc_x(group, offset):
	rospy.loginfo("[Decon] Increase ARM's x...")
 	waypoints = []
	wpose = group.get_current_pose().pose
	while offset >= scale:
		wpose.position.x += scale
		wpose.position.z += 0.015
		if not check_bounds(wpose):
			break
		waypoints.append(copy.deepcopy(wpose))
		offset -= scale
	if len(waypoints) > 0: 
		move_to_pose(group, waypoints)

def dec_x(group, offset):
	rospy.loginfo("[Decon] Decrease ARM's x...")
  	waypoints = []
	wpose = group.get_current_pose().pose
	while offset >= scale:
		wpose.position.x -= scale
		wpose.position.z += 0.007
		if not check_bounds(wpose):
			break
		waypoints.append(copy.deepcopy(wpose))
		offset -= scale
	if len(waypoints) > 0: 
		move_to_pose(group, waypoints)

def inc_y(group, offset):
	rospy.loginfo("[Decon] Increase ARM's y...")
  	waypoints = []
	wpose = group.get_current_pose().pose
	while offset >= scale:
		wpose.position.y += scale
		if not check_bounds(wpose):
			break
		waypoints.append(copy.deepcopy(wpose))
		offset -= scale
	if len(waypoints) > 0: 
		move_to_pose(group, waypoints)

def dec_y(group, offset):
	rospy.loginfo("[Decon] Decrease ARM's y...")
  	waypoints = []
	wpose = group.get_current_pose().pose
	while offset >= scale:
		wpose.position.y -= scale
		if not check_bounds(wpose):
			break
		waypoints.append(copy.deepcopy(wpose))
		offset -= scale
	if len(waypoints) > 0: 
		move_to_pose(group, waypoints)

def move(group, x_offset, y_offset, z_offset):
	
	#Z offset
	inc_z(group, z_offset) if z_offset > 0 else dec_z(group, abs(z_offset))
	#X offset
	inc_x(group, x_offset) if x_offset > 0 else dec_x(group, abs(x_offset))
	#Y offset
	inc_y(group, y_offset) if y_offset > 0 else dec_y(group, abs(y_offset))
	
