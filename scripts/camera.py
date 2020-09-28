import math
from math import tan, radians
import numpy as np
import sys 
import pyzed.sl as sl
from PIL import Image
import torch

def capture(imagePath):
	#Create a ZED camera object
	zed = sl.Camera()
	
	# Set configuration parameters
	init_params = sl.InitParameters()
	init_params.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA # Use ULTRA depth mode
	init_params.coordinate_units = sl.UNIT.UNIT_METER # Use meter units (for depth measurements)
	init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
	
	#Open the camera
	err = zed.open(init_params)
	if err != sl.ERROR_CODE.SUCCESS:
		print("[DECON] Error: Unable to open the camera")
		zed.close()
		exit(-1)
	
	try:
		image = sl.Mat()
		depth_map = sl.Mat()
		point_cloud = sl.Mat()
		mirror_ref = sl.Transform()
	    	mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    		tr_np = mirror_ref.m
		runtime_parameters = sl.RuntimeParameters()
		runtime_parameters.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD  # Use STANDARD sensing mode
    		# Setting the depth confidence parameters
    		#runtime_parameters.confidence_threshold = 100
    	#	runtime_parameters.textureness_confidence_threshold = 100
		if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS :
			# A new image and depth is available if grab() returns SUCCESS
			zed.retrieve_image(image, sl.VIEW.VIEW_LEFT) # Retrieve left image
			img = image.get_data()
			im = Image.fromarray(img).rotate(180)
			# Retrieve depth map. Depth is aligned on the left image
            		zed.retrieve_measure(depth_map, sl.MEASURE.MEASURE_DEPTH)
            		# Retrieve colored point cloud. Point cloud is aligned on the left image.
            		zed.retrieve_measure(point_cloud, sl.MEASURE.MEASURE_XYZRGBA)

            		# Get and print distance value in mm at the center of the image
            		# We measure the distance camera - object using Euclidean distance
            		x = round(image.get_width() / 2)
            		y = round(image.get_height() / 2)
            		err, point_cloud_value = point_cloud.get_value(x, y)

            		distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
            		                     point_cloud_value[1] * point_cloud_value[1] +
            		                     point_cloud_value[2] * point_cloud_value[2])

            		point_cloud_np = point_cloud.get_data()
            		point_cloud_np.dot(tr_np)

            		if not np.isnan(distance) and not np.isinf(distance):
				#print("Distance to camera at (" + str(x) + ", " + str(y) + ") (image center): " + str(distance))
				#print("Image size is: ",im.size)
				(w,h)=im.size
				k = 2*distance*tan(radians(30))/h
				s = 2*distance*tan(radians(45))/w
				print w,h,distance,s,k
				#print("Vertical m/pixel:",str(k))
				#print("Horizontal m/pixel:",str(s))
            		else:
            			print("Can't estimate distance at this position.")
            			print("Your camera is probably too close to the scene, please move it backwards.\n")
				torch.cuda.empty_cache()
				zed.close()
				exit(-1)
            		sys.stdout.flush()
			im.save(imagePath)
                	del im	
	except Exception as e:
		print("[DECON] Error: Exception occured")
		print(e)
	finally:
		torch.cuda.empty_cache()
		zed.close()

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("[DECON] Error: Correct command is python2.7 camera.py <imagePath>")
	else:
		capture(sys.argv[1])
