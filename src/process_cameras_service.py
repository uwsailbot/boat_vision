#!/usr/bin/env python
import rospy
import math
from boat_vision.srv import *

def sind(angle):
	return math.sin(math.radians(angle))

def cosd(angle):
	return math.cos(math.radians(angle))


# angle is in CCW degrees from x+ axis
def get_x_displacement(x_init, rad, angle):
	return cosd(angle) * rad + x_init

# angle is in CCW degrees from x+ axis
def get_y_displacement(y_init, rad, angle):
	return sind(angle) * rad + y_init


def process_cams(req):
	res = ProcessCamerasResponse()
	
	# Output the request
	rospy.loginfo(rospy.get_caller_id() + " Request: left=%.2f,%.2f, right=%.2f,%.2f", req.left.x, req.left.y, req.right.x, req.right.y)
	
	# Calculate the average y value and the angle to that y val
	avg_y = (req.left.y + req.right.y)/2.0
	vert_angle = ((avg_y / req.cam.height) * 2 - 1) * req.cam.fov_vert / 2
	
	# Determine the angle to the object in each camera, from -30 to 30deg (0deg is straight ahead)
	angle1 = ((req.left.x / req.cam.width) * 2 - 1) * req.cam.fov_horiz / 2; # -30-30deg
	angle2 = ((req.right.x / req.cam.width) * 2 - 1) * req.cam.fov_horiz / 2; # -30-30deg
	
	# If the lines are parallel or diverging, there is no solution
	if (angle1 <= angle2 or angle2 >= angle1):
		rospy.logerr("Invalid input, angles are diverging")
		res.target.dist = -1
		res.target.heading = -1
		return res
	
	
	# Convert the angles from 0deg = straight to the internal angle of the triangle bound by the rays
	angle1 = -angle1 + 90
	angle2 += 90
	
	# Determine the third angle through simple geometrical relationships
	angle3 = 180 - angle1 - angle2
	
	# Determine the length from each camera to the buoy, using the sine law
	dist1 = req.spacing / sind(angle3) * sind(angle1)
	dist2 = req.spacing / sind(angle3) * sind(angle2)
	
	# Determine the coordinates of the buoy relative to the center of the two cams.
	# x is horiz position, y is depth, in meters
	x_final = get_x_displacement(-req.spacing/2.0, dist1, angle1)
	y_final = get_y_displacement(0, dist1, angle1)
	
	# Perhaps useful if the cameras are tilted up or down?
	#depth_2d = sqrt(x_final * x_final + y_final * y_final)
	#z_final = depth_2d / sinDeg(90-vert_angle) * sinDeg(vert_angle)
	
	# Use simple trig to determine the distance and angle from the center of the
	# two cameras to the buoy's coordinates.
	
	res.target.dist = math.sqrt(x_final * x_final + y_final * y_final)
	#res.buoy.dist = sqrt(x_final * x_final + y_final * y_final + z_final * z_final)
	res.target.heading = math.degrees(math.atan(y_final / x_final))
	
	rospy.loginfo(rospy.get_caller_id() + "Returning response: [dist:%.2f, heading:%.2f]", res.target.dist, res.target.heading)
	return res


if __name__ == '__main__':
	rospy.init_node("process_cameras_server")
	rospy.Service('process_cameras', ProcessCameras, process_cams)
	rospy.spin()
