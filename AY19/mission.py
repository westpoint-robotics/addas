import dronekit
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from math import sin, cos, sqrt, atan2, radians
import RPi.GPIO as GPIO
import time
import subprocess
GPIO.setwarnings(False)

#---------USER INPUTS-------------
target_lat = 52.2296756		# Where the dispenser lands
target_lng = 21.0122287		# Where the dispenser lands

impact_lat = 41.390997		# Where the payload lands
impact_lng = -73.953330		# Where the payload lands
impact_alt = 5			# The altitude where the payload lands


impact_point = LocationGlobalRelative(impact_lat, impact_lng, impact_alt) 		# Sets location for impact point

dist_thresh = 9000000 		# Distance Threshold
alt_thresh = 5000	# Altitude Threshold
#-----------SETUP-----------------
loop_flag = 1
alt_flag = 0
low_alt_flag = 0
dist_flag = 0

connection_string = ('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00')		 # Path that RPI looks to connect to Pixhawk
vehicle = connect(connection_string, wait_ready = True)			# Connects to vehicle
print("Vehicle Connected, waiting 20 seconds")
time.sleep(1)


R_earth = 6373.0
c_target_lat = radians(target_lat)
c_target_lng = radians(target_lng)
c_impact_lat = radians(impact_lat)
c_impact_lng = radians(impact_lng)

def arm_and_takeoff(TargetAlt):			# Function to takeoff
	print "Baisc pre-arm tests"
	while not vehicle.is_armable:		# Doesn't start until  Pixhawk is initialized
		print "Waiting for vehicle to initialize"
		time.sleep(1)
	print "Arming Motors"
	vehicle.mode = VehicleMode("GUIDED")		# Sets flight mode
	vehicle.armed = True				# Arms throttle

	while not vehicle.armed:			# Doesn't try to takeoff until the throttle is armed
		print "Waiting for arming..."
		time.sleep(1)
	vehicle.simple_takeoff(TargetAlt)		# Takes off to target altitude

	while True:
		print "Altitude: ", vehicle.location.global_frame.alt		# Prints altitude until it reaches target height
		if vehicle.location.global_frame.alt >= TargetAlt*.95:		# Waits until drone is 95% of takeoff height
			print "Reached Target Altitude"
			break							# Stops the takeoff function
		time.sleep(1)
def fly_to_waypoint(targ_lat, targ_lon, point):


	vehicle.simple_goto(point, groundspeed = 5)
	while True
		c_lat = radians(vehicle.location.global_frame.lat)
		c_lon = radians(vehicle.location.global_frame.lon)
		dlat = c_lat - targ_lat
		dlon = c_lon - targ_lat
		a1 = sin(dlat/2)**2 + cos(targ?lat) * cos(c_lat) * sin(dlon/2)**2
		c1 = 2*atan2(sqrt(a1), sqrt(1-a1))
		dist = R_earth*c1
		print "Distance to Target: ", dist

		if dist < 1.5:
			print "Arrived at Target"
			break
		else:
			print "Flying to target"
			sleep(3)

#-----------LOOP------------------
while loop_flag > 0:
	current_lat = vehicle.location.global_frame.lat			# Getting current latitude
	current_lng = vehicle.location.global_frame.lon			# Getting current longitude
	current_alt = vehicle.location.global_frame.alt			# Getting current altitude
	print current_alt
	print current_lat
	time.sleep(.25)
	#-----DISTANCE CALC---------------
	c_current_lat = radians(current_lat)				# Calculations for finding straight line distance
	c_current_lng = radians(current_lng)
	d_lat = c_current_lat - c_target_lat
	d_lng = c_current_lng - c_target_lng
	a = sin(d_lat/2)**2 + cos(c_target_lat) * cos(c_current_lat) * sin(d_lng/2)**2
	c = 2 * atan2(sqrt(a), sqrt(1-a))
	distance = R_earth*c

	#----------LOGIC------------------
	if (distance < dist_thresh and current_alt < alt_thresh and dist_flag == 0): 	# Drone arms when at distance threshold, below altitude, and distance flag is met, only runs onc
		take_off_alt = current_alt + 4 						# Sets takeoff altitude 4 meters above current position
		arm_and_takeoff(take_off_alt)						# Arms drone and takes off
		print "done taking off"
		time.sleep(3)


		vehicle.simple_goto(impact_point, groundspeed = 5)			# Drone goes to impact point at 5 m/s
		fly_to_waypoint(											# Gives drone time to navigate to point
		vehicle.mode = VehicleMode("RTL")






# comment is backslack key or pound
