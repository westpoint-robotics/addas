import dronekit
from dronekit import connect, VehicleMode, LocationGlobal, Command
from math import sin, cos, sqrt, atan2, radians
import RPi.GPIO as GPIO
import time, sys, argparse, math
import subprocess
import timeit
from pymavlink import mavutil
import numpy as np
import csv, os
from datetime import datetime
GPIO.setwarnings(False)

# Define Mission Parameters
#target_lat =  32.790273			# Target Latitude
#target_lng = -111.432771		# Target Longitude
target_lat =  32.880346			# Target Latitude
target_lng = -111.697348		# Target Longitude
target_alt = 20                 # Target Altitude (not used)
arm_alt = 800

waypoint_thresh = 30 			# distance threshold for first waypoint
dist_thresh = 50			    # Distance when drone is allowed to arm 

state_machine = 1 # Phase of operation 
prev_alt = 0 # Altitude at previous time through loop 
throw_vert_rate = 0.3 # Rate in m/s we will assume throw engaged 
dist2wp_thresh = 10 # Threshold below which you consider arrived at waypoint 
start_range = 0
stop_range = 0

# Setup ---------------------------------------------------------------------
connection_string = ('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00') # Location of the connection to the drone
vehicle = connect(connection_string, wait_ready = True) # Opens Connection Port, Doesn't move on until connected

# Setup Initial Vehicle Mode
vehicle.armed = False 
vehicle.mode = VehicleMode("STABILIZE")

#-----------------DATALOGGING--------------------------------------------------
static_path = "log" +  ".csv"
archive_path = "archive/addas_" + str(datetime.now()) + ".csv"
time_start = timeit.default_timer()
time_now = time_start
log = []

def writeArchive(log):
	global archive_path
	script_dir = os.path.dirname(__file__)
	absolute_path = os.path.join(script_dir, archive_path)

	with open(archive_path, 'w') as outf:
		outfwriter = csv.writer(outf)
		for i in log:
			outfwriter.writerow(i)

	with open(static_path, 'w') as outf:
		outfwriter = csv.writer(outf)
		for i in log:
			outfwriter.writerow(i)
#-----------------FUNCTIONS----------------------------------------------------
def dist_2_wp(lat1, lon1, lat2, lon2):
	lat1 = radians(lat1)
	lon1 = radians(lon1)
	lat2 = radians(lat2)
	lon2 = radians(lon2)

	dlon = lon2 - lon1
	dlat = lat2 - lat1
	a = (sin(dlat/2))**2 + cos(lat1) * cos(lat2) * (sin(dlon/2))**2
	c = 2 * atan2(sqrt(a), sqrt(1-a))
	r = 6373
	dist_2_targ = r*c*1000
	return dist_2_targ

#--------------------------------------------------------------------------------
while True:
	time.sleep(1)
    # Loop timing
	prev_time = time_now
	time_loop = timeit.default_timer()
	time_now = time_loop
	delta_time = time_loop - time_start

    # Read in vehicle information
	airspeed = vehicle.velocity
	current_lat = vehicle.location.global_frame.lat
	current_lng = vehicle.location.global_frame.lon
	current_alt = vehicle.location.global_frame.alt

	print "Modenow: ", vehicle.mode.name, ", Armed: ", vehicle.armed, ", StateMachine: ", state_machine, "GlobAlt: ", current_alt
	alt_diff = abs(current_alt - prev_alt) #Compute alt change since last loop
	prev_alt = current_alt # Update previous alt for next time in loop

	# Check if within flight area.  Otherwise disarm.
	distance = dist_2_wp(current_lat, current_lng, target_lat, target_lng)
	if (distance > dist_thresh):
		print "Out of Area, Disarming"
		state_machine = 1
		#vehicle.armed= False
		#vehicle.mode = VehicleMode("STABILIZE")

	# Determine what state the machine is in (Phase of operation)
	if state_machine == 1: # In transit to op area (do not fly)
		vehicle.armed = False
		print "distance: ", distance, ", current_alt: ", current_alt
		if (distance < dist_thresh and current_alt < arm_alt):
			state_machine = 2
			print "switching to state 2"

	elif state_machine == 2: # In op area awaiting drop (arm throw mode)
		if (distance < dist_thresh and current_alt < arm_alt):
			#vehicle.mode = VehicleMode("THROW")
			#vehicle.armed = True
			print "Entered Area: Armed, Dist: ", distance
			if vehicle.mode.name == "AUTO":
				state_machine = 3
				print "switching to state 3"

		else:
			state_machine = 1

	elif state_machine == 3:
		print "Entered State 3: Flying to First Waypoint"
		dist_2_target = dist_2_wp(current_lat, current_lng, target_lat, target_lng)
		print "Distance to target (m): ", dist_2_target
		if dist_2_target < waypoint_thresh:
			print "Arrived at first waypoint"
			print "Switching to state 4"
			state_machine = 4
			land_alt = current_alt - 20
			vehicle.mode = VehicleMode("LAND")

	elif state_machine == 4: # Navigating to payload drop location
		print "Entered State 4"
		print "Landing drone"
		if current_alt < land_alt:
			print "Dropping Payload"
			GPIO.output(latch_pin, GPIO.HIGH)
			print "Payload Dropped"
			print "Switching to state 5"
			vehicle.mode = VehicleMode("AUTO")
			state_machine = 5

	elif state_machine == 5:
		print "Entered State 5: Completing mission"
		state_machine = 5


	print "logging"

	log.append([time_now, delta_time, current_lat, current_lng, current_alt, vehicle.mode.name, vehicle.armed, state_machine, distance, 0])
	writeArchive(log)
