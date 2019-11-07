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

target_lat =  32.791295			# Target Latitude
target_lng = -111.434894			# Target Longitude
target_alt = 5
arm_alt = 800

waypoint_thresh = 30 			# distance threshold for first waypoint
waypoint_flag = 0

dist_thresh =150			# Distance when drone is allowed to arm     50 meters

low_dist_thresh =2

ult_dist_thresh = .5 # Threshold for ultrasonic sensor 0.5 meters

impact_point = LocationGlobal(target_lat, target_lng, target_alt)

# Setup ---------------------------------------------------------------------
connection_string = ('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00') # Location of the connection to the drone
vehicle = connect(connection_string, wait_ready = True) # Opens Connection Port, Doesn't move on until connected

# GPIO Pin Setup
pin_trigger = 5
pin_echo = 7
latch_pin = 11
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_trigger, GPIO.OUT)
GPIO.setup(pin_echo, GPIO.IN)
GPIO.output(pin_trigger, GPIO.LOW)

GPIO.setup(latch_pin, GPIO.OUT)
GPIO.output(latch_pin, GPIO.LOW)
ult_dist = 0
state_machine = 1 # Phase of operation (1=transit, 2=onstation, 3=released
				# 4=nav to drop, 5=descent to drop, 6=nav to end)
prev_alt = 0 # Altitude at previous time through loop 
throw_vert_rate = 0.3 # Rate in m/s we will assume throw engaged 
dist2wp_thresh = 10 # Threshold below which you consider arrived at waypoint 
start_range = 0
stop_range = 0
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
def read_distance():
	break_flag = False
	start_time = timeit.default_timer()
	dist = []
	while timeit.default_timer() - start_time < .1:
		GPIO.output(pin_trigger, GPIO.HIGH)
		time.sleep(.00001)
		GPIO.output(pin_trigger, GPIO.LOW)
		break_time = timeit.default_timer()
		while GPIO.input(pin_echo) == 0:
			if (timeit.default_timer() - break_time) >.12 or break_flag == True:
				start_range = 0
				stop_range = 0
				break_flag = True
				break
			start_range = time.time()

		while GPIO.input(pin_echo) == 1:
			if (timeit.default_timer() - break_time) > .12 or break_flag == True:
				start_range = 0
				stop_range = 0
				break
			stop_range = time.time()
		duration =abs(stop_range - start_range)
		dist1 = duration * 17150/100
		dist.append(dist1)

	avg_dist = np.mean(dist)
	return avg_dist

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


low_alt = 6000

#timer_start = timeit.default_timer()

while True:
	time.sleep(1)
	airspeed = vehicle.velocity
	current_lat = vehicle.location.global_frame.lat
	current_lng = vehicle.location.global_frame.lon
	current_alt = vehicle.location.global_frame.alt
	print "Modenow: ", vehicle.mode.name, ", Armed: ", vehicle.armed, ", StateMachine: ", state_machine, "GlobAlt: ", current_alt
	alt_diff = abs(current_alt - prev_alt) #Compute alt change since last loop
	prev_alt = current_alt # Update previous alt for next time in loop


#Distance Calculation ---------------------------------------------------------------------
#	R_earth = 6373.0 calc_target_lat = radians(target_lat) calc_target_lng = radians(target_lng) calc_current_lat = radians(current_lat) calc_current_lng = radians(current_lng) d_lat = calc_current_lat - calc_target_lat d_lng = 
#calc_current_lng - calc_target_lng a = sin(d_lat/2)**2 + cos(calc_target_lat) * cos(calc_current_lat) * sin(d_lng/2)**2 c = 2 * atan2(sqrt(a), sqrt(1-a)) distance = R_earth*c 
#---------------------------------------------------------------------
	# Check if within flight area.  Otherwise disarm.
	distance = dist_2_wp(current_lat, current_lng, target_lat, target_lng)
	if (distance > dist_thresh):
		print "Out of Area, Disarming"
		state_machine = 1
		vehicle.armed= False
		vehicle.mode = VehicleMode("STABILIZE")

	# Determine what state the machine is in (Phase of operation)
	if state_machine == 1: # In transit to op area (do not fly)
		vehicle.armed = False
		print "distance: ", distance, ", current_alt: ", current_alt
		if (distance < dist_thresh and current_alt < arm_alt):
			state_machine = 2
			print "switching to state 2"


	elif state_machine == 2: # In op area awaiting drop (arm throw mode)
		if (distance < dist_thresh and current_alt < arm_alt):
			vehicle.mode = VehicleMode("THROW")
			vehicle.armed = True
			ult_dist = read_distance()
                        print "Ultrasound Distance", ult_dist
			print "Entered Area: Armed"
			# Detect if drop has occured (TODO: add ultrasonic readout)
			if (abs(alt_diff) > throw_vert_rate and ult_dist > ult_dist_thresh) or vehicle.mode.name == "AUTO" :
				state_machine = 3 
				print "switching to state 3, counting down to auto..."
				time.sleep(3)
				vehicle.mode = VehicleMode("AUTO")
		else:
			state_machine = 1

	elif state_machine == 3:
		print "Entered State 3"
		#vehicle.mode = VehicleMode("GUIDED") vehicle.armed = True vehicle.simple_takeoff(current_alt+4) vehicle.mode = VehicleMode("AUTO") print "Mode auto issued"
		print "Flying to First Waypoint"
		dist_2_target = dist_2_wp(current_lat, current_lng, target_lat, target_lng)
		print "Distance to target (m): ", dist_2_target
		if dist_2_target < waypoint_thresh:
			print "Arrived at first waypoint"
			print "Switching to state 4"
			state_machine = 4
			vehicle.mode = VehicleMode("LAND")
		#TODO: use seth's code to nav to specific waypoint

	elif state_machine == 4: # Navigating to payload drop location
		print "Entered State 4"
		print "Landing drone"

		ult_dist = read_distance()
		print "Distance to Ground (m):", ult_dist
		if ult_dist == 0:
			ult_dist = 3
		elif ult_dist < ult_dist_thresh:
			print "Dropping Payload"
			GPIO.output(latch_pin, GPIO.HIGH)
			time.sleep(.5)
			GPIO.output(latch_pin, GPIO.LOW)
			print "Payload Dropped"
			print "Switching to state 5"
#			vehicle.mode = VehicleMode("AUTO")
#			state_machine = 5
		#DROP PAYLOAD FUNCTION
#		dist2wp = 2
		#print "Nav to wpt, dist: ", dist2wp if dist2wp < dist2wp_thresh:
			#state_machine = 4 print "Descending to drop altitude" vehicle.mode = VehicleMode("GUIDED")
			##TODO: start descent

	elif state_machine == 5:
		print "Entered State 5"
		print "Completing mission"
		#vehicle.mode = VehicleMode("AUTO") TODO: check altitude TODO: Drop payload TODO: navigate to land location
		state_machine = 5

	print "logging"
	prev_time = time_now
	time_loop = timeit.default_timer()
	time_now = time_loop
	delta_time = time_loop - time_start
	log.append([time_now, delta_time, current_lat, current_lng, current_alt, vehicle.mode.name, vehicle.armed, state_machine, distance, ult_dist])
	writeArchive(log)
	print "log done"
