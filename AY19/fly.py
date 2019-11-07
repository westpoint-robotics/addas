import dronekit
from dronekit import connect, VehicleMode, LocationGlobal
from math import sin, cos, sqrt, atan2, radians
import RPi.GPIO as GPIO
import time
import subprocess
GPIO.setwarnings(False) 

target_lat = 52.2296756			# Target Latitude
target_lng = 21.0122287			# Target Longitude
arm_alt = 1000				# Altitude when drone is allowed to arm
dist_thresh = 600000000			# Distance when drone is allowed to arm
low_dist_thresh = .5
# Setup
#---------------------------------------------------------------------
connection_string = ('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00')		# Location of the connection to the drone
vehicle = connect(connection_string, wait_ready = True) 				# Opens Connection Port, Doesn't move on until connected to drone
loop_flag =  1

pin_trigger = 5
pin_echo = 7
latch_pin = 11 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_trigger, GPIO.OUT)
GPIO.setup(pin_echo, GPIO.IN)
GPIO.output(pin_trigger, GPIO.LOW)


GPIO.setup(latch_pin, GPIO.OUT)
GPIO.output(latch_pin, GPIO.LOW)
low_alt_flag = 0
dist_flag = 0
low_dist_flag = 0

#---------------------------------------------------------------------
#low_alt =  vehicle.location.global_frame.alt 
low_alt = 6000

while loop_flag > 0:
	airspeed = vehicle.velocity
	current_lat =  vehicle.location.global_frame.lat
	current_lng =  vehicle.location.global_frame.lon
	current_alt =  vehicle.location.global_frame.alt
	time.sleep(1)
	current_alt_2 = vehicle.location.global_frame.alt
#Distance Calculation
#---------------------------------------------------------------------
	R_earth = 6373.0
	calc_target_lat = radians(target_lat)
	calc_target_lng = radians(target_lng)
	calc_current_lat = radians(current_lat)
	calc_current_lng = radians(current_lng)
	d_lat = calc_current_lat - calc_target_lat
	d_lng = calc_current_lng - calc_target_lng
	a = sin(d_lat/2)**2 + cos(calc_target_lat) * cos(calc_current_lat) * sin(d_lng/2)**2
	c = 2 * atan2(sqrt(a), sqrt(1-a))
	distance = R_earth*c
#---------------------------------------------------------------------
	alt_flag = 1
	alt_diff = abs(current_alt - current_alt_2)
	print alt_diff
	if (distance < dist_thresh and current_alt < arm_alt):
		vehicle.mode = VehicleMode("THROW")
		vehicle.armed = True
		if abs(alt_diff) > .2:
			print "Counting Down"
			time.sleep(3)
			vehicle.mode = VehicleMode("AUTO")
			mode_now = vehicle.mode.name
			print mode_now
	if current_alt < low_alt:
		low_alt_flag = low_alt_flag + 1
	else:
		low_alt_flag = 0
	if low_alt_flag == 1: 
		print  "Low Altitude Achieved"
		while low_alt_flag ==1:
			GPIO.output(pin_trigger, GPIO.HIGH)
			time.sleep(0.00001)
			GPIO.output(pin_trigger, GPIO.LOW)
			while GPIO.input(pin_echo)  == 0:
				start = time.time()
			while GPIO.input(pin_echo) == 1: 
				stop = time.time()
			duration = stop - start
			dist = duration * 17150/100
			if dist < low_dist_thresh:
				dist_flag = dist_flag + 1
				GPIO.output(latch_pin, GPIO.HIGH)
				low_alt_flag = low_alt_flag + 1
			time.sleep(1)


