import dronekit
from dronekit import connect, VehicleMode, LocationGlobal
from math import sin, cos, sqrt, atan2, radians
import RPi.GPIO as GPIO
import time
import timeit
import numpy
import subprocess
GPIO.setwarnings(False) 

pin_trigger = 5
pin_echo = 7
latch_pin = 11 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_trigger, GPIO.OUT)
GPIO.setup(pin_echo, GPIO.IN)
GPIO.output(pin_trigger, GPIO.LOW)


GPIO.setup(latch_pin, GPIO.OUT)
GPIO.output(latch_pin, GPIO.LOW)
flag = 1


start_time = timeit.default_timer()
dist = []
thresh = .1
#while timeit.default_timer() - start_time < thresh:
while True:
	GPIO.output(pin_trigger, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(pin_trigger, GPIO.LOW)
	while GPIO.input(pin_echo)  == 0:
		start = time.time()
	while GPIO.input(pin_echo) == 1: 
		stop = time.time()
	duration = stop - start
	dist1 = duration * 17150/100
	print(dist1)
#	dist.append(dist1)
print dist
#print numpy.mean(dist)
#	if dist < low_dist_thresh:
#		dist_flag = dist_flag + 1
#		GPIO.output(latch_pin, GPIO.HIGH)
#		low_alt_flag = low_alt_flag + 1
#		time.sleep(1)


