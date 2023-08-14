#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 26 11:14:38 2021

@author: wesley
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import csv
import sys
from common import get_output_filename, save_data, clip

# Test parameters
sample_time		= 0.02	# Sample time [s] # Limited by USB/UART?
n				= 11	# True gear ratio
min_current		= 0.0	# Minimum current [A]
target_current	= 40.0	# Target current [A] (default, overwrite via command line argument)
duration		= 15	# Data recording duration, for each direction [s]
turns2deg		= 360	# One turn in [deg]
deg2rad			= math.pi/180 # Degrees to radians
turns2rad		= turns2deg*deg2rad # Turns to rad []

# Check for command-line arguments
if len(sys.argv) >= 2:
	arg_target_current = float(sys.argv[1])
	if arg_target_current != 0:
		print('Setting target current of', arg_target_current, 'A..')
		target_current = arg_target_current
	else:
		raise Exception('Invalid target current supplied by argument.')
else:
	print('Setting default target current of', target_current, 'A..')

# Compute ramping rate from min, max, and duration
ramp_rate		= 2 * (target_current-min_current) / duration # [rad/s^2]

# Prompt user for output filename
out_filename = get_output_filename('out_experiment_3_stiffness_current.csv')

# Find a connected ODrive (this will block until you connect one)
print("Finding odrive..")
my_drive = odrive.find_any()

# Check if the torque constant is unity (otherwise the setpoint is not current)
if my_drive.axis0.motor.config.torque_constant != 1.0:
	curses.endwin()
	sys.exit('torque_constant is not 1.0: Cannot continue. Quitting.')

# Set control mode
# Torque control (= current control)
print('Setting control mode and parameters..')
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
my_drive.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
my_drive.axis0.controller.config.enable_torque_mode_vel_limit = False # This function will cause issues if vel_gain is zero, and is weird anyway.

# Control gains
my_drive.axis0.motor.config.current_control_bandwidth = 250

print('Starting closed loop control..')
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Start, wait for acceleration
print('Starting motor at minimum current', round(min_current,2), 'A')
current_setpoint = min_current
my_drive.axis0.controller.input_torque = current_setpoint
time.sleep(1)

# Record data
print('Recording data for', duration, 'seconds in positive direction..')
data = []
t0 = time.monotonic()
t_prev = t0

# Ramping direction
ramp = True

# Loop while duration has not passed
while time.monotonic()-t0 < duration:
	# Get iteration start time
	t = time.monotonic()

	# Get data
	motorPos    = my_drive.axis0.encoder.pos_estimate
	outputPos   = my_drive.axis1.encoder.pos_estimate # May or may not be available
	motorVel    = my_drive.axis0.encoder.vel_estimate
	outputVel   = my_drive.axis1.encoder.vel_estimate # May or may not be available
	Iq_set		= my_drive.axis0.motor.current_control.Iq_setpoint
	Iq_meas		= my_drive.axis0.motor.current_control.Iq_measured
	data.append([
		t-t0,
		motorPos,
		outputPos,
		motorVel,
		outputVel,
		current_setpoint,
		Iq_set,
		Iq_meas,
	])

	# Increment/decrement reference current
	# We start with ramp=True, which means we are increasing current
	# After half duration, ramp=False, we start ramping down from max current
	if ramp:
		current_setpoint += (t-t_prev)*ramp_rate
	else:
		current_setpoint -= (t-t_prev)*ramp_rate

	# Set motor current, but limit for safety
	current_setpoint = clip(current_setpoint, min_current, target_current)
	print('Setting motor current of', round(current_setpoint,2), 'A..')
	my_drive.axis0.controller.input_torque = current_setpoint

	# Halfway duration we start ramping down
	if t-t0 >= duration/2:
		ramp = False
	
	# Save this iteration time to accurately compute current increments
	t_prev = t

	# Sleep for loop
	try:
		time.sleep(sample_time - (time.monotonic()-t))
	except:
		print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

print('Finished positive direction. Starting motor at minimum current in negative direction..')
my_drive.axis0.controller.input_torque = -min_current
time.sleep(1)

# Reset ramping direction
ramp = True

# Loop while duration has not passed
# Take a new time sample for the start of this duration that we use for timing (not for data out)
t1 = time.monotonic()
t_prev = t1
while time.monotonic()-t1 < duration:
	# Get iteration start time
	t = time.monotonic()

	# Get data
	motorPos    = my_drive.axis0.encoder.pos_estimate
	outputPos   = my_drive.axis1.encoder.pos_estimate # May or may not be available
	motorVel    = my_drive.axis0.encoder.vel_estimate
	outputVel   = my_drive.axis1.encoder.vel_estimate # May or may not be available
	Iq_set		= my_drive.axis0.motor.current_control.Iq_setpoint
	Iq_meas		= my_drive.axis0.motor.current_control.Iq_measured
	data.append([
		t-t0,
		motorPos,
		outputPos,
		motorVel,
		outputVel,
		current_setpoint,
		Iq_set,
		Iq_meas,
	])

	# Increment/decrement reference current
	# We start with ramp=True, which means we are increasing current
	# After half duration, ramp=False, we start ramping down from max current
	if ramp:
		current_setpoint -= (t-t_prev)*ramp_rate
	else:
		current_setpoint += (t-t_prev)*ramp_rate
	
	# Set motor current, but limit for safety
	current_setpoint = clip(current_setpoint, -target_current, -min_current)
	print('Setting motor current of', round(current_setpoint,2), 'A..')
	my_drive.axis0.controller.input_torque = current_setpoint

	# Halfway duration we start ramping down
	if t-t1 >= duration/2:
		ramp = False
		
	# Save this iteration time to accurately compute current increments
	t_prev = t

	# Sleep for loop
	try:
		time.sleep(sample_time - (time.monotonic()-t))
	except:
		print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

# Stop controller
print('Finished, stopping closed-loop control..')
my_drive.axis0.controller.input_torque = 0
time.sleep(1)
my_drive.axis0.requested_state = AXIS_STATE_IDLE

# Save data
save_data(data, out_filename)

# End
print('Done.')
