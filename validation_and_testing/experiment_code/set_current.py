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
import curses
import sys
from common import get_output_filename, save_data, append_filename

def main(stdscr):
	# Test parameters
	sample_time		= 0.02	# Sample time [s] # Limited by USB/UART?
	n				= 11	# Gear ratio []
	amp_per_step	= 0.1	# Ampere to increment per step
	turns2deg		= 360	# One turn in degrees []
	deg2rad			= math.pi/180 # Degrees to radians []
	turns2rad		= turns2deg*deg2rad # Turns to rad []

	# Control gains
	# Not setting gains as we are setting current directly in this test
	#current_lim			= 40		# Current limit [A] (40A tops out at around 31 Nm)
	#requested_current	= 60		# Current sensing limit [A]

	# Prompt user for output filename
	out_filename = get_output_filename('out_set_current.csv', stdscr)

	# curses: disable echo
	curses.noecho()
	# curses: make getch() non-blocking
	stdscr.nodelay(1)
	line = 0

	# Find a connected ODrive (this will block until you connect one)
	stdscr.addstr(line,0,'Finding odrive..'); line += 1
	my_drive = odrive.find_any()
	
	# Check if the torque constant is unity (otherwise the setpoint is not current)
	if my_drive.axis0.motor.config.torque_constant != 1.0:
		curses.endwin()
		sys.exit('torque_constant is not 1.0: Cannot continue. Quitting.')

	# Set control mode
	# Torque control (= current control)
	stdscr.addstr(line,0,'Setting control mode and parameters..'); line += 1
	my_drive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
	my_drive.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
	my_drive.axis0.controller.config.enable_torque_mode_vel_limit = False # This function will cause issues if vel_gain is zero, and is weird anyway.
	
	# Set current limit
	#my_drive.axis0.motor.config.current_lim = current_lim
	#my_drive.axis0.motor.config.requested_current_range = requested_current

	# Then start position control
	stdscr.addstr(line,0,'Starting closed loop control..'); line += 1
	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	time.sleep(1.0)

	# Record data
	stdscr.addstr(line,0,'Recording data..'); line += 1
	data = []
	data_stiffness = []
	t0 = time.monotonic()
	
	# Set initial current setpoint (KEEP ZERO)
	current_setpoint = 0

	# Loop until told to stop
	line += 1; stdscr.addstr(line,0,'Use +/- to increase/decrease current, or press \'q\' to stop.'); line += 1; line += 1
	stdscr.addstr(line,4,'Current setpoint: '+str(current_setpoint)+' A..')
	while True:
		# Get iteration start time
		t = time.monotonic()
		
		# curses: get keyboard input, returns -1 if none available
		c = stdscr.getch()

		if c == ord('+') or c == ord('='):
			current_setpoint += amp_per_step
			my_drive.axis0.controller.input_torque = current_setpoint
			stdscr.addstr(line,4,'Current setpoint: '+str(current_setpoint)+' A..')

		elif c == ord('-'):
			current_setpoint -= amp_per_step
			my_drive.axis0.controller.input_torque = current_setpoint
			stdscr.addstr(line,4,'Current setpoint: '+str(current_setpoint)+' A..')

		elif c == ord('q'):
			line += 1
			break

		# Get data
		motorPos	= my_drive.axis0.encoder.pos_estimate
		outputPos	= my_drive.axis1.encoder.pos_estimate # May or may not be available
		motorVel	= my_drive.axis0.encoder.vel_estimate
		outputVel	= my_drive.axis1.encoder.vel_estimate # May or may not be available
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
		
		# Sleep for loop
		try:
			time.sleep(sample_time - (time.monotonic()-t))
		except:
			print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

	# Stop controller
	line += 1; stdscr.addstr(line,0,'Finished, stopping closed-loop control..'); line += 1
	my_drive.axis0.controller.input_torque = 0
	my_drive.axis0.requested_state = AXIS_STATE_IDLE
	c = stdscr.getch()
	time.sleep(1)

	# Stop curses window
	curses.endwin()

	# Save data
	save_data(data, out_filename)

	# End
	print('Done.')


# Run
if __name__ == '__main__':
	curses.wrapper(main)
