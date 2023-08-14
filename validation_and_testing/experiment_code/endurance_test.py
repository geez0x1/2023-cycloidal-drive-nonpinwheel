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
import datetime
from common import get_output_filename, save_data

# Endurance test motion profile
def motionProfile(t):
	# Parameters
	target_pos		= [	math.pi,
						2*math.pi,
						math.pi,
						0,
						0.5*math.pi,
						math.pi,
						1.5*math.pi,
						2.0*math.pi,
						1.5*math.pi,
						math.pi,
						0.5*math.pi,
						0			] # Target positions [rad]
	target_times	= [	0,
						1*2.5,
						2*2.5,
						3*2.5,
						4*2.5,
						10+1*2.0,
						10+2*2.0,
						10+3*2.0,
						10+4*2.0,
						10+5*2.0,
						10+6*2.0,
						10+7*2.0 ] # Target times [s] (when to switch setpoint)
	t_total			= 10+8*2.0	# Total motion profile time [s]
	
	# Check on parameters
	if len(target_pos) != len(target_times):
		print('target_pos and target_times not equal length')
		return 0
	
	# Normalise time, removing full loops of the profile
	t = t % t_total
	
	# Find the part of the profile we are in
	for i in range(len(target_times)):
		# If there are no more waypoints, OR if the next target_time is in the future,
		# use this one at index i.
		if i+1 == len(target_times) or target_times[i+1] > t:
			# Use this one at index i
			return target_pos[i]		

# Experiment parameters (pendulum)
def loadInertia():
	m_weights	= 2.0		# Weight plates mass [kg]
	m_bar		= 0.432		# Pendulum mass [kg]
	m_mounting	= 0.211		# Mounting stuff (thread, nuts, etc) [kg]
	L			= 0.5		# Pendulum length

	return (m_weights+m_mounting) * L**2 + (1/3) * m_bar * L**2

# Experiment parameters (LIGHT SHORT pendulum)
def loadInertia_light():
	m_weights	= 0.0		# Weight plates mass [kg]
	m_bar		= (44/50)*0.432 # Pendulum mass [kg]
	m_mounting	= 0			# Mounting stuff (thread, nuts, etc) [kg]
	L			= 0.44		# Pendulum length

	return (m_weights+m_mounting) * L**2 + (1/3) * m_bar * L**2

def main(stdscr):
	# Test parameters
	sample_time		= 0.02	# Sample time [s] # Limited by USB/UART?
	n				= 11	# True gear ratio
	turns2deg		= 360	# One turn in [deg]
	deg2rad			= math.pi/180 # Degrees to radians
	turns2rad		= turns2deg*deg2rad # Turns to rad []
	inertia			= 1.0*loadInertia()	# Load inertia

	# Motion behaviour
	max_speed		= 3	# Max output speed [rad/s]
	max_accel		= 3	# Max acceleration/deceleration [rad/s^2]
	max_runtime		= 3600	# Maximum runtime [s]

	# Control gains
	pos_gain			= 5*20		# [turns/s / turn] - default 20
	vel_gain			= 15*0.16	# [Nm / turns/s] - default 0.16
	vel_integrator_gain	= 5*0.32	# [Nm / turn/s^2] - default 0.32
	current_lim			= 40		# Current limit [A] (40A tops out at around 31 Nm)
	requested_current	= 60		# Current sensing limit [A]
	# Not touching current control gains current_gain and
	# current_integrator_gain - they are automatically set
	# for a given current control bandwidth.

	# Prompt user for output filename
	tstamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
	out_filename = get_output_filename('out_endurance_test_'+tstamp+'.csv', stdscr)
	
	# curses: disable echo
	curses.noecho()
	# curses: make getch() non-blocking
	stdscr.nodelay(1)
	line = 0
	
	# Info
	stdscr.addstr(line,0,'This is the endurance test. It will run for up to '+str(max_runtime)+' s., or until stopped.'); line += 2

	# Find a connected ODrive (this will block until you connect one)
	stdscr.addstr(line,0,'Finding odrive..'); line += 1
	my_drive = odrive.find_any()

	# First set the reference position to whatever the current position is
	input_pos_initial = my_drive.axis0.encoder.pos_estimate
	my_drive.axis0.controller.input_pos = input_pos_initial
	stdscr.addstr(line,0,'Initial position offset is '+str(round(turns2rad*input_pos_initial/n,2))+' rad.'); line += 1

	# Set control mode
	# "Trajectory control" - essentially minimum jerk
	stdscr.addstr(line,0,'Setting control mode and parameters..'); line += 1
	my_drive.axis0.controller.config.control_mode	= CONTROL_MODE_POSITION_CONTROL
	my_drive.axis0.controller.config.input_mode		= INPUT_MODE_TRAP_TRAJ
	my_drive.axis0.trap_traj.config.vel_limit		= n * max_speed / turns2rad
	my_drive.axis0.trap_traj.config.accel_limit		= n * max_accel / turns2rad
	my_drive.axis0.trap_traj.config.decel_limit		= n * max_accel / turns2rad
	my_drive.axis0.controller.config.inertia		= inertia

	# Set control gains
	my_drive.axis0.controller.config.pos_gain = pos_gain
	my_drive.axis0.controller.config.vel_gain = vel_gain
	my_drive.axis0.controller.config.vel_integrator_gain = vel_integrator_gain

	# Set current limit
	#my_drive.axis0.motor.config.current_lim = current_lim
	#my_drive.axis0.motor.config.requested_current_range = requested_current

	stdscr.addstr(line,0,'Starting closed loop control..'); line += 1
	my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	time.sleep(1.0)

	# Record data
	stdscr.addstr(line,0,'Recording data..'); line += 1
	data = []
	t0 = time.monotonic()
	target_pos_prev = None

	# Until we break..
	while True:
		# Get iteration start time
		t = time.monotonic()
		
		# curses: get keyboard input, returns -1 if none available
		c = stdscr.getch()
		
		# Check whether we were asked to stop
		if c == ord('q'):
			# Clear the two lines used for showing time and set point during running.
			stdscr.move(line+2, 0); stdscr.clrtoeol(); stdscr.move(line+3, 0); stdscr.clrtoeol();
			
			# Advance two lines and break out of the infinite loop
			line += 2
			break

		# Get data
		motorPos    = my_drive.axis0.encoder.pos_estimate
		outputPos   = 0#my_drive.axis1.encoder.pos_estimate # Encoder not available, and not really needed
		motorVel    = my_drive.axis0.encoder.vel_estimate
		outputVel   = 0#my_drive.axis1.encoder.vel_estimate # Encoder not available, and not really needed
		Iq_set		= my_drive.axis0.motor.current_control.Iq_setpoint
		Iq_meas		= my_drive.axis0.motor.current_control.Iq_measured
		data.append([
			t-t0,
			motorPos,
			outputPos,
			motorVel,
			outputVel,
			my_drive.axis0.controller.input_pos,
			Iq_set,
			Iq_meas,
		])
		
		# If we have reached the maximum runtime, stop here.
		if t-t0 >= max_runtime:
			break

		# Compute target position at current time
		target_pos = motionProfile(time.monotonic() - t0)
		
		# If the target pos has changed, update it and display on screen
		if target_pos != target_pos_prev:
			# Compute new target position for motor
			target_pos_motor = input_pos_initial + n * target_pos / turns2rad # Motor pos in [turns]
			
			stdscr.addstr(line+2,0,'Running. t = '+str(round(t-t0))+' s.')
			stdscr.addstr(line+3,0,'Set point: '+str(round(target_pos_motor*turns2rad/n,2))+' rad.')
			
			# Set new set point
			my_drive.axis0.controller.input_pos = target_pos_motor
			
			# Save current set point as previous
			target_pos_prev = target_pos

		# Sleep for loop
		try:
			time.sleep(sample_time - (time.monotonic()-t))
		except:
			print('    WARNING: Iteration took longer than sample time of', sample_time, 's')

	# Move back to initial position (should already be there?)
	line += 1; stdscr.addstr(line,0,'Finished, setting initial position (should be no move, in which case remove this)..'); line += 1
	my_drive.axis0.controller.input_pos = input_pos_initial
	c = stdscr.getch()
	time.sleep(5)

	# Stop controller
	line += 1; stdscr.addstr(line,0,'Finished, stopping closed-loop control..'); line += 1
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
