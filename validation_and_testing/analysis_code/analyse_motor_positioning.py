import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_linear, fit_linear_no_offset, append_filename, export_data, reduce_data
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
k_t			= 0.082699 # Motor torque constant [Nm/A]
alpha		= 0.5	# Filtering constant for first-order IIR filter
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad
export		= False	# Whether to export data for pgfplots

# Load data
filename	= get_filename('../experiment_code/out_motor_positioning.csv')
data		= np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= -data[:,2]
motorVel	= data[:,3]
outputVel	= -data[:,4]
input_pos	= data[:,5]
Iq_set		= data[:,6]
Iq_meas		= data[:,7]

# Get timestep dt from the mean difference between subsequent timesteps
dt = np.mean(np.diff(t))

# Filter data
motorVel_filt	= filter_iir(motorVel, alpha)
outputVel_filt	= filter_iir(outputVel, alpha)
Iq_meas_filt	= filter_iir(Iq_meas, alpha)

# Set the initial positions to zero
outputPos = outputPos - outputPos[0]
motorPos = motorPos - motorPos[0]
input_pos = input_pos - input_pos[0]

# Compute torque reference and measurement
tau_m_set	= Iq_set * k_t * n
tau_m		= Iq_meas_filt * k_t * n

# Export data for pgfplots
if export:
	data = (
		t,
		turns2rad*input_pos/n,
		turns2rad*motorPos/n,
		turns2rad*motorVel/n,
		turns2rad*outputPos,
		turns2rad*outputVel,
		tau_m_set,
		tau_m,
		Iq_set,
		Iq_meas,
		Iq_meas_filt,
	)

	# Reduce data
	#dt = 0.1
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data,
		headers = ['t', 'input_pos_reduced', 'motorPos_reduced', 'motorVel_reduced', 'outputPos', 'outputVel', 'tau_m_set', 'tau_m', 'Iq_set', 'Iq_meas', 'Iq_meas_filt'],
		filename_base = 'demo1'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,4)
fig.suptitle('Motor positioning: ' + filename)

# Motor and output position, and reference, over time
ax[0].plot(t, turns2deg*input_pos/n, '--', color=c.reference, label='input_pos/n')
ax[0].plot(t, turns2deg*motorPos/n, color=c.motorPos, label='motorPos/n')
ax[0].plot(t, turns2deg*outputPos, color=c.outputPos, label='outputPos')
ax[0].set_title('Motor and output position')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Position [deg]')
ax[0].legend()
ax[0].grid()

# Motor speed
ax[1].plot(t, turns2rad*motorVel/n, color=c.motorPos_light)
ax[1].plot(t, turns2rad*outputVel, color=c.outputPos_light)
ax[1].plot(t, turns2rad*motorVel_filt/n, color=c.motorPos, label='Motor speed')
ax[1].plot(t, turns2rad*outputVel_filt, color=c.outputPos, label='Output speed')
ax[1].set_title('Motor and output speed')
ax[1].set_xlabel('Time [s]')
ax[1].set_ylabel('Speed [rad/s]')
ax[1].legend()
ax[1].grid()

# Current
ax[2].plot(t, Iq_meas, color=(0.8,0.8,0.8), label='Iq_meas')
ax[2].plot(t, Iq_meas_filt, color=(0.4,0.4,0.4), label='Iq_meas_filt')
ax[2].plot(t, Iq_set, '--', color=(0,0,0), label='Iq_set')
ax[2].set_title('Current')
ax[2].set_xlabel('Time [s]')
ax[2].set_ylabel('Current [A]')
ax[2].legend()
ax[2].grid()

# Torque
ax[3].plot(t, tau_m, color=(0.4,0.4,0.4), label='Torque')
ax[3].plot(t, tau_m_set, color=(0,0,0), label='Reference')
ax[3].set_title('Torque')
ax[3].set_xlabel('Time [s]')
ax[3].set_ylabel('Torque [Nm]')
ax[3].legend()
ax[3].grid()

plt.show()
