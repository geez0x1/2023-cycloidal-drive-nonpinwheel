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
filename	= get_filename('../experiment_code/out_endurance_test.csv')
data		= np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos	= data[:,1]
outputPos	= data[:,2]
motorVel	= data[:,3]
outputVel	= data[:,4]
input_pos	= data[:,5]
Iq_set		= data[:,6]
Iq_meas		= data[:,7]

# Get timestep dt from the mean difference between subsequent timesteps
dt = np.mean(np.diff(t))

# Filter data
motorVel_filt	= filter_iir(motorVel, alpha)
Iq_meas_filt	= filter_iir(Iq_meas, alpha)

# Set the initial positions to zero
motorPos	= motorPos - motorPos[0]
input_pos	= input_pos - input_pos[0]

# Compute torque reference and measurement
tau_m_set	= Iq_set * k_t * n
tau_m		= Iq_meas_filt * k_t * n

# Find the start of each motion sequence
first_setpoint = math.pi # First setpoint in the motion profile defined in the experiment
first_input_pos = n * first_setpoint / turns2rad # First value in input_pos (motor turns)
eps = 1e-3 # Threshold for equality (floating point comparison)
motion_profile_start_idxs = []
for i in range(input_pos.size-1):
	if abs(input_pos[i]) <= eps and abs(input_pos[i+1]-first_input_pos) <= eps:
		#print('Found start at i=', i)
		motion_profile_start_idxs.append(i)

#print('Found motion profile start indices:', motion_profile_start_idxs)
#print('Found motion profile start times:', np.round(t[motion_profile_start_idxs],1), 's.')
print('Found', len(motion_profile_start_idxs), 'motion profile starts.')

# Compute some statistics

# Get start indices of the first and the next profile to
# get the length of one full profile. This determines array size.
idx1 = motion_profile_start_idxs[0]
idx2 = motion_profile_start_idxs[1]
pos_mean			= np.zeros(idx2-idx1)
pos_variance		= np.zeros(idx2-idx1)
tau_m_mean		= np.zeros(idx2-idx1)
tau_m_variance	= np.zeros(idx2-idx1)

# Step through each timestep and compute mean and variance
# between the different profiles
for i in range(0, pos_mean.size):
	# Take all but the last profile as it may not be complete
	idxs			= np.array(motion_profile_start_idxs[1:-1])+i
	#print('idxs (', idxs.size, 'of them):', idxs)

	pos_mean[i]			= np.mean(motorPos[idxs])
	pos_variance[i]		= np.var(motorPos[idxs])
	tau_m_mean[i]		= np.mean(tau_m[idxs])
	tau_m_variance[i]	= np.var(tau_m[idxs])

# Show some info on statistics
print('Mean output position variance:', np.mean(turns2rad*pos_variance/n), 'rad.')
print('Mean torque variance:', np.mean(tau_m_variance), 'Nm.')


# Export data for pgfplots
if export:
	data = (
		t,
		turns2rad*input_pos/n,
		turns2rad*motorPos/n,
		turns2rad*motorVel/n,
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
		headers = ['t', 'input_pos_reduced', 'motorPos_reduced', 'motorVel_reduced', 'tau_m_set', 'tau_m', 'Iq_set', 'Iq_meas', 'Iq_meas_filt'],
		filename_base = 'demo1'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(2,2)
fig.suptitle('Demo 1 (p2p position control): ' + filename)

# Motor position and velocity, and reference, over time
ax[0,0].plot(t, turns2rad*input_pos/n, '--', color=c.reference, label='input_pos/n')
ax[0,0].plot(t, turns2rad*motorPos/n, color=c.motorPos, label='motorPos/n')
ax[0,0].plot(t, turns2rad*motorVel/n, color=c.outputPos, label='motorVel_reduced')
ax[0,0].set_title('Reference and measured position')
ax[0,0].set_xlabel('Time [s]')
ax[0,0].set_ylabel('Position [rad] and velocity [rad/s]')
ax[0,0].legend()
ax[0,0].grid()

# For each found motion profile start...
for i in range(len(motion_profile_start_idxs)):
	# Get start and end index (next profile minus one, or end of array)
	idx1 = motion_profile_start_idxs[i]
	if i == len(motion_profile_start_idxs) - 1:
		idx2 = input_pos.size - 1
	else:
		idx2 = motion_profile_start_idxs[i+1] - 1
	
	# Debug
	#print('Ploting indices:', idx1, idx2)
	
	# Get time vector
	t_plot = t[idx1:idx2]-t[idx1]
	
	# Motor position and velocity, and reference, over time
	ax[0,1].plot(t_plot, turns2rad*input_pos[idx1:idx2]/n, '--', color=c.reference, label='input_pos/n')
	ax[0,1].plot(t_plot, turns2rad*motorPos[idx1:idx2]/n, color=c.motorPos, label='motorPos/n')
	ax[0,1].plot(t_plot, turns2rad*motorVel[idx1:idx2]/n, color=c.outputPos, label='motorVel_reduced')
	
	# Current
	ax[1,0].plot(t_plot, Iq_meas[idx1:idx2], color=(0.8,0.8,0.8), label='Iq_meas')
	ax[1,0].plot(t_plot, Iq_meas_filt[idx1:idx2], color=(0.4,0.4,0.4), label='Iq_meas_filt')
	ax[1,0].plot(t_plot, Iq_set[idx1:idx2], '--', color=(0,0,0), label='Iq_set')
	
	# Torque
	ax[1,1].plot(t_plot, tau_m[idx1:idx2], color=(0.4,0.4,0.4), label='Torque')
	ax[1,1].plot(t_plot, tau_m_set[idx1:idx2], color=(0,0,0), label='Reference')

# Variance - motorPos
clr = list(c.motorPos)
clr.append(0.4)
clr = tuple(clr)
ax[0,1].fill_between(dt*range(pos_mean.size), turns2rad*(pos_mean-500*pos_variance)/n, turns2rad*(pos_mean+500*pos_variance)/n, color=clr, label='Variance', zorder=-10)

# Variance - tau_m
clr = (0.4,0.4,0.4,0.4)
ax[1,1].fill_between(dt*range(tau_m_mean.size), tau_m_mean-50*tau_m_variance, tau_m_mean+50*tau_m_variance, color=clr, label='Variance', zorder=-10)

# Motor position and velocity, and reference, over time
ax[0,1].set_title('Reference and measured position')
ax[0,1].set_xlabel('Time [s]')
ax[0,1].set_ylabel('Position [rad] and velocity [rad/s]')
#ax[0,1].legend()
ax[0,1].grid()

# Current
ax[1,0].set_title('Current')
ax[1,0].set_xlabel('Time [s]')
ax[1,0].set_ylabel('Current [A]')
#ax[1,0].legend()
ax[1,0].grid()

# Torque
ax[1,1].set_title('Torque')
ax[1,1].set_xlabel('Time [s]')
ax[1,1].set_ylabel('Torque [Nm]')
#ax[1,1].legend()
ax[1,1].grid()

plt.show()
