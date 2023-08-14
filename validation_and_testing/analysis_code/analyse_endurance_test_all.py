import sys
import os
import glob
import math
from types import SimpleNamespace
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
export		= True	# Whether to export data for pgfplots

# Define filenames
path = '../2023-07-25_endurance_tests/rev1_proto5_pinwheel/'
#path = '../2023-07-25_endurance_tests/rev2_proto1_nonpinwheel/'
filenames = glob.glob(path+'*.csv')

# Load data for each file
datasets = []
for filename in filenames:
	# Load this file
	d = np.genfromtxt(filename, delimiter=",")

	# Create simple namespace with raw data and other computed fields
	exp = SimpleNamespace()

	# Get fields
	exp.t			= d[:,0]
	exp.motorPos		= d[:,1]
	exp.outputPos	= d[:,2]
	exp.motorVel		= d[:,3]
	exp.outputVel	= d[:,4]
	exp.input_pos	= d[:,5]
	exp.Iq_set		= d[:,6]
	exp.Iq_meas		= d[:,7]
	exp.filename		= os.path.basename(filename)

	# Get timestep dt from the mean difference between subsequent timesteps
	exp.dt = np.mean(np.diff(exp.t))

	# Filter data
	exp.motorVel_filt	= filter_iir(exp.motorVel, alpha)
	exp.Iq_meas_filt		= filter_iir(exp.Iq_meas, alpha)
	
	# Set the initial positions to zero
	# We re-zero the motorPos later as the first motion profile may not
	# be representative of the whole set. The input (reference) can be
	# assumed to be consistent.
	exp.motorPos			= exp.motorPos - exp.motorPos[0]
	exp.input_pos		= exp.input_pos - exp.input_pos[0]
	
	# Compute torque reference and measurement
	exp.tau_m_set		= exp.Iq_set * k_t * n
	exp.tau_m			= exp.Iq_meas_filt * k_t * n

	# Find the start of each motion sequence
	first_setpoint = math.pi # First setpoint in the motion profile defined in the experiment
	first_input_pos = n * first_setpoint / turns2rad # First value in input_pos (motor turns)
	eps = 1e-3 # Threshold for equality (floating point comparison)
	exp.motion_profile_start_idxs = []
	for i in range(exp.input_pos.size-1):
		if abs(exp.input_pos[i]) <= eps and abs(exp.input_pos[i+1]-first_input_pos) <= eps:
			#print('Found start at i=', i)
			exp.motion_profile_start_idxs.append(i)

	#print('Found motion profile start indices:', motion_profile_start_idxs)
	#print('Found motion profile start times:', np.round(t[motion_profile_start_idxs],1), 's.')
	print('Found', len(exp.motion_profile_start_idxs), 'motion profile starts.')

	# Before computing statistics, re-zero the motorPos using the mean of
	# all the starting positions.
	exp.motorPos = exp.motorPos - np.mean(exp.motorPos[exp.motion_profile_start_idxs])

	# Compute some statistics
	
	# Get start indices of the first and the next profile to
	# get the length of one full profile. This determines array size.
	idx1 = exp.motion_profile_start_idxs[0]
	idx2 = exp.motion_profile_start_idxs[1]
	exp.pos_mean			= np.zeros(idx2-idx1)
	exp.pos_variance		= np.zeros(idx2-idx1)
	exp.vel_mean			= np.zeros(idx2-idx1)
	exp.vel_variance		= np.zeros(idx2-idx1)
	exp.Iq_set_mean		= np.zeros(idx2-idx1)
	exp.Iq_set_variance	= np.zeros(idx2-idx1)
	exp.Iq_meas_mean		= np.zeros(idx2-idx1)
	exp.Iq_meas_variance	= np.zeros(idx2-idx1)
	exp.tau_m_mean		= np.zeros(idx2-idx1)
	exp.tau_m_variance	= np.zeros(idx2-idx1)
	
	# Step through each timestep and compute mean and variance
	# between the different profiles
	for i in range(0, exp.pos_mean.size):
		# Take all but the last profile as it may not be complete
		idxs			= np.array(exp.motion_profile_start_idxs[1:-1])+i
		#print('idxs (', idxs.size, 'of them):', idxs)

		exp.pos_mean[i]			= np.mean(exp.motorPos[idxs])
		exp.pos_variance[i]		= np.var(exp.motorPos[idxs])
		exp.vel_mean[i]			= np.mean(exp.motorVel[idxs])
		exp.vel_variance[i]		= np.var(exp.motorVel[idxs])
		exp.Iq_set_mean[i]		= np.mean(exp.Iq_set[idxs])
		exp.Iq_set_variance[i]	= np.var(exp.Iq_set[idxs])
		exp.Iq_meas_mean[i]		= np.mean(exp.Iq_meas_filt[idxs]) # filt
		exp.Iq_meas_variance[i]	= np.var(exp.Iq_meas_filt[idxs]) # filt
		exp.tau_m_mean[i]		= np.mean(exp.tau_m[idxs])
		exp.tau_m_variance[i]	= np.var(exp.tau_m[idxs])

	# Time vector for plotting
	exp.t_plot = exp.t[idx1:idx2] - exp.t[idx1]

	# Show some info on statistics
	print('Mean output position variance:', np.mean(turns2rad*exp.pos_variance/n), 'rad.')
	print('Mean output velocity variance:', np.mean(turns2rad*exp.vel_variance/n), 'rad/s.')
	print('Mean current variance:', np.mean(exp.Iq_set_variance), 'A set or', np.mean(exp.Iq_meas_variance), 'A measured.')
	print('Mean torque variance:', np.mean(exp.tau_m_variance), 'Nm.')

	# Append object
	datasets.append(exp)

	print('----')

# Hour-to-hour statistics

# There appear to be some issues with this - the differences are so small
# that indexing of vectors (that may not be identical in time) causes
# incorrect results.

# stats = SimpleNamespace()

# # Size of the cross-experiment statistics is determined by the shortest one
# N = np.min([exp.pos_mean.size for exp in datasets])

# # Allocate same fields as before
# stats.pos_mean			= np.zeros(N)
# stats.pos_variance		= np.zeros(N)
# stats.vel_mean			= np.zeros(N)
# stats.vel_variance		= np.zeros(N)
# stats.Iq_set_mean		= np.zeros(N)
# stats.Iq_set_variance	= np.zeros(N)
# stats.Iq_meas_mean		= np.zeros(N)
# stats.Iq_meas_variance	= np.zeros(N)
# stats.tau_m_mean			= np.zeros(N)
# stats.tau_m_variance		= np.zeros(N)

# # Step through each timestep and compute mean and variance
# # between the different MEANS for each hour
# for i in range(0, N):
# 	stats.pos_mean[i]			= np.mean([exp.pos_mean[i] for exp in datasets])
# 	stats.pos_variance[i]		= np.var([exp.pos_mean[i] for exp in datasets])
# 	stats.vel_mean[i]			= 0
# 	stats.vel_variance[i]		= 0
# 	stats.Iq_set_mean[i]			= 0
# 	stats.Iq_set_variance[i]		= 0
# 	stats.Iq_meas_mean[i]		= np.mean([exp.Iq_meas_mean[i] for exp in datasets])
# 	stats.Iq_meas_variance[i]	= np.var([exp.Iq_meas_mean[i] for exp in datasets])
# 	stats.tau_m_mean	[i]			= 0
# 	stats.tau_m_variance[i]		= 0

# # Time vector that is the same for all
# stats.t_plot = datasets[0].t_plot


# Export data for pgfplots
if export:
	for exp in datasets:
	 	data = (
			exp.t_plot,
			turns2rad*exp.pos_mean/n,
			turns2rad*exp.pos_variance/n,
			turns2rad*exp.vel_mean/n,
			turns2rad*exp.vel_variance/n,
			exp.Iq_meas_mean,
			exp.Iq_meas_variance,
	 	)
	
	 	# Reduce data
	 	#dt = 0.1
	 	#(t_RS, data_RS) = reduce_data(exp.t_plot, dt, data)
	
	 	export_data(
			data,
			headers = ['t', 'pos_mean', 'pos_variance',
					  'vel_mean', 'vel_variance',
					  'Iq_meas_mean', 'Iq_meas_variance'],
			filename_base = os.path.splitext(exp.filename)[0]+'_plot'
	 	)


# Plotting

# Figure and axes
(fig, ax) = plt.subplots(2,2)
fig.suptitle('Endurance test (p2p position control): Multiple datasets')

# Motor position over time
for i, exp in enumerate(datasets):
	# Plot mean - motorPos
	ax[0,0].plot(exp.t_plot, turns2rad*exp.pos_mean/n, label='Hour '+str(i))

	# Plot variance - motorPos
	#clr = list(c.motorPos)
	#clr.append(0.2)
	#clr = tuple(clr)
	#ax[0,0].fill_between(exp.t_plot, turns2rad*(exp.pos_mean-100*exp.pos_variance)/n, turns2rad*(exp.pos_mean+100*exp.pos_variance)/n, color=clr, zorder=-10)

# Plot variance BETWEEN HOURS - motorPos
# clr = list(c.motorPos)
# clr.append(0.2)
# clr = tuple(clr)
# ax[0,0].plot(stats.t_plot, turns2rad*stats.pos_mean/n, color='black', label='Mean')
# ax[0,0].fill_between(stats.t_plot, turns2rad*(stats.pos_mean-1000*stats.pos_variance)/n, turns2rad*(stats.pos_mean+1000*stats.pos_variance)/n, color=clr, zorder=-10)

ax[0,0].set_title('Measured position')
ax[0,0].set_xlabel('Time [s]')
ax[0,0].set_ylabel('Position [rad]')
ax[0,0].legend()
ax[0,0].grid()

# Motor position and velocity over time
for i, exp in enumerate(datasets):
	ax[0,1].plot(exp.t_plot, turns2rad*exp.vel_mean/n, label='Hour '+str(i))

	# Plot variance - motorVel
	#clr = list(c.motorPos)
	#clr.append(0.2)
	#clr = tuple(clr)
	#ax[0,1].fill_between(exp.t_plot, turns2rad*(exp.vel_mean-10*exp.vel_variance)/n, turns2rad*(exp.vel_mean+10*exp.vel_variance)/n, color=clr, zorder=-10)

ax[0,1].set_title('Measured velocity')
ax[0,1].set_xlabel('Time [s]')
ax[0,1].set_ylabel('Velocity [rad/s]')
ax[0,1].legend()
ax[0,1].grid()

# Current
for i, exp in enumerate(datasets):
	#ax[1,0].plot(exp.t_plot, exp.Iq_set_mean, label='Hour '+str(i))
	ax[1,0].plot(exp.t_plot, exp.Iq_meas_mean, label='Hour '+str(i))

	# Plot variance - Iq_meas
	#clr = list(c.motorPos)
	#clr.append(0.2)
	#clr = tuple(clr)
	#ax[1,0].fill_between(exp.t_plot, exp.Iq_meas_mean-10*exp.Iq_meas_variance, exp.Iq_meas_mean+10*exp.Iq_meas_variance, color=clr, zorder=-10)

# Plot variance BETWEEN HOURS - Iq_meas
# clr = list(c.motorPos)
# clr.append(0.2)
# clr = tuple(clr)
# ax[1,0].plot(stats.t_plot, stats.Iq_meas_mean, color='black', label='Mean')
# ax[1,0].fill_between(stats.t_plot, stats.Iq_meas_mean-10*stats.Iq_meas_variance, stats.Iq_meas_mean+10*stats.Iq_meas_variance, color=clr, zorder=-10)

ax[1,0].set_title('Current')
ax[1,0].set_xlabel('Time [s]')
ax[1,0].set_ylabel('Current [A]')
#ax[1,0].legend()
ax[1,0].grid()

plt.show()
