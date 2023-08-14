import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_linear, fit_linear_no_offset, append_filename, export_data, reduce_data, fit_piecewise_linear_no_offset, fit_linear_bidirectional
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
k_t			= 0.082699 # Motor torque constant [Nm/A]
#alpha		= 0.50	# Filtering constant for first-order IIR filter
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad
export		= True	# Whether to export data for pgfplots
est_func		= fit_linear_bidirectional # Function to use for estimation of stiffness

# Load data
filename	= get_filename('../experiment_code/out_experiment_3_stiffness_current.csv')
data		= np.genfromtxt(filename, delimiter=",")

# Get fields
t			= data[:,0]
motorPos		= data[:,1]
outputPos	= -data[:,2]
motorVel		= data[:,3]
outputVel	= -data[:,4]
input_pos	= data[:,5]
Iq_set		= data[:,6]
Iq_meas		= data[:,7]

# Get timestep dt from the mean difference between subsequent timesteps
dt = np.mean(np.diff(t))

# Filter data
Iq_meas_filt = filter_iir(Iq_meas, f=2.0, dt=dt)

# Set the mean position to zero
motorPos		= motorPos - np.mean(motorPos)

# Compute the position difference, remove offset
diff		= (motorPos/n) - 0

# Compute applied torque
tau			= n * k_t * Iq_meas
tau_filt		= n * k_t * Iq_meas_filt

# Do curve fitting and estimate stiffness by averaging the two slopes
(opt_params, parms_cov) = curve_fit(est_func, tau, diff)
k_est_1		= 1 / (turns2rad*opt_params[0]) # [Nm/rad]
k_est_2		= 1 / (turns2rad*opt_params[2]) # [Nm/rad]
k_estimate	= np.mean([k_est_1, k_est_2])

# Output information on fit
print('Curve fitting parameters:')
print(opt_params)
print('Estimated stiffness:', k_estimate, 'Nm/rad')

# Export data for pgfplots
if export:
	data = (
		tau,
		tau_filt,
		turns2deg*diff,
		Iq_set,
		Iq_meas,
		Iq_meas_filt,
	)

	# Reduce data
	#dt = 0.1
	#(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data,
		headers = ['tau', 'tau_filt', 'diff', 'Iq_set', 'Iq_meas', 'Iq_meas_filt'],
		filename_base = 'exp3_current'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,3)
fig.suptitle('Experiment 3 (stiffness through current): ' + filename)

# Motor position over time
ax[0].plot(t, turns2deg*motorPos/n, color=c.motorPos, label='motorPos/n')
ax[0].set_title('Motor and output position')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Position [deg]')
ax[0].legend()

# Difference
ax[1].plot(tau, turns2deg*diff, color=c.rel_pos, label='Difference')
ax[1].plot(tau, turns2deg*(est_func(tau, *opt_params)), '--', color='brown', label='Linear fit ('+str(round(k_estimate))+' Nm/rad)')
ax[1].set_title('Internal deflection (input vs fixed output)')
ax[1].set_xlabel('Torque [Nm]')
ax[1].set_ylabel('Deflection [deg]')
ax[1].legend()

# Current
ax[2].plot(t, Iq_meas, color=(0.8,0.8,0.8), label='Iq_meas')
ax[2].plot(t, Iq_meas_filt, color=(0.4,0.4,0.4), label='Iq_meas_filt')
ax[2].plot(t, Iq_set, '--', color=(0,0,0), label='Iq_set')
ax[2].set_title('Current')
ax[2].set_xlabel('Time [s]')
ax[2].set_ylabel('Current [A]')
ax[2].legend()

plt.show()
