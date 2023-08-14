import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from common import get_filename, filter_iir, fit_sines, export_data, reduce_data, fit_coulomb_viscous, fit_coulomb_viscous_bidirectional
from common import plot_colours as c

# Parameters
n			= 11	# True gear ratio
k_t			= 0.082699 # Motor torque constant [Nm/A]
alpha		= 0.8	# Filtering constant for first-order IIR filter
turns2deg	= 360	# One turn in [deg]
deg2rad		= math.pi/180 # Degree to rad
turns2rad	= turns2deg*deg2rad # Turns to rad
export		= True	# Whether to export data for pgfplots

# Load data
filename = get_filename('../experiment_code/out_experiment_4_peak_speed.csv')
data = np.genfromtxt(filename, delimiter=",")

# Indices of relevant data
idx = np.where(data[:,0])[0] # Default - all data
# Time-based constraint

# 2023-06-15_proto5_and_rev2proto1_run-in
# Same starting time for both
t_min = 4.5
t_max = t_min + 90*60 # 90 min

# Get indices from starting to ending time
idx = np.intersect1d(np.where(data[:,0] >= t_min), np.where(data[:,0] <= t_max))

# Get fields
t			= data[idx,0]
motorPos		= data[idx,1]
outputPos	= data[idx,2]
motorVel		= data[idx,3]
outputVel	= data[idx,4]
input_vel	= data[idx,5]
Iq_set		= data[idx,6]
Iq_meas		= data[idx,7]

# Get timestep dt from the mean difference between subsequent timesteps
dt = np.mean(np.diff(t))

# Check if outputVel is all zeroes; if so, replace by motorVel/n
if not np.any(outputVel):
	outputPos = motorPos/n
	outputVel = motorVel/n
	print('Warning: outputPos and outputVel computed from motorPos and motorVel.')

# Filter data
motorPos_filt	= filter_iir(motorPos, alpha)
outputPos_filt	= filter_iir(outputPos, alpha)
motorVel_filt	= filter_iir(motorVel, alpha)
outputVel_filt	= filter_iir(outputVel, alpha)
Iq_set_filt		= filter_iir(Iq_set, alpha)
Iq_meas_filt		= filter_iir(Iq_meas, alpha)
Iq_meas_filt2	= filter_iir(Iq_meas, f=0.01, dt=dt) # Heavy filtering

# Estimate motor torque
tau_m			= k_t * n * Iq_meas
tau_m_filt		= k_t * n * Iq_meas_filt
tau_m_filt2		= filter_iir(tau_m, f=0.01, dt=dt) # Heavy filtering

# Export data for pgfplots
if export:
	data = (
		t,
		turns2rad*input_vel/n,
		#turns2rad*motorVel,
		#turns2rad*motorVel_filt,
		turns2rad*outputVel,
		turns2rad*outputVel_filt,
		Iq_meas,
		Iq_meas_filt,
		Iq_meas_filt2,
		Iq_set,
		tau_m,
		tau_m_filt,
		tau_m_filt2

	)

	# Reduce data
	dt = 5.0
	(t_RS, data_RS) = reduce_data(t, dt, data)

	export_data(
		data_RS,
		headers = ['t', 'input_vel', 'outputVel', 'outputVel_filt', 'Iq_meas', 'Iq_meas_filt', 'Iq_meas_filt2', 'Iq_set', 'tau_m', 'tau_m_filt', 'tau_m_filt2'],
		filename_base = 'run_in'
	)

# Plotting

# Figure and axes
(fig, ax) = plt.subplots(1,4)
fig.suptitle('Experiment 4 (peak speed): ' + filename)

# Motor speed
ax[0].plot(t, turns2rad*input_vel, '--', color=c.reference, label='Reference')
ax[0].plot(t, turns2rad*motorVel, color=c.motorPos_light)
ax[0].plot(t, turns2rad*motorVel_filt, color=c.motorPos, label='Motor speed')
ax[0].set_title('Motor speed')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Speed [rad/s]')
ax[0].legend()

# Output speed
ax[1].plot(t, turns2rad*input_vel/n, '--', color=c.reference, label='Reference')
ax[1].plot(t, turns2rad*outputVel, color=c.outputPos_light)
ax[1].plot(t, turns2rad*outputVel_filt, color=c.outputPos, label='Output speed')
ax[1].set_title('Output speed')
ax[1].set_xlabel('Time [s]')
ax[1].set_ylabel('Speed [rad/s]')
ax[1].legend()

# Current
ax[2].plot(t, Iq_meas, color=(0.8,0.8,0.8), label='Iq_meas')
ax[2].plot(t, Iq_meas_filt, color=(0.4,0.4,0.4), label='Iq_meas_filt')
ax[2].plot(t, Iq_meas_filt2, color=(0.2,0.2,0.2), label='Iq_meas_filt2')
ax[2].plot(t, Iq_set, '--', color=(0,0,0), label='Iq_set')
ax[2].set_title('Current')
ax[2].set_xlabel('Time [s]')
ax[2].set_ylabel('Current [A]')
ax[2].legend()

# Current
#ax[3].plot(t, tau_m, color=(0.8,0.8,0.8), label='tau_m')
ax[3].plot(t, tau_m_filt, color=(0.4,0.4,0.4), label='tau_m_filt')
ax[3].plot(t, tau_m_filt2, color=(0,0,0), label='tau_m_filt2')
ax[3].set_title('Torque')
ax[3].set_xlabel('Time [s]')
ax[3].set_ylabel('Torque [Nm]')
ax[3].legend()

plt.show()
