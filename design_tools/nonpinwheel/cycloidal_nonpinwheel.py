# cycloidal_nonpinwheel.py

# Copyright © 2023 Wesley Roozing and Glenn Roozing

# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files(the “Software”), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and / or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Imports
import numpy as np
import scipy as sp
import matplotlib.pyplot as pyplot
import math
import os
import shutil
import time

# Cycloidal curve generator
from cycloidal_curves import *


###############################################################################
# Functions

# Rotate a vector consisting of a tuple
def rotate(v, theta):
	if type(v) is np.ndarray:
		for i,vi in enumerate(v):
			(x,y) = rotate((vi[0],vi[1]),theta)
			v[i][0] = x
			v[i][1] = y

		return v
	else:
		# Get x/y coordinates of point
		x = v[0]
		y = v[1]

		# Return new tuple
		return (	x * np.cos(theta) - y * np.sin(theta),
					x * np.sin(theta) + y * np.cos(theta)		)


# Move points by a vector m
def move(v, m):
	if type(v) is np.ndarray:
		v = v+np.array(m)
		return v
	else:
		return (v[0]+m[0], v[1]+m[1])


###############################################################################
# Parameters

design = cycloidal_design()

# Major parameters
design.R = 83/2		# Rotor radius [mm]
design.N = 12		# Number of rollers [] - needs to be even for two disks
design.No = 6		# Number of output pins/holes [] - needs to be even for two disks

# Compute gear ratio
g = design.g

# Hardware parameters
bearing_outer = 9/2				# Roller/pin bearing outer radius [mm]
bearing_lg_outer = 32/2			# Eccentric bearing outer radius [mm]

# Dependent parameters
design.Rr = bearing_outer		# Roller radius [mm]
design.Ro = bearing_outer		# Output pin radius [mm]
design.Lo = 0.62 * design.R		# Output hole midpoint location (radially) [mm]
design.E = 0.45 * design.Rr		# Eccentricity [mm]
design.Re = bearing_lg_outer	# Input shaft radius [mm]

design.maxDist = 0.005 * design.Rr

# Show design info
design_info(design)


###############################################################################
# Point generation

print('Generating cycloidal disk...')
(points, numPoints) = generate_disk(design, debug=False)

print('Generating outer profile...')
(points_outer, numPoints_outer, psi_1_list) = generate_outer_profile(design, debug=False)


###############################################################################
# Compute distances between generated points

dist_list = []
for i in range(0, numPoints_outer-1):
	p0 = points_outer[i]
	p1 = points_outer[i+1]
	dist_list.append(getDist(p0[0], p0[1], p1[0], p1[1]))


###############################################################################
# Plots

# Do we want to make a video?
makeVideo = False

# Plot two disks?
twoDisks = False

# Zoom in to one pin?
zoom = False

# Get design parameters for plotting
# Major parameters
R = design.R	# Rotor radius [mm]
N = design.N	# Number of rollers [] - needs to be even for two disks
No = design.No	# Number of output pins/holes [] - needs to be even for two disks
Rr = design.Rr	# Roller radius [mm]
Ro = design.Ro	# Output pin radius [mm]
Lo = design.Lo	# Output hole midpoint location (radially) [mm]
E = design.E	# Eccentricity [mm]
Re = design.Re	# Input shaft radius [mm]

# Dependent parameters
Roh = design.Roh # Output hole radius [mm]

# Prepare some variables for video
if makeVideo:
	j = 0 # Frame counter
	theta_end = 2*np.pi/g # One output rotation
	fps = 60
	slowmo = 1/4 # Slow-motion factor (<1 for slow-motion)
	n = round(fps/g/slowmo) # fps frames per input rotation

	# Make sure there is a directory for rendered frames
	if not os.path.isdir('/tmp/frames'):
		os.mkdir('/tmp/frames')
else:
	theta_end = 2*np.pi # One input rotation
	n = 1 # Number of frames in total

# For a range of thetas..
for theta in np.linspace(0, theta_end, n):
	# Print current value of theta
	if makeVideo:
		print('Rendering theta =', round(theta,2), 'rad')

	# Figure and axes
	fig = pyplot.figure(figsize=(10,10))
	ax = pyplot.axes()

	# x=0 and y=0 grid lines
	ax.plot((-R-1,R+1),(0,0), color=(0.8,0.8,0.8))
	ax.plot((0,0),(-R-1,R+1), color=(0.8,0.8,0.8))

	# Rotor radius
	c = pyplot.Circle((0,0), R, fill=False)
	ax.add_artist(c)

	# Input shaft and axis of rotation
	c = pyplot.Circle(rotate((E,0),theta), Re, color='g')
	ax.add_artist(c)
	c = pyplot.Circle(rotate((E,0),theta), 0.5, color='k')
	ax.add_artist(c)
	c = pyplot.Circle((0,0), 1.0, color='k')
	ax.add_artist(c)

	# Show a line depicting the input axis
	p = rotate((R,0),theta)
	ax.plot((0,p[0]), (0,p[1]), color='b')

	# Plot of rotor curve

	# Get numpy array of points
	dt=np.dtype('float,float')
	points_arr = np.array(points)
	points_arr2 = points_arr.copy()

	# Transform in three steps:
	# 1. Rotate around rollers
	# 2. Offset by eccentricity
	# 3. Rotate with input shaft
	points_arr = rotate(points_arr, -N*g*theta) # Guessed this angle heuristically but it works
	points_arr = rotate(points_arr, -math.pi/2) # Hsieh, 2014.
	points_arr = move(points_arr, (E,0)) # Hsieh, 2014.
	points_arr = rotate(points_arr, theta)
	ax.plot(points_arr[:,0], points_arr[:,1], color='b')

	# Compute rotational offset for second disk and its holes
	# This doesn't seem to be necessary
	offset = 0

	# Plot a second disk
	if twoDisks:
		points_arr2 = rotate(points_arr2, -N*g*theta+offset)
		points_arr2 = rotate(points_arr2, -math.pi/2)
		points_arr2 = move(points_arr2, (E,0))
		points_arr2 = rotate(points_arr2, theta+math.pi)
		ax.plot(points_arr2[:,0], points_arr2[:,1], color='pink')

	# Plot the outer curve
	points_outer_arr = np.array(points_outer)
	points_outer_arr = rotate(points_outer_arr, -math.pi/2 + math.pi/N) # To align with our original pins
	ax.plot(points_outer_arr[:,0], points_outer_arr[:,1], color='k')

	# Add rollers using circles
	for i in range(0,N):
		x = R * math.cos(i * 2 * math.pi / N)
		y = R * math.sin(i * 2 * math.pi / N)
		c = pyplot.Circle((x,y), Rr, color=(0.8,0.8,0.8), fill=False)
		ax.add_artist(c)

	# Add output holes on rotor
	for i in range(0,No):
		# Get locations of output holes on disk
		x = Lo * math.cos(i * 2 * math.pi / No)
		y = Lo * math.sin(i * 2 * math.pi / No)

		# Same three transform steps as above
		p = rotate((x,y), -N*g*theta)
		p = move(p, (E,0))
		p = rotate(p, theta)

		# Plot
		c = pyplot.Circle(p, Roh, color='b', fill=False)
		ax.add_artist(c)

		# If we are plotting two disks..
		if twoDisks:
			# Same procedure for second disk holes
			p = rotate((x,y), -N*g*theta+offset)
			p = move(p, (E,0))
			p = rotate(p, theta+math.pi)

			# Plot
			c = pyplot.Circle(p, Roh, color='pink', fill=False)
			ax.add_artist(c)

	# Add output pins
	for i in range(0,No):
		x = Lo * math.cos(i * 2 * math.pi / No)
		y = Lo * math.sin(i * 2 * math.pi / No)
		c = pyplot.Circle(rotate((x,y),-theta*g), Ro, color='r')
		ax.add_artist(c)

	# Show output axis line
	p = rotate((R,0),-theta*g)
	ax.plot((0,p[0]), (0,p[1]), color='r')

	# Plot properties
	if zoom:
		z = 3*Rr
		ax.set_xlim(R-z, R+z)
		ax.set_ylim(-z, +z)
	else:
		ax.set_xlim(-R-10, R+10)
		ax.set_ylim(-R-10, R+10)
	ax.set_title('theta = ' + str(round(theta,2)) + ' rad')

	if makeVideo:
		fig.savefig('/tmp/frames/frame_' + str(j) + '.png')
		pyplot.close(fig)
		j=j+1


# Plot thetas
# Enable to show solutions to the meshing equation
# fig2 = pyplot.figure(figsize=(7,7))
# ax2 = pyplot.axes()
# ax2.plot(psi_1_list)
# ax2.set_title('Solutions for psi_1')

# Plot distances between points
# Enable to show distances between generated points (in mm)
# fig2 = pyplot.figure(figsize=(7,7))
# ax2 = pyplot.axes()
# ax2.plot(dist_list)
# ax2.set_title('Distances between points')

# Display final state
print("Final state:")
print("Input angle:", round(theta,3), "rad, output angle:", -round(theta*g,3), "rad.")

# Create video
if makeVideo:
	print('Converting to mkv video file..')
	os.system('ffmpeg -f image2 -r ' + str(fps) + ' -i /tmp/frames/frame_%d.png video.mp4')
	shutil.rmtree('/tmp/frames', ignore_errors=True) # Delete non-empty directory
