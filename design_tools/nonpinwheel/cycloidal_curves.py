# cycloidal_curves.py

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
import math

# Class to hold design parameters
class cycloidal_design:
	# Major parameters
	# Note that not all these parameters are used for profile generation
	R = 50		# Rotor radius [mm]
	N = 12		# Number of rollers [] - needs to be even for two disks
	No = 6		# Number of output pins/holes [] - needs to be even for two disks
	Rr = 4		# Roller radius [mm]
	Ro = 4		# Output pin radius [mm]
	Lo = R/2	# Output hole midpoint location (radially) [mm]
	E = Rr/2	# Eccentricity [mm]
	Re = 16		# Input shaft radius [mm]
	
	# Parameters for point/curve generation of the cycloid disk
	maxDist = 0.005 * Rr # Maximum allowed distance between points

	# Output hole radius [mm] - dependent computed property
	@property
	def Roh(self):
		return self.Ro + self.E
	
	# Gear ratio
	@property
	def g(self):
		return self.gear_ratio
	@property
	def gear_ratio(self):
		return 1/(self.N-1)

	# Minimum allowed distance between points
	# minDist for point generation is always half maxDist
	@property
	def minDist(self):
		return 0.5 * self.maxDist


# Get a point along the curve of the cycloidal disk
def getPoint(t, i, R, Rr, E, N):
	return getPoint_Hsieh2014(t, i, R, Rr, E, N)


def getPoint_ShinKwon2006(t, R, Rr, E, N):
	# Contact angle
	psi = math.atan2(math.sin((1-N)*t), ((R/(E*N))-math.cos((1-N)*t)))

	# Geometry of cycloidal disk as function of t
	x = (R*math.cos(t))-(Rr*math.cos(t+psi))-(E*math.cos(N*t))
	y = (-R*math.sin(t))+(Rr*math.sin(t+psi))+(E*math.sin(N*t))
	return (x,y)


# Hsieh, 2014
def getPoint_Hsieh2014(t, i, R, Rr, E, N):
	# Parameters
	c = E		# Eccentricity
	rho = Rr	# Roller radius
	E = R		# Distance to roller center

	# Variables
	phi_2 = t
	phi_1 = phi_2 * (N-1)/N

	# Pin number
	zeta_i = 2*math.pi*(i-1)/N

	# Solution to meshing equation (Eq. 7)
	alpha = math.atan2(math.sin(phi_1 - zeta_i), (E/(c*N))-math.cos(phi_1 - zeta_i))

	# Outer rotor (cycloidal disk) according to Hsieh, 2014 (Eq. 4)
	x = E * math.sin(zeta_i - phi_1 + phi_2) + \
		rho * math.sin(alpha + phi_1 - phi_2 - zeta_i) - \
		c * math.sin(phi_2)
	y = E * math.cos(zeta_i - phi_1 + phi_2) - \
		rho * math.cos(alpha + phi_1 - phi_2 - zeta_i) - \
		c * math.cos(phi_2)

	return (x,y)


# Get a point along the outer profile
def getPoint_outer(t, i, R, Rr, E, N, psi_1_prev):
	return getPoint_outer_Hsieh2014(t, i, R, Rr, E, N, psi_1_prev)


# Hsieh, 2014
def getPoint_outer_Hsieh2014(t, i, R, Rr, E, N, psi_1_prev):
	# Parameters
	c = E		# Eccentricity
	rho = Rr	# Roller radius
	E = R		# Distance to roller center

	# Variables
	phi_2 = t
	phi_1 = phi_2 * (N-1)/N

	# Pin number
	zeta_i = 2*math.pi*(i-1)/N

	# Solution to meshing equation (Eq. 7)
	alpha = math.atan2(math.sin(phi_1 - zeta_i), (E/(c*N))-math.cos(phi_1 - zeta_i))

	# Define shorthands
	a = E / (c*N)
	b = math.sin(zeta_i - phi_1)
	cc = zeta_i - phi_1 + phi_2
	d = rho / (c*N)
	e = math.sin(alpha - zeta_i + phi_1)
	f = alpha
	g = phi_2

	# More shorthands in A*sin + B*cos = C
	A = a * math.cos(cc) - d * math.cos(cc-f) - math.cos(g)
	B = -a * math.sin(cc) + d * math.sin(cc-f) + math.sin(g)
	C = a * b + d * e

	# Solution to the second meshing equation (Eq. 11)
	psi_1 = 2 * math.atan2(A + math.sqrt(A**2 + B**2 - C**2), B + C)

	# Take out multiples of pi
	# If water is wet...
	if 1==1 or abs(psi_1 - psi_1_prev) > 0.1:
		diff = psi_1 - psi_1_prev
		multiples = round(diff / math.pi) * math.pi
		psi_1 = psi_1 - multiples

	if abs(psi_1 - psi_1_prev) > 0.1:
		print('Warning: t =', round(t,3), ', psi_1 =', round(psi_1,3), ', remaining difference with previous is still', round(psi_1 - psi_1_prev,3), 'and I removed', round(multiples,3))

	# Define psi_2
	psi_2 = ((N-1)/N) * psi_1

	# Outer rotor according to Hsieh, 2014 (Eq. 8)
	x = E * math.sin(zeta_i - phi_1 + phi_2 - psi_1 + psi_2) + \
		rho * math.sin(alpha + phi_1 - phi_2 + psi_1 - psi_2 - zeta_i) - \
		c * (math.sin(psi_2) - math.sin(psi_1 - psi_2 - phi_2))
	y = E * math.cos(zeta_i - phi_1 + phi_2 - psi_1 + psi_2) - \
		rho * math.cos(alpha + phi_1 - phi_2 + psi_1 - psi_2 - zeta_i) - \
		c * (math.cos(psi_2) + math.cos(psi_1 - psi_2 - phi_2))

	return (x,y,psi_1)


# Get the Euclidian distance between two points
def getDist(xa, ya, xb, yb):
	return math.sqrt((xa-xb)**2 + (ya-yb)**2)


# Print out design info
def design_info(design):
	# Get design parameters
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
	
	# Parameters for point/curve generation of the cycloid disk
	maxDist = design.maxDist	# Maximum allowed distance between points
	minDist = design.minDist	# Minimum allowed distance between points		
	
	# Compute gear ratio
	g = 1/(N-1)

	# Output major parameters
	print('Rotor radius R =', R, 'mm')
	print('Number of rollers N =', N)
	print('Ratio will be 1/(N-1) = g =', g)
	
	# Check wall thicknesses
	print('Minimum wall thicknesses:')
	
	# Input shaft to output hole
	x_input_right_edge = E + Re
	x_output_left_edge = E + Lo - Roh
	d = x_output_left_edge-x_input_right_edge
	print('    Input shaft to output hole:', round(d,3), 'mm.')
	
	# Output hole to disk edge
	x_output_edge = E + Lo + Roh
	x_disk_edge = E + (R-Rr) # TODO CHECK
	d = x_disk_edge-x_output_edge
	print('    Output hole to disk edge:', round(d,3), 'mm.')
	
	# Output hole to output hole
	x1 = Lo * math.cos(1 * 2 * math.pi / No) + E
	y1 = Lo * math.sin(1 * 2 * math.pi / No)
	x2 = Lo * math.cos(2 * 2 * math.pi / No) + E
	y2 = Lo * math.sin(2 * 2 * math.pi / No)
	d = getDist(x1, y1, x2, y2) - 2 * Roh
	print('    Output hole to output hole:', round(d,3), 'mm.')


# Generate cycloidal disk
def generate_disk(design, debug=False):
	# Get design parameters
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
	
	# Parameters for point/curve generation of the cycloid disk
	maxDist = design.maxDist	# Maximum allowed distance between points
	minDist = design.minDist	# Minimum allowed distance between points	
	
	# Prepare list of points
	points = []
	
	# Get starting point
	(xs, ys) = getPoint(0, 1, R, Rr, E, N)
	points.append((xs,ys))
	#print('(xs, ys) = (', xs, ', ', ys, ')')
	(x, y) = (xs, ys)
	
	# Compute ending point
	et = 2 * math.pi # Angular offset ending point
	
	# Initialise point generation
	dist = 0 # Distance to previous point
	dt = 2*math.pi / N / 100 # Initial step size
	numPoints = 0
	
	# Generate cycloidal disk
	# For each roller pin..
	for i in range(1, N+1):
		if debug:
			print('----')
			print('i =', i)
	
		# Starting angle
		t = 0

		# Flag to make sure we generate a point *at* et (=2pi) each time
		lastPoint = False
	
		# While we have not reached the starting point...
		while (i < N or getDist(xs, ys, x, y) > maxDist):
			if debug:
				print('----')
				print('t =', round(t,4), ', dt =', dt)

			# If our current increment brings us beyond the end point,
			# we generate one more point and end for this value of i.
			if t+dt >= et:
				dt = et - t
				lastPoint = True
				if debug:
					print('Instead generating last point at ', t+dt, 'using dt = ', dt)

			# Get a new point with the existing stepsize
			(xt, yt) = getPoint(t+dt, i, R, Rr, E, N)
	
			# Compute the distance to the previous point
			dist = getDist(x, y, xt, yt)

			if debug:
				print('initial try before adjustment: dist =', dist, 'for point at', (xt, yt))
	
			# Set angle increment adjustment
			ddt = dt/2
			lastTooBig = False
			lastTooSmall = False
	
			# Find a good stepsize dt that puts the new point within a certain distance range
			# If it's the last point, we don't care about minimum distance
			while (not lastPoint and dist < minDist) or dist > maxDist:
				if dist > maxDist:
					# If we're too far away, decrease the stepsize

					# If we went from too small to too big, decrease the size of changes ddt
					if (lastTooSmall):
						ddt /= 2
		
					lastTooSmall = False
					lastTooBig = True
	
					# Don't decrease by more than half
					if (ddt > dt/2):
						ddt = dt/2
	
					dt -= ddt

					if debug:
						print('Decreasing stepsize to', dt)
	
				elif dist < minDist:
					# If we're too close, increase the stepsize

					# If we went from too big to too small, decrease the size of changes ddt
					if (lastTooBig):
						ddt /= 2

					lastTooSmall = True
					lastTooBig = False
					dt += ddt

					# Don't step over the end
					if t+dt >= et:
						dt = et - t
						lastPoint = True
						if debug:
							print('Instead generating last point at ', t+dt, 'using dt = ', dt)
					elif debug:
						print('Increasing stepsize to', dt)

				# Raise an exception if the stepsize diverges or converges to zero
				if dt > 1.0:
					raise Exception('Stepsize diverging! Stopping.')
				if dt <= 1e-15:
					raise Exception('Stepsize converging to zero! Stopping.')

				# With this new dt, compute another point and get its distance from previous
				(xt, yt) = getPoint(t+dt, i, R, Rr, E, N)
				dist = getDist(x, y, xt, yt)

				if debug:
					print('dist =', dist, 'for point at', (xt, yt))
	
			# Increment the angular offset by the angle increment with which the new point was computed
			t += dt

			# Store found point
			if debug:
				print('Adding point at', (xt, yt))
			(x, y) = (xt, yt)
			points.append((x,y))
			numPoints += 1

			if debug:
				print('t is now', t)

			# If the stepsize was reduced because we were too far,
			# We generate one more point
			#if t < et:
				#lastPoint = False

			if lastPoint:
				if debug:
					print('This was the last point, carrying on...')
				break
	
	# Add starting point again as last point to perfectly close the path
	points.append((xs,ys))
	
	return (points, numPoints)


# Generate cycloidal outer profile
def generate_outer_profile(design, debug=False):
	# Get design parameters
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
	
	# Parameters for point/curve generation of the cycloid disk
	maxDist = design.maxDist	# Maximum allowed distance between points
	minDist = design.minDist	# Minimum allowed distance between points	
	
	# Prepare list of points
	points_outer = []
	psi_1_list = []
	
	# Outer profile - get starting point
	(xs_outer, ys_outer, psi_1_prev) = getPoint_outer(0, 1, R, Rr, E, N, math.pi)
	points_outer.append((xs_outer,ys_outer))
	#print('(xs_outer, ys_outer) = (', xs_outer, ', ', ys_outer, ')')
	(x_outer, y_outer) = (xs_outer, ys_outer)
	
	# Compute ending point
	et = 2 * math.pi # Angular offset ending point
	
	# Initialise point generation
	dist = 0 # Distance to previous point
	dt = 2*math.pi / N / 100 # Initial step size
	numPoints_outer = 0
	
	# Generate outer profile
	# While we haven't reached the starting point again...
	# Also don't continue too for long (something went wrong in that case)
	i=1
	while (i < N or getDist(xs_outer, ys_outer, x_outer, y_outer) > maxDist) and i<(N+3):
		if debug:
			print('----')
			print('i =', i)

		# Starting angle
		t = 0 # TODO issues for t=0; programmatically avoid it?

		# Flag to make sure we generate a point *at* et (=2pi) each time
		lastPoint = False

		# While we have not reached the starting point...
		while (i < N or getDist(xs_outer, ys_outer, x_outer, y_outer) > maxDist):
			if debug:
				print('----')
				print('t =', round(t,4), ', dt =', dt)

			# If our current increment brings us beyond the end point,
			# we generate one more point and end for this value of i.
			if t+dt >= et:
				dt = et - t
				lastPoint = True
				if debug:
					print('Instead generating last point at ', t+dt, 'using dt = ', dt)

			# Get a new outer point with the existing stepsize
			(xt, yt, psi_1_prev_temp) = getPoint_outer(t+dt, i, R, Rr, E, N, psi_1_prev)

			# Compute the distance to the previous point
			dist = getDist(x_outer, y_outer, xt, yt)

			if debug:
				print('initial try before adjustment: dist =', dist, 'for point at', (xt, yt))

			# Set angle increment adjustment
			ddt = dt/2
			lastTooBig = False
			lastTooSmall = False

			# Find a good stepsize dt that puts the new point within a certain distance range
			# If it's the last point, we don't care about minimum distance
			while (not lastPoint and dist < minDist) or dist > maxDist:
				#print('Trying dt =', dt)
				if dist > maxDist:
					# If we're too far away, decrease the stepsize

					# If we went from too small to too big, decrease the size of changes ddt
					if (lastTooSmall):
						ddt /= 2
		
					lastTooSmall = False
					lastTooBig = True

					# Don't decrease by more than half
					if (ddt > dt/2):
						ddt = dt/2

					dt -= ddt

					if debug:
						print('Decreasing stepsize to', dt)

				elif dist < minDist:
					# If we're too close, increase the stepsize

					# If we went from too big to too small, decrease the size of changes ddt
					if (lastTooBig):
						ddt /= 2

					lastTooSmall = True
					lastTooBig = False
					dt += ddt

					# Don't step over the end
					if t+dt >= et:
						dt = et - t
						lastPoint = True
						if debug:
							print('Instead generating last point at ', t+dt, 'using dt = ', dt)
					elif debug:
						print('Increasing stepsize to', dt)

				# Raise an exception if the stepsize diverges or converges to zero
				if dt > 1.0:
					raise Exception('Stepsize diverging! Stopping.')
				if dt <= 1e-15:
					raise Exception('Stepsize converging to zero! Stopping.')

				# With this new dt, compute another point and get its distance from previous
				(xt, yt, psi_1_prev_temp) = getPoint_outer(t+dt, i, R, Rr, E, N, psi_1_prev)
				dist = getDist(x_outer, y_outer, xt, yt)

				if debug:
					print('dist =', dist, 'for point at', (xt, yt))

			if debug:
				print('Finally used: t =', t, ', dt =', dt, ' and t+dt =', t+dt)

			# Increment the angular offset by the angle increment with which the new point was computed
			t += dt

			# Store found point
			if debug:
				print('Adding point at', (xt, yt))
			(x_outer, y_outer) = (xt, yt)
			points_outer.append((x_outer,y_outer))
			psi_1_prev = psi_1_prev_temp
			psi_1_list.append(psi_1_prev)
			numPoints_outer += 1

			if debug:
				print('t is now', t)

			# If the stepsize was reduced because we were too far,
			# We generate one more point
			#if t < et:
				#lastPoint = False

			if lastPoint:
				if debug:
					print('This was the last point, carrying on...')
				break

		# Increment pin counter
		i += 1

	# Add starting point again as last point to perfectly close the path
	points_outer.append((xs_outer,ys_outer))
	
	return (points_outer, numPoints_outer, psi_1_list)
