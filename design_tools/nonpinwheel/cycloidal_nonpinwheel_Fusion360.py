# cycloidal_nonpinwheel_Fusion360.py

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
import adsk.core, adsk.fusion, adsk.cam, traceback
import math

# Cycloidal curve generator
from cycloidal_curves import *


###############################################################################
# Functions

# Using everything from cycloidal_curves.py


###############################################################################
# Run
def run(context):
	ui = None

	try:
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

		# Change max (and min) distance between generated points (larger than the default to reduce points)
		design.maxDist = 0.02 * design.Rr


		###############################################################################
		# Get interface
		app = adsk.core.Application.get()
		ui  = app.userInterface
		des = adsk.fusion.Design.cast(app.activeProduct)

		# Get root
		root = des.rootComponent


		###############################################################################
		# Prepare rotor part, sketch, and points list (ObjectCollection)

		# Get rotor and its component
		rotorOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
		rotor = rotorOcc.component
		rotor.name = 'rotor'

		# Add sketch
		sk = rotor.sketches.add(root.xYConstructionPlane)


		###############################################################################
		# Prepare housing part, sketch, and points list (ObjectCollection)

		# Get housing and its component
		housingOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
		housing = housingOcc.component
		housing.name = 'housing'

		# Add sketch
		sk_housing = housing.sketches.add(root.xYConstructionPlane)


		###############################################################################
		# Rotor and housing sketch points generation

		(curve, _) = generate_disk(design)
		(curve_outer, _, _) = generate_outer_profile(design)


		###############################################################################
		# Add each set of points as a line segment

		# Cycloidal disk
		lines = sk.sketchCurves.sketchLines
		for i in range(0, len(curve)-1):
			p0 = adsk.core.Point3D.create(curve[i][0]/10, curve[i][1]/10, 0)
			p1 = adsk.core.Point3D.create(curve[i+1][0]/10, curve[i+1][1]/10, 0)
			lines.addByTwoPoints(p0, p1)

		# Outer profile
		lines_outer = sk_housing.sketchCurves.sketchLines
		for i in range(0, len(curve_outer)-1):
			p0 = adsk.core.Point3D.create(curve_outer[i][0]/10, curve_outer[i][1]/10, 0)
			p1 = adsk.core.Point3D.create(curve_outer[i+1][0]/10, curve_outer[i+1][1]/10, 0)
			lines_outer.addByTwoPoints(p0, p1)

		return

	except:
		if ui:
			ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
