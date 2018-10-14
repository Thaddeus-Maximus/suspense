#!/usr/bin/env python

### Imports
import numpy as np
from scipy.linalg import solve
import csv


### Flags
pullrod = False			# whether this is a pull- or push- rod suspension
indirect_arm = False	# whether the pull- or push-rod attaches to the A-arm or the upright directly
has_axle = False		# whether the upright has an axle that takes the hub moment

# Initialize the members
members = {}
member_vectors = {}
member_points = {}
zrod = None
F = None

# Read through the CSV and parse out the different things
with open('front_007.tsv','r') as tsv:
	table = csv.reader(tsv, delimiter='\t')
	for row in table:
		if row[0] == 'config':
			if row[1] == 'pullrod':
				pullrod = True
			if row[2] == 'a_arm':
				indirect_arm = True
			if row[3] == 'has_axle':
				has_axle = True
		elif row[0] == 'wheel':
			wheel_center = np.array(row[1:4]).astype(np.float)
			wheel_radius = float(row[4])
		elif row[0] == 'wheel_load':
			F = np.array(row[1:4]).astype(np.float)
		elif indirect_arm and row[0][:2] == 'pu':
			zrod = np.array(row[1:]).astype(np.float)
		else:
			members[row[0]] = np.array(row[1:]).astype(np.float)

# Generate unit vectors for each suspension member
for k in members:
	member_points[k] = members[k][3:]
	member_vectors[k] = members[k][:3] - members[k][3:]
	member_vectors[k] = member_vectors[k]/np.linalg.norm(member_vectors[k])

# Generate the point at which the wheel touches the ground
contact_point = wheel_center - np.array([0.0,wheel_radius,0.0])
contact_point = contact_point.astype(np.float)

# Find the forces in each member
if indirect_arm:
	# solve A x = b
	# v = the relevant component of the unit vector associated with the force
	# First three rows: COLM: p*v + F = 0
	# Next three rows: COAM:  p* (r x v) + (r x F) = 0
	A = np.zeros((6,6))
	b = np.zeros((6,1))
	i = 0
	member_names = ['lower_a', 'lower_f', 'steer'] if pullrod else ['upper_a', 'upper_f', 'steer'] 
	for k in member_names:
		# For each member, set up the COLM equations
		A[0,i] = member_vectors[k][0]
		A[1,i] = member_vectors[k][1]
		A[2,i] = member_vectors[k][2]
		# Set up the COAM equations (cross products are taken row-wise)
		A[3:6,i] = np.cross(member_points[k], member_vectors[k])
		i+=1

	# Set up the load vector
	b[0:3] = F.reshape((3,1))
	b[3:6] = np.cross(wheel_center if has_axle else contact_point, F).reshape((3,1))

	# Set up the COLM equations for the fixed ball joint
	A[0,3] = -1
	A[1,4] = -1
	A[2,5] = -1

	# Set up the COAM equations for the fixed ball joint
	if pullrod:
		A[3:6,3] = -np.cross(member_points['upper_f'], np.array([1,0,0]))
		A[3:6,4] = -np.cross(member_points['upper_f'], np.array([0,1,0]))
		A[3:6,5] = -np.cross(member_points['upper_f'], np.array([0,0,1]))
	else:
		A[3:6,3] = -np.cross(member_points['lower_f'], np.array([1,0,0]))
		A[3:6,4] = -np.cross(member_points['lower_f'], np.array([0,1,0]))
		A[3:6,3] = -np.cross(member_points['lower_f'], np.array([0,0,1]))

	# Solve the equation
	x = solve(A, -b)
	i = 0
	for k in member_names:
		print(k, x[i][0])
		i+=1
	for k in ['Ball X', 'Ball Y', 'Ball Z']:
		print(k, x[i][0])
		i+=1
	print('Ball Magnitude', np.linalg.norm(x[3:6]))
	print('A Arm Magnitude', np.linalg.norm(member_vectors[member_names[0]]*x[0,0]+member_vectors[member_names[1]]*x[1,0]))
else:
	# solve A x = b
	# p*v + F = 0
	# p*v*r + F*r = 0
	A = np.zeros((6,6))
	b = np.zeros((6,1))
	i = 0
	for k in members:
		A[0,i] = member_vectors[k][0]
		A[1,i] = member_vectors[k][1]
		A[2,i] = member_vectors[k][2]
		

		A[3:6,i] = np.cross(member_points[k], member_vectors[k])
		print(contact_point, F)
		i+=1

	b[0:3] = F.reshape((3,1))
	b[3:6] = np.cross(wheel_center if has_axle else contact_point, F).reshape((3,1))

	x = solve(A, -b)
	i = 0
	for k in members:
		print(k, x[i][0])
		i+=1