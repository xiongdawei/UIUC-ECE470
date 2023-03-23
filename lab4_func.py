#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.identity(4)
	S = np.zeros((6,6))
	S1 = np.array([0,0,1, 150, 150, 0]).T
	S2 = np.array([0,1,0, -162, 0, -150]).T
	S3 = np.array([0,1,0, -162, 0, 94]).T
	S4 = np.array([0,1,0, -162, 0, 307]).T
	S5 = np.array([1,0,0,0,162,-260 ]).T
	S6 = np.array([0,1,0, -162, 0, 390]).T
	S = [S1,S2,S3,S4,S5,S6]
	M[0][3] = 150
	M[1][3] = -150
	M[2][3] = -10





	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()
	S1 = ScrewToBracket(S[0])
	S2 = ScrewToBracket(S[1])
	S3 = ScrewToBracket(S[2])
	S4 = ScrewToBracket(S[3])
	S5 = ScrewToBracket(S[4])
	S6 = ScrewToBracket(S[5])
	T = expm(S1*theta1)@expm(S2*theta2)@expm(S3*theta3)@expm(S4*theta4)@expm(S5*theta5)@expm(S6*theta6)@M

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	yaw = yaw_WgripDegree * np.pi/180
	xB = xWgrip + 150
	yB = yWgrip - 150
	zB = zWgrip - 10
	L = 53.5
	L5_y = xB - L * np.cos(yaw)
	L5_x = yB - L * np.sin(yaw)
	alpha = np.arctan2([L5_y], [L5_x])
	theta1 = alpha - np.arctan2([110], [457])
	theta6 = np.pi/2 + theta1 - yaw
	theta5 = np.pi/2
	temp_X = xB - L - 213
	temp_Z = zB + 59 + 83
	theta2_p2 = np.arctan2([temp_Z - 152],[temp_X])
	LL = ((temp_Z - 152)**2 + temp_X**2)**0.5
	theta2_p1 = -np.arccos([(244**2 + LL**2 - 213**2)/2])
	theta2 = theta2_p1 + theta2_p2
	theta3 = np.pi - np.arccos([(244**2 + 213**2 - LL**2)/2])
	theta4 = -theta2 - theta3






	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

def ScrewToBracket(S):
	S = S.T
	r = S[:3]
	v = S[3:]
	r_skew = np.array([[0, -r[2], r[1], v[0]],[r[2], 0, -r[0], v[1]], [-r[1],r[0],0, v[2]]])
	print(r_skew)
	res = np.concatenate((r_skew,np.array([[0,0,0,0]])), axis = 0)
	return res