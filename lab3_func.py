#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0, -1, 0, 390],[0, 0, -1, 401],[1,0,0,215.5],[0,0,0,1]])
	S1 = np.array([0,0,1, 150, 150, 0]).T
	S2 = np.array([0,1,0, -162, 0, -150]).T
	S3 = np.array([0,1,0, -162, 0, 94]).T
	S4 = np.array([0,1,0, -162, 0, 307]).T
	S5 = np.array([1,0,0,0,162,-260 ]).T
	S6 = np.array([0,1,0, -162, 0, 390]).T
	S = [S1,S2,S3,S4,S5,S6]
	




	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
	S1 = ScrewToBracket(S[0])
	S2 = ScrewToBracket(S[1])
	S3 = ScrewToBracket(S[2])
	S4 = ScrewToBracket(S[3])
	S5 = ScrewToBracket(S[4])
	S6 = ScrewToBracket(S[5])
	T = expm(S1*theta1)@expm(S2*theta2)@expm(S3*theta3)@expm(S4*theta4)@expm(S5*theta5)@expm(S6*theta6)@M
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

def ScrewToBracket(S):
	S = S.T
	r = S[:3]
	v = S[3:]
	r_skew = np.array([[0, -r[2], r[1], v[0]],[r[2], 0, -r[0], v[1]], [-r[1],r[0],0, v[2]]])
	print(r_skew)
	res = np.concatenate((r_skew,np.array([[0,0,0,0]])), axis = 0)
	return res