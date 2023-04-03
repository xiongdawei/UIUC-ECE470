#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
import math
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
	S1 = np.array([0,0,1, 150/1000, 150/1000, 0]).T
	S2 = np.array([0,1,0, -162/1000, 0, -150/1000]).T
	S3 = np.array([0,1,0, -162/1000, 0, 94/1000]).T
	S4 = np.array([0,1,0, -162/1000, 0, 307/1000]).T
	S5 = np.array([1,0,0,0,162/1000,-260/1000]).T
	S6 = np.array([0,1,0, -162/1000, 0, 390/1000]).T
	S = [S1,S2,S3,S4,S5,S6]
	# M[0][3] = 150
	# M[1][3] = -150
	# M[2][3] = -10
	M[0][3] = 390/1000
	M[1][3] = 401/1000
	M[2][3] = 215.5/1000





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
	L1 = 152/1000
	L2 = 120/1000
	L3 = 244/1000
	L4 = 93/1000
	L5 = 213/1000
	L6 = 83/1000
	L7 = 83/1000
	L8 = 82/1000
	L = 53.5/1000
	yaw = yaw_WgripDegree * np.pi/180
	x = xWgrip + 150/1000
	y = yWgrip - 150/1000
	z = zWgrip - 10/1000
	
	x_cen = x - L * np.cos(yaw)
	y_cen = y - L * np.sin(yaw)
	alpha = np.arctan2([y_cen], [x_cen])
	line = math.sqrt(x_cen**2 + y_cen**2)
	alpha_prime = np.arcsin((L2-L4+L6)/line)
	theta1 = alpha - alpha_prime
	theta6 = np.pi/2 + theta1 - yaw
	theta5 = -np.pi/2

	line_prime = line*np.cos(alpha_prime) - L7
	x_3end = line_prime*np.sin(theta1)
	y_3end = line_prime*np.cos(theta1)
	z_3end = z + L + L8

	theta2_p2 = np.arctan2([z_3end - L1],[math.sqrt(x_3end**2 + y_3end**2)])
	LL = ((z_3end - L1)**2 + x_3end**2 + y_3end**2)**0.5

	temp = LL**2 + (L3)**2 - (L5)**2

	theta2_p1 = np.arccos([((temp/(2*LL*L3)))])
	theta2 = theta2_p1 + theta2_p2
	theta2 = -theta2
	temp = L3**2 + L5**2 - LL**2

	theta3 = np.pi - np.arccos([temp/(2*L3*L5)])
	theta4 = -theta2 - theta3
	

	theta1 = theta1 * 180/np.pi
	theta2 = theta2 * 180/np.pi
	theta3 = theta3 * 180/np.pi
	theta4 = theta4 * 180/np.pi
	theta5 = theta5 * 180/np.pi
	theta6 = theta6 * 180/np.pi

	print("Theta1:", theta1)
	print("Theta6:", theta6)
	print("Theta5:", theta5)
	print("Theta2:", theta2)
	print("Theta3:", theta3)
	print("Theta4:", theta4)

	theta1 = theta1 * np.pi/180
	theta2 = theta2 * np.pi/180
	theta3 = theta3 * np.pi/180
	theta4 = theta4 * np.pi/180
	theta5 = theta5 * np.pi/180
	theta6 = theta6 * np.pi/180


	# theta1 = 0.0
	# theta2 = 0.0
	# theta3 = 0.0
	# theta4 = 0.0
	# theta5 = 0.0
	# theta6 = 0.0
	# theta1 = -0.055
	# theta2 = -0.441
	# theta3 = 2.539
	# theta4 = -2.098
	# theta5 = -pi/2
	# theta6 = -0.3168
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