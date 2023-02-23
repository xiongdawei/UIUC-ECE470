#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q11 = [145.45*pi/180, -54.18*pi/180, 120.78*pi/180, -162.75*pi/180, -89.84*pi/180, (-67.46)*pi/180] # Left Tower Bottom)
Q12 = [144.54*pi/180, -61.88*pi/180, 120.63*pi/180, -154.91*pi/180, -89.67*pi/180, (-68.44)*pi/180] # Left Tower Middle
Q13 = [145.66*pi/180, -69.75*pi/180, 118.22*pi/180, -144.35*pi/180, -89.62*pi/180, (-68.51)*pi/180] # Left Tower Top
T1 = [145.66*pi/180, -73.77*pi/180, 116.29*pi/180, -138.67*pi/180, -89.62*pi/180, (-68.59)*pi/180]  # Left Tower High Point
Q21 = [165.32*pi/180, -58.08*pi/180, 128.27*pi/180, -164.43*pi/180, -94.21*pi/180, (-25.25)*pi/180] # Middle Tower Bottom
Q22 = [166.36*pi/180, -63.79*pi/180, 122.63*pi/180, -154.42*pi/180, -91.97*pi/180, (-46.73)*pi/180] # Middle Tower Middle
Q23 = [165.82*pi/180, -71.24*pi/180, 121.80*pi/180, -146.01*pi/180, -91.09*pi/180, (-47.65)*pi/180] # Middle Tower Top
T2 = [165.85*pi/180, -75.94*pi/180, 119.49*pi/180, -139.00*pi/180, -91.05*pi/180, (-47.71)*pi/180]  # Middle Tower High Point
Q31 = [189.23*pi/180, -51.26*pi/180, 110.83*pi/180, -153.69*pi/180, -93.93*pi/180, (-23.90)*pi/180] # Right Tower Bottom
Q32 = [189.24*pi/180, -57.71*pi/180, 110.07*pi/180, -146.49*pi/180, -93.89*pi/180, (-23.97)*pi/180] # Right Tower Middle
Q33 = [189.26*pi/180, -63.80*pi/180, 108.07*pi/180, -138.41*pi/180, -93.84*pi/180, (-24.05)*pi/180] # Right Tower Top
T3 = [189.26*pi/180, -68.46*pi/180, 107.85*pi/180, -133.69*pi/180, -93.05*pi/180, (-24.39)*pi/180]  # Right Tower High Point

# Initialize Q Matrix
Q = [[Q11, Q12, Q13, T1],
     [Q21, Q22, Q23, T2],
     [Q31, Q32, Q33, T3]]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):
    global digital_in_0

    digital_in_0 = msg.DIGIN

    global analog_in_0
    global analog_in_1

    analog_in_0 = msg.AIN0
    analog_in_1 = msg.AIN1




############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height,
               end_loc, end_height):
    move_arm(pub_cmd, loop_rate, Q[start_loc][3], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.5)
    if(digital_in_0 == 0):
        gripper(pub_cmd, loop_rate, suction_off)
        error = 1
    else:
        move_arm(pub_cmd, loop_rate, Q[start_loc][3], 4.0,4.0)
        move_arm(pub_cmd, loop_rate, Q[end_loc][3], 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(0.5)
        move_arm(pub_cmd, loop_rate, Q[end_loc][3], 4.0, 4.0)
        error = 0

    return error


############### Your Code End Here ###############


# def TowerOfHanoi(n, start_loc, end_loc, aux_rod, pub_cmd, loop_rate, height):
#     if n == 0:
#         return
#     TowerOfHanoi(n-1, start_loc, aux_rod, end_loc, pub_cmd, loop_rate, height)
#     move_block(pub_cmd, loop_rate, start_loc, height[start_loc], end_loc, height[end_loc])
#     print("Move disk", n, "from rod", start_loc, "to rod", end_loc)
#     height[start_loc] = height[start_loc] - 1
#     height[end_loc] = height[end_loc] + 1
#     TowerOfHanoi(n-1, aux_rod, start_loc, end_loc, pub_cmd, loop_rate, height)


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 1
    start_loc = 0
    end_loc = 0
    height = [0,0,0]

    while(not input_done):
        # input_string = input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
        input_string = input("Enter the start and end location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")
        res = input_string.split(" ")
        if (len(res) != 2):
            print("Please type both the start and end location")
            continue
        start_loc = int(res[0])
        end_loc = int(res[1])
        input_done = 1
        if (start_loc == end_loc) or int(res[0]) == 0 or int(res[1]) == 0:
            print("Quitting... ")
            sys.exit()
   
        
        # if(int(input_string) == 1):
        #     input_done = 1
        #     loop_count = 1
        # elif (int(input_string) == 2):
        #     input_done = 1
        #     loop_count = 2
        # elif (int(input_string) == 3):
        #     input_done = 1
        #     loop_count = 3
        # elif (int(input_string) == 0):

        # else:
        #     print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    pos = [1, 2, 3]
    pos.remove(start_loc)
    pos.remove(end_loc)
    middle_loc = pos[0]
    print("Middle Location is:" + str(middle_loc))
    # aux_rod = pos[0]
    # print("aux is:" + str(aux_rod))



    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # while(loop_count > 0):
    #     height[start_loc] = 2
    #     TowerOfHanoi(3, start_loc, end_loc, aux_rod, pub_command, loop_rate, height)

    middle_loc = 0
    if((start_loc == 1 and end_loc == 3) or (start_loc == 3 and end_loc == 1)) :
        middle_loc = 2
    if((start_loc == 1 and end_loc == 2) or (start_loc == 2 and end_loc == 1)) :
        middle_loc = 3
    if((start_loc == 2 and end_loc == 3) or (start_loc == 3 and end_loc == 2)) :
        middle_loc = 1
    block_error = 0
    gripper(pub_command, loop_rate, suction_off)
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    rospy.loginfo("Sending goal 1 ...")
    block_error = move_block(pub_command, loop_rate, start_loc - 1, 2, end_loc - 1, 0)
    if(block_error == 1):
        print("Oops, there's something wrong with the block")
        sys.exit()
    block_error = move_block(pub_command, loop_rate, start_loc - 1, 1, middle_loc - 1, 0)
    if(block_error == 1):
        print("Oops, there's something wrong with the block")
        sys.exit()
    block_error = move_block(pub_command, loop_rate, end_loc - 1, 0, middle_loc - 1, 1)
    if(block_error == 1):
        print("Oops, there's something wrong with the block")
        sys.exit()
    block_error = move_block(pub_command, loop_rate, start_loc - 1, 0, end_loc - 1, 0)
    if(block_error == 1):
        print("Oops, there's something wrong with the block")
        sys.exit()
    block_error = move_block(pub_command, loop_rate, middle_loc - 1, 1, start_loc - 1, 0)
    if(block_error == 1):
        print("Oops, there's something wrong with the block")
        sys.exit()
    block_error = move_block(pub_command, loop_rate, middle_loc - 1, 0, end_loc - 1, 1)
    if(block_error == 1):
        print("Oops, there's something wrong with the block")
        sys.exit()
    block_error = move_block(pub_command, loop_rate, start_loc - 1, 0, end_loc - 1, 2)
    if(block_error == 1):
        print("Oops, there's something wrong with the block")
        sys.exit()
    print("Congrats! It works!")
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    sys.exit()


    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

        # loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
