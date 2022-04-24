# -*- coding: utf-8 -*-
"""
Created on Sat Apr 23 15:33:14 2022

@author: 7450339
"""


import numpy as np

# Plan for particle filter. 
#
# We know the current position of the robot
#
# We want to find out the next time step position given the controls and 
# measurements
#
# We can potentially make a function to do this. 

# We know the following about the robot from simulator.py
from simulator.py import robot_length, robot_radius, robot_width 

    

def deriv(state, u, t=1, r=robot_radius, w=robot_width):
        x = state[0]
        y = state[1]
        theta = state[2]

        omega1 = u[0]
        omega2 = u[1]

        s = (omega1+omega2) * r / 2

        xdot = s*np.cos(theta)
        ydot = s*np.sin(theta)

        thetadot = r/w * (omega1-omega2)

        return [xdot, ydot, thetadot]