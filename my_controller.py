# -*- coding: utf-8 -*-
"""
Created on Sat Apr 23 12:54:50 2022

@author: 7450339
"""

import zmq
import time
import json
import numpy as np


# Connect with the zmq server
context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
pub_socket.connect("tcp://localhost:5557")

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("tcp://localhost:5555")

# Listen to the following topics
sub_socket.setsockopt(zmq.SUBSCRIBE, b"state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"collision")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"lidar")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"landmarks")

# When the simulation starts, wherever the robot is at, 
# is (0, 0, 0) for (x, y, theta). 
# x = 0
# y = 0
# theta = 0

s = 10
k = 0.0
r = 0.5
w = 1
k_dir = 1

omega1 = 0.1
omega2 = 0.1

lidar_queue = []
landmark_queue = []
collision_queue = []


while True:

    topic, message = sub_socket.recv_multipart()
    
    if( topic == b"landmark"):
        landmark_queue.append(message)
    
    elif(topic == b"lidar"):
        lidar_queue.append(message)
    
    elif (topic == b"collision"):
        collision_queue.append(message)
        
    
    while(len(landmark_queue) > 0):
        # Handle landmark stuff
    
    
        # In the end empty the landmark queue
        landmark_queue = []
        
    
    if (len(lidar_queue) > 0):
        max_dist = 0.5
        angle_interval = np.pi/19
        
        
        #use the latest information
        message = lidar_queue[-1]
        
        message_dict = json.loads(message.decode())
        lidar_dists = message_dict["distances"]

        min_index = np.argmin(lidar_dists)
        min_angle = -np.pi/2 + min_index*angle_interval
        min_dist = lidar_dists[min_index]

        if min_dist < 0.8*max_dist:
            k = -10*np.copysign(np.exp(-np.abs(min_angle)), min_angle)
            if not np.isfinite(k):
                k = 0
        else:
            k = 0

        omega1 = (s+w*k)/(2*r)
        omega2 = (s-w*k)/(2*r)

        wheel_speeds = {"omega1": omega1, "omega2": omega2}
        pub_socket.send_multipart(
            [b"wheel_speeds", json.dumps(wheel_speeds).encode()])
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    