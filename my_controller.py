# -*- coding: utf-8 -*-
"""
Created on Sat Apr 23 12:54:50 2022

@author: 7450339
"""

import zmq
import time
import json
import numpy as np
import random
import matplotlib.pyplot as plt
import copy 
import io
import pstats
from pstats import SortKey
import cProfile


def add_wheel_noise(omega1, omega2):
    sigma1 = np.sqrt(wheel_noise_a1*omega1**2+wheel_noise_a2*omega2**2)
    sigma2 = np.sqrt(wheel_noise_a3*omega1**2+wheel_noise_a4*omega2**2)
    omega1 += np.random.normal(0, sigma1)
    omega2 += np.random.normal(0, sigma2)
    return omega1, omega2


robot_width = 0.1
robot_length = 0.2
robot_radius = 0.05
timestep = 0.02

lidar_num_rays = 20
lidar_max_dist = 0.5
lidar_min_angle = -np.pi/2
lidar_max_angle = np.pi/2

wheel_noise_a1 = 0.01
wheel_noise_a2 = 0
wheel_noise_a3 = 0
wheel_noise_a4 = 0.01

landmark_range_sigma = 0.05
landmark_bearing_sigma = 0.01


# Connect with the zmq server
context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
pub_socket.connect("tcp://localhost:5557")

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("tcp://localhost:5555")

# Listen to the following topics
sub_socket.setsockopt(zmq.SUBSCRIBE, b"collision")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"lidar")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"landmarks")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"state")




s = 10
k = 0.0
r = 0.5
w = 1
k_dir = 1
count = 0

omega1 = 0.1
omega2 = 0.1



def get_position_of_landmark(robo_x, robo_y, robo_theta, landmark_dist, landmark_angle):
    # print("In get position of landmark: robo_x-", robo_x, "robo_y-", robo_y, 
    #       "robo_theta-", robo_theta, "landmark_dist-", landmark_dist, "landmark_angle-", landmark_angle)
    h = np.sin(landmark_angle)*landmark_dist
    l = np.cos(landmark_angle)*landmark_dist
    
    a = np.sin(robo_theta)*h
    b = np.cos(robo_theta)*h
    
    c = np.sin(robo_theta)*l
    d = np.cos(robo_theta)*l
    
    landmark_x = robo_x + (d-a)
    landmark_y = robo_y + (c+b)
    
    return landmark_x, landmark_y


def move_robot(x, y, theta, wheel_speed_1, wheel_speed_2, t=timestep):
    # print("in move_robot:", "orig_wheel_speeds", wheel_speed_1, wheel_speed_2)
    wheel1, wheel2 = add_wheel_noise(wheel_speed_1, wheel_speed_2)
    # print("after noise: wheel1", wheel1, wheel2)
    s = (wheel1+wheel2) * robot_radius / 2
    x_dot = s*np.cos(theta)
    y_dot = s*np.sin(theta)
    theta_dot = robot_radius/robot_width * (wheel1-wheel2)
    
    # print("x_dot", x_dot)
    # print("y_dot", y_dot)
    # print("theta_dot", theta_dot)
    
    new_x = x+x_dot*t
    new_y = y+y_dot*t
    new_theta = theta +theta_dot*t
    
    # print("new_x", new_x)
    # print("new_y", new_y)
    # print("new_theta", new_theta)
    
    return new_x, new_y, new_theta


def move_robots_time_steps(particles, wheel_speeds):
    
    for wheel_speed in wheel_speeds:
        wheel1 = wheel_speed[0]
        wheel2 = wheel_speed[1]
        # print("Wheel speeds:", wheel1, wheel2)
        for i, p in enumerate(particles):
            # print("index of particle", i)
            x, y, theta, landmarks = p
            # print("original", x, y, theta)
            x_update, y_update, theta_update = move_robot(x, y, theta, wheel1, wheel2)
            # print("updates", x_update, y_update, theta_update)
            # p[0] = x_update
            # p[1] = y_update
            # p[2] = theta_update
            particles[i] = [x_update, y_update, theta_update, landmarks]
            particle_past.append([x_update, y_update, theta_update])
    wheel_speeds = []
    

    return wheel_speeds, particles



def h_map_position_to_measurement(landmark_x, landmark_y, robo_x, robo_y, robo_theta):
    dist = np.sqrt((landmark_x-robo_x)**2 + (landmark_y - robo_y)**2)
    angle = np.arctan2((landmark_y - robo_y), (landmark_x - robo_x)) - robo_theta
    angle = np.remainder(angle+np.pi, 2*np.pi)-np.pi
    
    z = np.array([[dist],[angle]])
    return z

def landmark_position_to_dist_angle(landmark_x, landmark_y, robo_x, robo_y, robo_theta):
    # print("In landmark position to dist angle",
    #       "landmark_x", landmark_x, type(landmark_x),
    #       "landmark_y", landmark_y, type(landmark_y),
    #       "robo_x", robo_x, type(robo_x),
    #       "robo_y", robo_y, type(robo_y),
    #       "robo_theta", robo_theta, type(robo_theta))
    dist = np.sqrt((landmark_x-robo_x)**2 + (landmark_y - robo_y)**2)
    angle = np.arctan2((landmark_y - robo_y), (landmark_x - robo_x)) - robo_theta
    angle = np.remainder(angle+np.pi, 2*np.pi)-np.pi
    
    return dist, angle


# xk, yk => landmark_x, landmark_y and x, y => robo_x, robo_y
def H_jocobian_of_h_wrt_landmark_position(xk, yk, x, y):
    dist_deno = np.sqrt((xk-x)**2 + (yk - y)**2)
    theta_deno = xk**2 - 2*xk*x + yk**2 -2*yk*y+ x**2 +y**2
    
    partial_dist_partial_xk = (xk - x)/dist_deno
    partial_dist_partial_yk = (yk - y)/dist_deno
    
    partial_theta_partial_xk = (y - yk)/theta_deno
    partial_theta_partial_yk = (xk - x)/theta_deno
    
    H = np.array([[partial_dist_partial_xk, partial_dist_partial_yk], 
                   [partial_theta_partial_xk, partial_theta_partial_yk]])
    
    return H

def key_in_landmarks_1_particle(particle, key):
    all_landmarks = particle[3]
    for index, landmark in enumerate(all_landmarks):
        l_key = landmark[0]
        if(l_key == key):
            return True, index
    return False, -1

                
def ekf(particle, index_of_landmark, dist, angle, Q, V):
    
    robo_x, robo_y, robo_theta, all_landmarks = particle
    
    landmark_key_part, landmark_x, landmark_y, P = all_landmarks[index_of_landmark]

    xk1_k1 = landmark_x
    yk1_k1 = landmark_y
    
    # print("In ekf:", "P", P)
    # print("landmark_x, landmark_y", landmark_x, landmark_y)
    
    Pk1_k1 = np.array([[P[0][0], P[0][1]], [P[1][0], P[1][1]]])
    
    #Process Update
    xk_k1 = xk1_k1
    yk_k1 = yk1_k1
    
    Pk_k1 = Pk1_k1 + V
    
    # Posteriori Step's helper definition
    xyk_k1 = np.array([[xk_k1],[yk_k1]])
    
    h = h_map_position_to_measurement(xk_k1, yk_k1, robo_x, robo_y, robo_theta)
    # print("h", h, type(h))
    Hk = H_jocobian_of_h_wrt_landmark_position(xk_k1,yk_k1, robo_x, robo_y)
    zk = np.array([[dist],[angle]])
    # print("zk", zk)
    
    # kalman Step
    yk = zk - h
    Sk = Hk @ Pk_k1 @ Hk.T  + Q
    
    
    Kk = Pk_k1 @ Hk.T @ np.linalg.inv(Sk)
    
    
    # print("xyk_k1:", xyk_k1)
    # print("kk:", Kk)
    xkk = xyk_k1 + Kk @ yk
    I = np.identity(2)
    Pkk = (I - Kk @ Hk) @ Pk_k1
    # print("xkk", xkk)
    # print("Pkk", Pkk)
    
    return xkk, Pkk



def particles_badness_deprecated(particles, landmark_measurements):
    
    badness = []
    badness_dist = []
    badness_angle = []
    # print("In particles badness: particles", particles)
    # print("landmark_measurements:", landmark_measurements)
    
    
    for particle in particles:
        # print("Particle:", particle)
        robo_x, robo_y, robo_theta, landmarks = particle
        particle_dist_error = 0
        particle_angle_error = 0
        for key in landmark_measurements:
            # print("landmark:", landmark_measurements[key])
            key_exist, index = key_in_landmarks_1_particle(particle, key)
            particle_landmark = landmarks[index]
            particle_landmark_dist, particle_landmark_angle = landmark_position_to_dist_angle(particle_landmark[1], 
                                                               particle_landmark[2], robo_x, robo_y, robo_theta)
            true_landmark_dist = landmark_measurements[key]['dist']
            true_landmark_angle = landmark_measurements[key]['theta']
            
            particle_dist_error += abs(particle_landmark_dist - true_landmark_dist)
            particle_angle_error += abs(particle_landmark_angle - true_landmark_angle)
            
            # print("error:", particle_dist_error+particle_angle_error)
        badness_dist.append(particle_dist_error)
        badness_angle.append(particle_angle_error)
        
        badness.append(particle_dist_error+particle_angle_error)
    
    # print("in Particles badness: ", badness)
    # print("Badness_dist", badness_dist)
    # print("Badness_angle", badness_angle)
    return badness


def get_priot_list_deprecated(badness, size=100):
    badness_index = np.argsort(badness)[::-1][:len(badness)]
    badness = np.sort(badness)[::-1]   
    if(len(badness) == 0):
        print("BADNESS IS EMPTY.")
    product = np.product(badness)

    A = np.where(~(np.isfinite(product/badness)), 0.1, product/badness)
    maxx = np.max(A)
    A_new = A/maxx
        
    semi_final = []
    for i in range(size):
        r = random.uniform(0.0, 1.0)
    
        for index in range(len(A_new)):
            if(r < A_new[index]):
                semi_final.append(index)
                break
    
    
    final = badness_index[semi_final]
    return final

#             #0   #1  #2   #3    #4   #5    #6
# badness = [0.1, 0.2, 0.5, 0.4, 0.8, 0.09, 0.8]

# badness= [0.1686977333162936,    #0
#           0.1611077528269935,    #1
#           0.09320913116965049,   #2
#           0.14174663630024206,   #3
#           0.09330533778222028,   #4
#           0.07501415202391568,   #5
#           0.034329908157696265,  #6
#           0.020346492842330532,  #7
#           0.08906310996308409,   #8
#           0.0005700852176953597] #9
# print(get_priot_list(badness, 20))


def particles_resample_deprecated(particles, landmark_measurements):
    # if(len(particles) == 0):
    #     print("PARTICLES WAS EMPTY!")
    #     print("Landmark_measurements:", landmark_measurements)
    badness = particles_badness_deprecated(particles, landmark_measurements)
    # print("badness:", badness)
    
    priot_index = get_priot_list_deprecated(badness, 10)
    
    if(len(priot_index) == 0):
        print("Particles:", particles)
        print("Landmark_measurements:", landmark_measurements)
        print("Badness:", badness)
        
    # print("In particles resample, badness:", badness)
    # print("priot_index:", priot_index, len(priot_index))
    new_particles = []
    # priortized_list = badness[priot_index]
    for i in range(len(particles)):
        index = random.choice(priot_index)
        new_particles.append(particles[index])
    
    return new_particles



def return_goodness(particles, landmark_measurements):
    goodness = []
    
    for particle in particles:
        robo_x, robo_y, robo_theta, landmarks = particle
        particle_dist_error = 0
        particle_angle_error = 0
        for key in landmark_measurements:
            key_exist, index = key_in_landmarks_1_particle(particle, key)
            particle_landmark = landmarks[index]
            particle_landmark_dist, particle_landmark_angle = landmark_position_to_dist_angle(particle_landmark[1], 
                                                               particle_landmark[2], robo_x, robo_y, robo_theta)
            true_landmark_dist = landmark_measurements[key]['dist']
            true_landmark_angle = landmark_measurements[key]['theta']
            
            particle_dist_error += abs(particle_landmark_dist - true_landmark_dist)
            particle_angle_error += abs(particle_landmark_angle - true_landmark_angle)
            
        badness = particle_dist_error+particle_angle_error
        goodness.append(1/badness)
    
    return goodness


def priority_index(goodness):
    goodness_sum = np.sum(goodness)
        
    priot_list = []
    # running_sum = 0
    
    for good in goodness:
        # running_sum  += good/goodness_sum
        priot_list.append(good/goodness_sum)
        
    return priot_list

# goodness = [42.87136997,77.42917811,  30.82083669, 29.74582979, 21.67092421, 18.72368001, 17.75926573, 16.06789043, 10.06745649, 9.599174951]
# priot_list = priority_index(goodness)
# print("Priot_list", priot_list)




def particles_resample(particles, landmark_measurements, num_particles=10):
    new_particles = []
    goodness = return_goodness(particles, landmark_measurements)
    priot_list = priority_index(goodness)
    
    # print("Priot_list", priot_list)
    particle_indexes = np.arange(0, num_particles)
    new_index = np.random.choice(particle_indexes, num_particles, p=priot_list)

    for i in new_index:
        new_particles.append(particles[i])
        
    return new_particles



def get_landmarks_x_and_y(landmarks):
    # print("landmarks:",landmarks)
    # print("landmarks[4]", landmarks[4])
    landmark_xs = []
    landmark_ys = []
    
    for landmark in landmarks:
        # print("1 landmark:", landmark)
        [landmark_key, landmark_x, landmark_y, P] = landmark
        landmark_xs.append(landmark_x)
        landmark_ys.append(landmark_y)
    
    return landmark_xs, landmark_ys
    

# def plot_all_landmarks_for_all_particles_with()
    
    
    
    # These two functions are taken from StackOverflow
def eigsorted(cov):
    '''
    Eigenvalues and eigenvectors of the covariance matrix.
    '''
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:, order]


def cov_ellipse(points, cov, nstd):
    """
    Source: http://stackoverflow.com/a/12321306/1391441
    """

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)

    return width, height, theta

def plot_all_particles(particles):
    for p in particles:
        robo_x, robo_y, robo_theta, particle_landmarks = p
        plt.plot([robo_x], [robo_y], '.')
    
    
def plot_particles_past(particle_past, true_state_past, all_particles_all_landmarks, num_of_particles = 10):
    # print(len(all_particles_all_landmarks))
    # print(len(true_state_past))
    first_true_state = true_state_past[0]
    for i in range(0, len(particle_past), num_of_particles):
        
        for j in range(num_of_particles):
            part_past = particle_past[i+j]
            
            # print("should plot:", part_past[0], ",", part_past[1])
            plt.plot([part_past[0]], [part_past[1]], '.', color='blue')
            plt.plot([part_past[0]+first_true_state[0]], [part_past[1]+first_true_state[1]], '.')
        
        state_past = true_state_past[int(i/num_of_particles)]
        plt.plot([state_past[0]], [state_past[1]], '.', color='black')
        
        
    for i in range(num_of_particles):
        landmarks = all_particles_all_landmarks[i]
        l_xs, l_ys = get_landmarks_x_and_y(landmarks)
        plt.plot(l_xs, l_ys, '*', color='green')
    


true_state_past = []
all_particles_all_landmarks = []


# A particle is [robo_x, robo_y, robo_theta, [landmark_key, landmark_x, landmark_y, [[p1, p2],[p3, p4]]]]
num_of_particles = 50
particles = []
particle_past = []
for i in range(num_of_particles):
    p = [0.0, 0.0, 0.0, []]
    # particle_past.append([p[0],p[1],p[2]])
    particles.append(p)
    all_particles_all_landmarks.append([])


#counter 
counter = 0

# Record previous lidar messages
lidar_wheel_speeds= []


# Landmark Noise
Q =  np.array([[landmark_range_sigma**2, 0], [0, landmark_bearing_sigma**2]])
# Q =  sym.Matrix([[0, 0], [0, 0]])

# Process Noise -- Will have to tune, somehow
V = np.array([[0, 0],[0,0]])

topic, message = sub_socket.recv_multipart()
while(topic != b'state'):
    topic, message = sub_socket.recv_multipart()
    

pr = cProfile.Profile()
pr.enable()
# Infinite loop
while(counter < 1000):
    # print("STARTING of the LOOP!!")
    counter += 1
    landmark_handled = False
    # print("Landmark_handled:", landmark_handled)

    lidar_mess = []
    landmark_mess = []
    collision_mess = []
    state_mess = []
    
# Get the message from ZMQ
    for i in range(4):
        topic, message = sub_socket.recv_multipart()
        
        if(topic == b'lidar'):
            # lidar_mess = message
            lidar_mess = json.loads(message.decode())
            
        elif(topic == b'landmarks'):
            landmark_mess = json.loads(message.decode())["landmarks"]
            
        elif(topic == b'collision'):
            collision_mess = json.loads(message.decode())
            
        elif(topic == b'state'):
            state_mess = json.loads(message.decode())
            true_state_past.append([state_mess['x'], state_mess['y']])
            
    # Handle the landmarks if any:
    for landmark_key in landmark_mess:
        # print("landmark_mess:", landmark_mess)
        landmark_handled = True
        # print("Landmark handles was made True")
        
        # lidar_wheel_speeds, particles = move_robots_time_steps(particles, lidar_wheel_speeds)
        
        landmark_dist = landmark_mess[landmark_key]['dist']
        landmark_angle = landmark_mess[landmark_key]['theta']
        
        for p_i, ptcls in enumerate(particles):
            # print("In the particle loop")
            [robo_x, robo_y, robo_theta, all_landmarks] = ptcls
            
            key_in_landmarks, index_of_landmark = key_in_landmarks_1_particle(ptcls, landmark_key)
            
            if(not(key_in_landmarks)):
                landmark_x, landmark_y = get_position_of_landmark(robo_x, robo_y, robo_theta, landmark_dist, landmark_angle)
                initial_P = [[2., 0.], [0., 2.]]
                all_landmarks.append([landmark_key, landmark_x, landmark_y, initial_P])
                particles[p_i][3] = all_landmarks
                
            else:
                xkk, Pkk = ekf(ptcls, index_of_landmark, landmark_dist, landmark_angle, Q, V)
                
                new_landmark = [landmark_key, xkk[0][0], xkk[1][0], Pkk]
                all_landmarks[index_of_landmark] = new_landmark
                particles[p_i][3] = all_landmarks
            
            all_particles_all_landmarks[p_i] = all_landmarks
            
            
    # Then, use this new found location of the landmark to resample the 50 robots. 
    # print("Landmark_handled:", landmark_handled)
    if(landmark_handled):
        # if(len(particles) == 0):
            # print("particles is empty!")
        particles = particles_resample(particles, landmark_mess, num_of_particles)
        landmark_handled = False
        
    

    # After handling the landmarks, we can now handle the lidar information. 
    # Update the wheel speeds and send them to the simulator. 
    # Also, keep track of the new wheel speeds by adding them to the wheel speed
    # queue. 
    
    # lidar_message_dict = json.loads(lidar_dict.decode())
    lidar_dists = lidar_mess["distances"]
        
    if (len(lidar_dists) > 0):
        # print("lidar_mess:", lidar_mess)
        max_dist = 0.5
        angle_interval = np.pi/19
        
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
        # omega1 = 0.5
        # omega2 = 0.5

        wheel_speeds = {"omega1": omega1, "omega2": omega2}
        pub_socket.send_multipart(
            [b"wheel_speeds", json.dumps(wheel_speeds).encode()])
        
        lidar_wheel_speeds.append((omega1, omega2))
        lidar_wheel_speeds, particles = move_robots_time_steps(particles, lidar_wheel_speeds)


pr.disable()
s = io.StringIO()
sortby = SortKey.CUMULATIVE
ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
ps.print_stats()
print(s.getvalue())

# print(particle_past)  
# print(true_state_past)  
plt.figure()
plt.xlim(-5, 4)
plt.ylim(-5, 3)

plot_particles_past(particle_past, true_state_past, all_particles_all_landmarks, num_of_particles)
# plt.draw()
# plt.pause(0.1)

plt.show()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    