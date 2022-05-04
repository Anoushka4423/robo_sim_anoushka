import numpy as np


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
    
    
    print("xyk_k1:", xyk_k1)
    print("kk:", Kk)
    xkk = xyk_k1 + Kk @ yk
    I = np.identity(2)
    Pkk = (I - Kk @ Hk) @ Pk_k1
    # print("xkk", xkk)
    # print("Pkk", Pkk)
    
    return xkk, Pkk



particle = [0, 3, 0.4, [['34', 0, 2.5, [[0, 0],[0,0]]]]]
index_of_landmark = 0
dist = 0.5
angle = 2
Q = np.array([[0.1,0],[0,0.2]])
V = np.array([[0,0],[0,0]])

xkk, Pkk = ekf(particle, index_of_landmark, dist, angle, Q, V)