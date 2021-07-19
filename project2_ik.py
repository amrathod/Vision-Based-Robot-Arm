import numpy as np
import math
import sympy as sp

def inverse_k2(position):
    #position in the world frame!
    x_old = position[0]
    z = position[1]

    # after rotating the base, the diagonal of the triangle on the  x-z plane is the new x
    x = np.sqrt(x_old**2 + z**2)

    y=0 # height where the end effector should be in the robot frame

    # setting lengths of links
    l1= 125
    l2= 125
    l3= 135
    aa = -45
    
    # decomposing arm to planar 2 link and planar 1 link

    x_prime = x - l3*np.cos(aa)
    
    y_prime = y - l3*np.sin(aa)
    
    c2=(x_prime**2 +y_prime**2 - l1**2 -l2**2)/(2*l1*l2)
    
    theta2=-np.arccos(c2)* 180 / np.pi
    
    alpha= np.arctan2(y_prime, x_prime) * 180 / np.pi
    
    beta= np.arccos((x_prime**2+y_prime**2+l1**2-l2**2)/(2*l1*np.sqrt(x_prime**2+y_prime**2)))*180/np.pi

    theta1 = alpha + beta
    
    theta3 = aa-theta1-theta2 
    
    angles = [theta1-20, theta2 + 106, theta3 + 90] # adjusted for proper alignment of robot arm joints
    
    return angles    
