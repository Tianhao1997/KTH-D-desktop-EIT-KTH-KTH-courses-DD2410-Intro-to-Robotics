#! /usr/bin/env python3

import math
from math import acos, asin, sqrt, atan2, sin, cos, pi
import numpy as np
from numpy import linalg
from numpy.linalg import inv

"""
    # {Tianhao He}
    # {tianhaoh@kth.se}
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    
    # scara details
    L0 = 0.07
    L1 = 0.3  
    L2 = 0.35
   
    x = x - L0

    cos2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

    q2 = math.atan2(math.sqrt(1 - cos2**2), ((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)))

    q1 = math.atan2(y, x) - math.atan2((L2 * math.sin(q2)), (L1 + (L2 * math.cos(q2))))

    q3 = z 
    
    q = [q1, q2, q3]

    return q


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    # kuka details
    L = 0.4
    M = 0.39
    PI = math.pi
    counter = 0
    R = np.array(R)
    while True:

        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        q4 = q[3]
        q5 = q[4]
        q6 = q[5]
        q7 = q[6]
        # compute jacobian 
        jacobian = np.matrix([[L * sin(q1) * sin(q2) - M * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2)),
            -cos(q1) * (M * (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4)) + L * cos(q2)), 
            -M * sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)), 
            M * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))* (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4)) + M * sin(q2) * sin(q3) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2)), 0, 0, 0], 
            [ -M * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * cos(q4) * sin(q2)) - L * cos(q1) * sin(q2), -1 * sin(q1) * (M * (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4)) + L * cos(q2)),
            M * sin(q4) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)), 
            M * (sin(q1) * sin(q2) * sin(q4) + cos(q1) * cos(q4) * sin(q3) + cos(q2) * cos(q3) * cos(q4) * sin(q1)), 0, 0, 0], 
            [0, M * cos(q2) * cos(q3) * sin(q4) - M * cos(q4) * sin(q2) - L * sin(q2), -M * sin(q2) * sin(q3) * sin(q4), 
            M * cos(q3) * cos(q4) * sin(q2) - M * cos(q2) * sin(q4), 0, 0, 0], 
            [0, sin(q1), -1 * cos(q1) * sin(q2), -1 * cos(q3) * sin(q1) - cos(q1) * cos(q2) * sin(q3),
            -sin(q4) * (sin(q1) * sin(q3) -1 * cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2),
            cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) - sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * sin(q2) * sin(q4)),
            sin(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) - cos(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * cos(q4) * sin(q2))], 
            [0, -1 * cos(q1), -1 * sin(q1) * sin(q2), cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3), sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2), 
            sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)), 
            cos(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2)) - sin(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1)*cos(q3) - cos(q2) * sin(q1) * sin(q3)))], 
            [1, 0, cos(q2), -1 * sin(q2) * sin(q3), cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4), cos(q5) * sin(q2) * sin(q3) - sin(q5) * (cos(q2) * sin(q4) - cos(q3) * cos(q4) * sin(q2)),
            sin(q6) * (cos(q5) * (cos(q2) * sin(q4) - cos(q3) * cos(q4) * sin(q2)) + sin(q2) * sin(q3)*sin(q5)) + cos(q6) * (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4))]])
        
            
        # compute transformed base to end effecter 
        transform_to_ee = np.array([[sin(q7) * (sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) - cos(q7) * (sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * cos(q4) * sin(q2)) + cos(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)))),  
        sin(q7) * (sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * cos(q4) * sin(q2)) + cos(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)))) + cos(q7) * (sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2)*sin(q3))), 
        sin(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) - cos(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * cos(q4) * sin(q2)),
        -1 * M * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * cos(q4) * sin(q2)) - L * cos(q1) * sin(q2)], 
        [cos(q7) * (sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2)) + cos(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)))) - sin(q7) * (sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))), 
        -1 * sin(q7) * (sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2)) + cos(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)))) - cos(q7) * (sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))), 
        cos(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2)) - sin(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))),
        M * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * sin(q1) * sin(q2)) - L * sin(q1) * sin(q2)], 
        [sin(q7) * (sin(q5) * (cos(q2) * sin(q4) - cos(q3) * cos(q4) * sin(q2)) - cos(q5) * sin(q2) * sin(q3)) - cos(q7) * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) - cos(q3) * cos(q4) * sin(q2)) + sin(q2) * sin(q3) * sin(q5)) - sin(q6) * (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4))),
        cos(q7) * (sin(q5) * (cos(q2) * sin(q4) - cos(q3) * cos(q4)*sin(q2)) - cos(q5) * sin(q2) * sin(q3)) + sin(q7) * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) - cos(q3) * cos(q4) * sin(q2)) + sin(q2) * sin(q3) * sin(q5)) - sin(q6) * (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4))),
        sin(q6) * (cos(q5) * (cos(q2) * sin(q4) - cos(q3) * cos(q4) * sin(q2)) + sin(q2) *sin(q3) * sin(q5)) + cos(q6) * (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4)),
        M * (cos(q2) * cos(q4) + cos(q3) * sin(q2) * sin(q4)) + L * cos(q2)], [0, 0, 0, 1]])
        
        
        ee = transform_to_ee.dot([[0], [0], [0.078], [1]])
        ee[2] = 0.311 + ee[2]
        #R = np.array(R)
        angular_error = -0.2 * (np.cross(transform_to_ee[:3, 0], R[:, 0]) + np.cross(transform_to_ee[:3, 1], R[:, 1]) + np.cross(transform_to_ee[:3, 2], R[:, 2]))
        error_cord = np.array([ee[0]-x, ee[1]-y, ee[2]-z, angular_error[0], angular_error[1], angular_error[2]])
        theta = (jacobian.T * inv(jacobian * jacobian.T)).dot(np.transpose(error_cord))
        error_norm = linalg.norm(error_cord)
        for i in np.arange(7):
            q[i] = q[i] - float(theta[0, i])
        counter = counter + 1 
        if  error_norm < 0.04 or counter > 5:
            break

    return q

