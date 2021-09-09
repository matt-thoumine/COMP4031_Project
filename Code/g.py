import numpy as np
import math

course_start = np.array([1000,500,0])
g = 9.81
rho = 1.225 #kg/m^3 (Air Density)

def mag(A):
    # Return magnitude of vector
    # ||A||
    return np.linalg.norm(A)

def normalize(A):
    # Return the unit vector
    # A/||A||
    if np.linalg.norm(A) != 0:
        return A/np.linalg.norm(A)
    else:
        return np.array([0,0,0])

def limit(A,maximum):
    # Limits vector to maximum value if greater than find unit vector and multiply
    # (A/||A||)*max
    if np.linalg.norm(A) > maximum:
        return normalize(A)*maximum
    else:
        return A

def angleBetween(A,B):
    # Return the angle between 2 vectors
    # inverse cos ((A.B)/(||A||*||B||))
    if np.linalg.norm(A) == 0 or np.linalg.norm(B) == 0:
        return 0
    else:
        return np.arccos((np.dot(A,B))/(np.linalg.norm(A)*np.linalg.norm(B)))


def distance(A,B):
    # Euclidean Distance between 2 vectors as points in 3D
    distX = A[0]-B[0]
    distY = A[1]-B[1]
    distZ = A[2]-B[2]
    return math.sqrt((distX*distX)+(distY*distY)+(distZ*distZ))

def getNormalPoint(P,A,B):
    # Returns Normal Point on line A->B from Point
    AP = P - A #Vector from A->P
    AB = B - A #Vector from A->B (along line)
    
    if np.linalg.norm(AB) != 0:
        AB = AB/np.linalg.norm(AB)
    else:
        AB = 0
    
    return A + (AB * np.dot(AP,AB))

