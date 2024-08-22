# Compute camera intrinsics (K) using Zhang's method using a known checkerboard pattern.
import numpy as np
import matplotlib.pyplot as plt

def compute_homography(x, X):
    """ Computes homography H (3x3) given 2D-3D correspondences """
    # Build linear system

    
    # Perform SVD


    # Extract solution
    
    H = None
    return H

def compute_V(H):
    """ Computes the constraint matrix (2x6) given the homography for each image """
    V = None
    return V

def compute_K(V):
    """ Extracts K from the constraint matrix V """
    #Solve for Vb = 0
    
    # Decompose B to obtain K
 
    # Rotate to have a negative camera constant c
 
    K = None
    return K

