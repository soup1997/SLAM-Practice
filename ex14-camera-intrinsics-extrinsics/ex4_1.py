# Implementation of the DLT method to compute the camera projection matrix 

import numpy as np
import matplotlib.pyplot as plt


def decompose_P(P):
    # Decompose Camera projection matrix P into intrinsic (c, s, m , x_0, y_0)
    # and extrinsic  parameters (R, X0).
    # Input:  Camera projection matrix (P:= 3x4)
    # Output: Intrinsics (K:= 3x3)
    #         Origin of camera in world frame (X0:= 3x1)
    #         Orientation of the camera in the world frame (R:=3x3)
    # Notes:
    # Camera projection equation
    # X (3D point in world) -> x (2d position in image)
    # x = KR[I|-X0]X

    [K, R, X0] = [None, None, None]
    
    return [K, R, X0]
    
    
def dlt(x, X):
    # 1. Build linear system Mp = 0
    
    # 2. Perform SVD M = USV'
    
    # 3. Extract solution
    
    P = None
    return P

