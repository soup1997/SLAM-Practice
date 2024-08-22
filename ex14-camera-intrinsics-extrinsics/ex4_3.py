# Compute camera extrinsics using P3P (spatial resection)
import numpy as np  

def spatial_resection(x, X, K):
    """ Implements the P3P algorithm to estimate the extrinsics given
    4 3D-2D point correspondences """
    
    # 1. Compute angles and distances to the 3D points
    
        
    # 2. Identify the correct solution using the 4th point
    
    
    # 3. Compute the coordinate transformation to obtain 𝑅 and 𝑋0
    
    
    R = np.eye(3, dtype = np.float)
    X0 = np.zeros((3, 1), dtype=np.float)
    
    return R, X0