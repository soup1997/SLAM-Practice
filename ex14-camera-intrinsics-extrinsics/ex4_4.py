import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt


def E_from_point_pairs(xs, xss, K):
    """Compute Essential Matrix from points pairs."

    Input:
        xs  - [N x 3] points in first image
        xss - [N x 3] points in second image
        K   - [3 x 3] calibration matrix
    Output:
        E  - [3 x 3] essential matrix of camera pair
    """
    A = np.zeros((xs.shape[0],9))
    # compute direction of ray in camera coordinate system",

    # compute svd",

    # compute essential Matrix\n",

    # apply conditions to E\n",

    # final essential matrix\n",
    E = None
    
    return E


def relative_orientation_from_E(E, Z, W):
    """Compute the rotation and direction given E.",

    Input:
        E - [3 x 3] essential matrix of camera pair
        Z - [3 x 3] Solution by Hartley & Zisserman
        W - [3 x 3] Solution by Hartley & Zisserman
    Output:
        R  - [3 x 3] rotation matrix
        Sb - [3 x 3] skew sym matrix which represents direction of 2nd cam
    """
  
    # compute svd
    
    # compute rotation matrix",

    # compute skew-sym matrix\n",
  
    return R, Sb


def compute_intersection(X0, r, X1, s):
    """ Compute the intersection of rays of two cameras

    Input:
        X0 - [3 x 1] position of 1st camera
        r  - [3 x 1] direction vector of 1st cam
        X1 - [3 x 1] position of 2nd camera
        s  - [3 x 1] direction vector of 2nd cam
    Output:
        X - [3 x 1] point of intersection
    """
    r = r / np.linalg.norm(r)
    s = s / np.linalg.norm(s)

    # derived from geometrical constraints\n",
    A = None

    b = None
    
    x = None

    # X0 - point in r where the lines should intersect
    # X1 - point in s where the lines should intersect
    F = None
    H = None
    
    # we need a mean point if two lines do not intersect in 3D
    X = None

    return X


def triangulate_points(xs, xss, K, R01, X1):
    """Triangulate a batch of rays

    Input:
        xs  - [N x 3] points in first image
        xss - [N x 3] points in second image
        K   - [3 x 3] calibration matrix
        R12 - [3 x 3] rotation of 2nd cam w.r.t. 1st cam
        X1  - [3 x 1] projection center of 2nd cam
      
    Output:
        X - [N x 3] 3D points in global frame
    """

    # compute direction in corresponding camera frame

    # rotate rays of 2nd cam s.t. they are represented in the coord system of 1st cam,

    # intersect points,
    X = None
    return X


def point_in_front_of_cam(X):
    """Check whether or not points lie in front of the camera

    Inputs:
        X - [3 x N] Points in camera coordinate system
    """
    check = None

    return np.all(check)


def gen_homogeneous_pts(pts):
    homo_pts = np.ones((pts.shape[0], pts.shape[1] + 1))
    homo_pts[:, :pts.shape[1]] = pts
    return homo_pts


if __name__ == '__main__':
    pass