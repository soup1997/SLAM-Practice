# add your fancy code here
import math


def inv_motion_model(u_t):
    '''
    Takes as input an odometry reading u_t that consists x_(t-1) to x_t
    The Output is relative motion rot1, trans, rot2
    '''
    x_prev = u_t[0]
    x_curr = u_t[1]

    trans = math.sqrt((x_curr[0] - x_prev[0])**2 + (x_curr[1] - x_prev[1]))
    rot1 = math.atan2(x_curr[1] - x_prev[1], x_curr[0] - x_prev[0]) - x_prev[2]
    rot2 = (x_curr[2] - x_prev[2]) - rot1

    return (rot1, trans, rot2)


def gaussian_pdf(x, mu, sigma):
    coefficient = 1 / (sigma * math.sqrt(2 * math.pi))
    exponent = -((x - mu) ** 2) / (2 * sigma ** 2)
    return coefficient * math.exp(exponent)


def motion_model_odometry(init_pose, final_pose, u_t, alpha):
    prob = []
    rot1, trans, rot2 = inv_motion_model(u_t)
    
    for noise in alpha:
        p_rot1 = gaussian_pdf(rot1, 0, noise)
        p_trans = gaussian_pdf(trans, 0, noise)
        p_rot2 = gaussian_pdf(rot2, 0, noise)
        prob.append([p_rot1, p_trans, p_rot2])
        

