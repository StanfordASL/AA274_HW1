import numpy as np
from numpy import linalg
from P3_pose_stabilization import ctrl_pose

def ctrl_traj(x, y, th,
              ctrl_prev,
              x_d, y_d,
              xd_d, yd_d,
              xdd_d, ydd_d,
              x_g, y_g, th_g):
    '''
    This function computes the closed-loop control law.
    Inputs:
        (x,y,th): current state
        ctrl_prev: previous control input (V,om)
        (x_d, y_d): desired position
        (xd_d, yd_d): desired velocity
        (xdd_d, ydd_d): desired acceleration
        (x_g,y_g,th_g): desired final state
    Outputs:
        (V, om): a numpy array np.array([V, om]) containing the desired control inputs
    '''

    # Timestep
    dt = 0.005
    
    ########## Code starts here ##########




    ########## Code ends here ##########

    return np.array([V, om])