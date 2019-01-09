import numpy as np
from utils import wrapToPi

def ctrl_pose(x, y, th, xg, yg, thg):
    '''
    This function implements the pose stabilization controller.
    Inputs:
        x, y, th: the current pose of the robot
        xg, yg, thg: the desired pose of the robot
    Outputs:
        ctrl: a numpy array np.array([V, om]) containing the desired control inputs
    HINT: you need to use the wrapToPi function
    HINT: don't forget to saturate your control inputs
    '''

    ########## Code starts here ##########



    ########## Code ends here ##########

    return np.array(ctrl)