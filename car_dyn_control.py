import numpy as np
from scipy import linalg

def car_dyn(x, t, ctrl, noise):

    u_0 = ctrl[0] + noise[0]
    u_1 = ctrl[1] + noise[1]

    dxdt = [u_0*np.cos(x[2]), u_0*np.sin(x[2]), u_1]

    return dxdt

def ctrl_traj(x,y,th,dyn_state,ctrl_prev,x_d,y_d,xd_d,yd_d,xdd_d,ydd_d,x_g,y_g,th_g):
    #(x,y,th): current state
    #dyn_state: compensator internal dynamic state
    #ctrl_prev: previous control input (V,om)
    #(xd_d, yd_d): desired Velocity
    #(xdd_d, ydd_d): desired acceleration
    #(x_g,y_g,th_g): desired final state

    # Timestep
    dt = 0.005

    # Gains
    kpx =
    kpy =
    kdx =
    kdy =

    #Code trajectory controller (Switch to pose controller once "close" enough)
    #...FILL...#

    #Define control inputs (V,om) - without saturation constraints

    #Define accel = dV/dt

    #Integrate dynamic state with anti-windup
    if (V > 0.5 or om > 1.0) and (accel > 0.0): #integration will only make things worse
        if (V > 0.5): dyn_state_up = 0.5 #cap-off integrator at max
        else: dyn_state_up = dyn_state #or just freeze integration
    else:
        dyn_state_up = dyn_state + accel*dt

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om, dyn_state_up])

def wrapToPi(a):
    b = a
    for i in range(len(a)):
        if a[i] < -np.pi or a[i] > np.pi:
            b[i] = ((a[i]+np.pi) % (2*np.pi)) - np.pi
    return b

def ctrl_pose (x,y,th,x_g,y_g,th_g):
    #(x,y,th): current state
    #(x_g,y_g,th-g): desired final state

    #Code pose controller
    #...FILL...#

    #Define control inputs (V,om) - without saturation constraints

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om])
