import numpy as np

def control_allocation(tau):
    # Thrust configuration matrix
    T_thrust = np.array([[0.707, 0.707, -0.707, -0.707, 0, 0],
                         [-0.707, 0.707, -0.707, 0.707, 0, 0],
                         [0, 0, 0, 0, 1, 1],
                         [0.06, -0.06, 0.06, -0.06, 0.111, -0.111],
                         [0.06, 0.06, -0.06, -0.06, 0, 0],
                         [-0.1888, 0.1888, 0.1888, -0.1888, 0, 0] ])
    
    K_thrust = np.diag([40, 40, 40, 40, 40, 40])
    K_thrust_inv = np.linalg.pinv(K_thrust)
    
    T_pseudo_inv = np.linalg.pinv(T_thrust)
    control_input = np.matmul( K_thrust_inv, np.matmul(T_pseudo_inv, tau) )

    return control_input