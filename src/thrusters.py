import numpy as np

def thrusters(control_input):

    T_thrust = np.array([[0.707, 0.707, -0.707, -0.707, 0, 0],
                         [-0.707, 0.707, -0.707, 0.707, 0, 0],
                         [0, 0, 0, 0, 1, 1],
                         [0.06, -0.06, 0.06, -0.06, 0.111, -0.111],
                         [0.06, 0.06, -0.06, -0.06, 0, 0],
                         [-0.1888, 0.1888, 0.1888, -0.1888, 0, 0]  ])
    
    K_thrust = np.diag([40, 40, 40, 40, 40, 40])

    tau = np.matmul( T_thrust, np.matmul(K_thrust, control_input) )
    return tau