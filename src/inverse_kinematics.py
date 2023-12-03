# transform position error in velocity error in order to be able to use Fossen model

import numpy as np

def inverse_kinematics(current_position, error):
    Jone11 =  np.cos(current_position[5])* np.cos(current_position[4]) 
    Jone21 =  np.sin(current_position[5])* np.cos(current_position[4]) 
    Jone31 = - np.sin(current_position[4]) 
    Jone12 = - np.sin(current_position[5])* np.cos(current_position[3])+ np.cos(current_position[5])* np.sin(current_position[4])* np.sin(current_position[3]) 
    Jone22 =  np.cos(current_position[5])* np.cos(current_position[3])+ np.sin(current_position[3])* np.sin(current_position[4])* np.sin(current_position[5]) 
    Jone32 =  np.cos(current_position[4])* np.sin(current_position[3]) 
    Jone13 =  np.sin(current_position[5])* np.sin(current_position[3])+ np.cos(current_position[5])* np.cos(current_position[3])* np.sin(current_position[4]) 
    Jone23 = - np.cos(current_position[5])* np.sin(current_position[3])+ np.sin(current_position[4])* np.sin(current_position[5])* np.cos(current_position[3]) 
    Jone33 =  np.cos(current_position[4])* np.cos(current_position[3]) 

    Jtwo11 = 1
    Jtwo21 = 0
    Jtwo31 = 0
    Jtwo12 =  np.sin(current_position[3])* np.tan(current_position[4])
    Jtwo22 =  np.cos(current_position[3])
    Jtwo32 =  np.sin(current_position[3])/ np.cos(current_position[4])
    Jtwo13 =  np.cos(current_position[3])*np.tan(current_position[4])
    Jtwo23 = - np.sin(current_position[3])
    Jtwo33 =  np.cos(current_position[3])/ np.cos(current_position[4])

    Jn = np.array([ [Jone11, Jone12, Jone13, 0, 0, 0],
                    [Jone21, Jone22, Jone23, 0, 0, 0],
                    [Jone31, Jone32, Jone33, 0, 0, 0],
                    [0, 0, 0, Jtwo11, Jtwo12, Jtwo13],
                    [0, 0, 0, Jtwo21, Jtwo22, Jtwo23],
                    [0, 0, 0, Jtwo31, Jtwo32, Jtwo33] ])

    return Jn, np.matmul(Jn.T, error)