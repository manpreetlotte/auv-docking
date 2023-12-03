import numpy as np

class Chassis:
    def __init__(self) -> None:
        # moment of inertias (MoI) wrt principal axes
        self.Ix = 0.16 
        self.Iy = 0.16 
        self.Iz = 0.16 
        self.I = [self.Ix, self.Iy, self.Iz]

class Body:
    def __init__(self) -> None:
        
        self.g = 9.81
        self.ro = 1000
        self.delta = 0.0117
        self.m = 11.5
        # center of mass (CoM) position
        self.xg = 0  # CoM x-pos in self frame
        self.yg = 0  # CoM y-pos in self frame
        self.zg = 0.02  # CoM z-pos in self frame

        # weight of the self
        self.W = self.m*self.g
        # buoyancy force
        self.B = self.ro*self.g*self.delta

        # added mass parameters
        self.X_udot = -5.5 
        self.Y_vdot = -12.7 
        self.Z_wdot = -14.57 
        self.K_pdot = -0.12 
        self.M_qdot = -0.12 
        self.N_rdot = -0.12 

        # linear damping coefficients
        self.Xu = -4.03 
        self.Yv = -6.22 
        self.Zw = -5.18 
        self.Kp = -0.07 
        self.Mq = -0.07 
        self.Nr = -0.07 

        # non-linear damping coefficients
        self.Xu_mod = -18.18 
        self.Yv_mod = -21.66 
        self.Zw_mod = -36.99 
        self.Kp_mod = -1.55 
        self.Mq_mod = -1.55 
        self.Nr_mod = -1.55

        self.Chassis = Chassis()

        # Constant damping matrix
        self.damping_matrix = - np.diag( [self.Xu, self.Yv, self.Zw, self.Kp, self.Mq, self.Nr] )

        # Inertia matrix
        Mrb = np.array([ [self.m, 0, 0, 0, self.m*self.zg, 0],
                         [0, self.m, 0, -self.m*self.zg, 0, 0],
                         [0, 0, self.m, 0, 0, 0],
                         [0, -self.m*self.zg, 0, self.Chassis.Ix, 0, 0],
                         [self.m*self.zg, 0, 0, 0, self.Chassis.Iy, 0],
                         [0, 0, 0, 0, 0, self.Chassis.Iz] ])
  
        Ma = - np.diag( [self.X_udot, self.Y_vdot, self.Z_wdot, self.K_pdot, self.M_qdot, self.N_rdot] )

        self.inertia_matrix = Mrb + Ma

def Ca_matrix(vref, Body):
        Ca = np.array([ [0, 0, 0, 0, Body.Z_wdot*vref[2], 0], 
                        [0, 0, 0, - Body.Z_wdot*vref[2], 0, -  Body.X_udot*vref[0]], 
                        [0, 0, 0, - Body.Y_vdot*vref[1],  Body.X_udot*vref[0], 0], 
                        [0, - Body.Z_wdot*vref[2],  Body.Y_vdot*vref[1], 0, - Body.N_rdot*vref[5],  Body.M_qdot*vref[4]],
                        [ Body.Z_wdot*vref[2], 0, - Body.X_udot*vref[0],  Body.N_rdot*vref[5], 0, - Body.K_pdot*vref[3]],
                        [- Body.Y_vdot*vref[1],  Body.X_udot*vref[0], 0, - Body.M_qdot*vref[4],  Body.K_pdot*vref[3], 0] ])
        
        return Ca

def Crb_matrix(current_velocity, Body, Chassis):
        Crb = np.array([ [0, 0, 0, 0,  Body.m*current_velocity[2], 0], 
                         [0, 0, 0, - Body.m*current_velocity[2], 0, 0], 
                         [0, 0, 0,  Body.m*current_velocity[1], - Body.m*current_velocity[0], 0], 
                         [0,  Body.m*current_velocity[2], - Body.m*current_velocity[1], 0,  Chassis.Iz*current_velocity[5],  - Chassis.Iy*current_velocity[4]],
                         [- Body.m*current_velocity[2], 0, - Body.m*current_velocity[0], - Chassis.Iz*current_velocity[5], 0,  Chassis.Ix*current_velocity[3]],
                         [ Body.m*current_velocity[1], - Body.m*current_velocity[0], 0,  Chassis.Iy*current_velocity[4], - Chassis.Ix*current_velocity[3], 0] ])
        
        return Crb

def Dn(current_velocity, Body):
    D11 =  Body.Xu_mod * abs(current_velocity[0])
    D22 =  Body.Yv_mod * abs(current_velocity[1])
    D33 =  Body.Zw_mod * abs(current_velocity[2])
    D44 =  Body.Kp_mod * abs(current_velocity[3])
    D55 =  Body.Mq_mod * abs(current_velocity[4])
    D66 =  Body.Nr_mod * abs(current_velocity[5])

    D = [D11, D22, D33, D44, D55, D66]
    return np.diag(D)

def Gn(current_position, Body):
    g1 = ( Body.W - Body.B)*np.sin(current_position[4])
    g2 = -( Body.W - Body.B)*np.cos(current_position[4])*np.sin(current_position[3])
    g3 = -( Body.W - Body.B)*np.cos(current_position[4])*np.cos(current_position[3])
    g4 = Body.zg*Body.W*np.cos(current_position[4])*np.sin(current_position[3])
    g5 = Body.zg*Body.W*np.sin(current_position[4])
    g6 = 0

    return np.array([g1, g2, g3, g4, g5, g6])