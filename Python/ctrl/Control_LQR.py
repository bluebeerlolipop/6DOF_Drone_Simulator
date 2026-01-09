import numpy as np
import control as ct
from stateMatrix import stateMatrix

class Control_LQR:
    def __init__(self, q, r, droneParams, refSig):
        self.Q = np.diag(q)
        self.R = np.diag(r)

        self.m = droneParams['mass']
        self.g = 9.81
        self.u0 = np.array([self.m*self.g, 0, 0, 0])

        self.A, self.B = stateMatrix(droneParams, refSig)
        self.K, self.S, self.E = ct.lqr(self.A, self.B, self.Q, self.R)

    def AttitudeCtrl(self, currentState, refSig):

        x_des = refSig[0]
        y_des = refSig[1]
        z_des = refSig[2]
        psi_des = refSig[3]

        ref_vector = np.array([
            x_des, y_des,   z_des,
                0,     0,       0,
                0,     0, psi_des,
                0,     0,       0
        ])

        current_x = currentState
        error = current_x - ref_vector

        u_fb = -self.K @ error
        u = self.u0 + u_fb

        return u