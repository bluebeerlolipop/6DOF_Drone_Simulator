import numpy as np

class Control_Position:
    def __init__(self, gains, droneParams, dt):
        self.g = 9.81
        self.dt = dt
        self.m = droneParams['mass']

        self.x_err = 0.0
        self.x_err_prev = 0.0
        self.x_err_sum = 0.0

        self.y_err = 0.0
        self.y_err_prev = 0.0
        self.y_err_sum = 0.0

        self.z_err = 0.0
        self.z_err_prev = 0.0
        self.z_err_sum = 0.0

        self.kP_x = gains['P_x']
        self.kI_x = gains['I_x']
        self.kD_x = gains['D_x']

        self.kP_y = gains['P_y']
        self.kI_y = gains['I_y']
        self.kD_y = gains['D_y']

        self.kP_z = gains['P_z']
        self.kI_z = gains['I_z']
        self.kD_z = gains['D_z']

    def PositionCtrl(self, currentState, refSig):

        current_r = currentState[0:3]

        x_des = refSig[0]
        y_des = refSig[1]
        z_des = refSig[2]
        psi_cmd = refSig[3]

        self.x_err = x_des - current_r[0]
        self.y_err = y_des - current_r[1]
        self.z_err = z_des - current_r[2]

        u_thrust = self.m * self.g - (self.kP_z * self.z_err +
                                      self.kI_z * self.z_err_sum +
                                      self.kD_z * (self.z_err - self.z_err_prev)/self.dt)
        
        self.z_err_sum  += self.z_err * self.dt
        self.z_err_prev = self.z_err

        theta_cmd =  - (self.kP_x * self.x_err +
                     self.kI_x * self.x_err_sum +
                     self.kD_x * (self.x_err - self.x_err_prev)/self.dt)
        
        self.x_err_sum += self.x_err * self.dt
        self.x_err_prev = self.x_err

        phi_cmd = (self.kP_y * self.y_err +
                     self.kI_y * self.y_err_sum +
                     self.kD_y * (self.y_err - self.y_err_prev)/self.dt)
        
        self.y_err_sum += self.y_err * self.dt
        self.y_err_prev = self.y_err
        

        u = u_thrust
        cmd = np.array([phi_cmd, theta_cmd, psi_cmd])

        return u, cmd