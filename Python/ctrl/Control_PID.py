import numpy as np

class Contrl_PID:
    def __init__(self, gains, dt):
        self.dt = dt

        self.phi_err = 0.0
        self.phi_err_prev = 0.0
        self.phi_err_sum = 0.0

        self.theta_err = 0.0
        spsi = 0.0
        self.theta_err_sum  = 0.0

        self.psi_err = 0.0
        self.psi_err_prev = 0.0
        self.psi_err_sum  = 0.0

        self.kP_phi = gains['P_phi']
        self.kI_phi = gains['I_phi']
        self.kD_phi = gains['D_phi']

        self.kP_theta = gains['P_theta']
        self.kI_theta = gains['I_theta']
        self.kD_theta = gains['D_theta']

        self.kP_psi = gains['P_psi']
        self.kI_psi = gains['I_psi']
        self.kD_psi = gains['D_psi']

    def AttitudeCtrl(self, currentState, refSig):

        current_euler = currentState[6:9]

        phi_des = refSig[0]
        theta_des = refSig[1]
        psi_des = refSig[2]

        self.phi_err = phi_des - current_euler[0]
        self.theta_err = theta_des - current_euler[1]
        self.psi_err = psi_des - current_euler[2]

        u = np.zeros(3)
        u[0] = (self.kP_phi * self.phi_err +
                self.kI_phi * self.phi_err_sum +
                self.kD_phi * ((self.phi_err - self.phi_err_prev) / self.dt))
        
        self.phi_err_sum += self.phi_err * self.dt
        self.phi_err_prev = self.phi_err

        u[1] = (self.kP_theta * self.theta_err +
                self.kI_theta * self.theta_err_sum +
                self.kD_theta * ((self.theta_err - self.theta_err_prev) / self.dt))
        
        self.theta_err_sum += self.theta_err * self.dt
        self.theta_err_prev = self.theta_err

        u[2] = (self.kP_psi * self.psi_err +
                self.kI_psi * self.psi_err_sum +
                self.kD_psi * ((self.psi_err - self.psi_err_prev) / self.dt))
        
        self.psi_err_sum += self.psi_err * self.dt
        self.psi_err_prev = self.psi_err