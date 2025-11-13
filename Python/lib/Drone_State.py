import numpy as np
from RPY2Rot import RPY2Rot

class Drone_State:
    def __init__(self, params, initStates, simTime, dt):
        self.g = 9.81
        self.t = 0.0
        self.dt = dt
        self.tf = simTime

        self.m = params['mass']
        self.l = params['armLength']
        self.I = np.array([
            [params['Ixx'], 0, 0],
            [0, params['Iyy'], 0],
            [0, 0, params['Izz']]
        ])

        self.x = initStates
        self.r = self.x[0:3]
        self.dr = self.x[3:6]
        self.euler = self.x[6:9]
        self.w = self.x[9:12]

        self.dx = np.zeros(12)
        self.T = 0.0
        self.M = np.zeros(3)

    def GetState(self):
        return self.x
    
    def EvalEOM(self):
        bRi = RPY2Rot(self.euler)
        R = bRi.T
        
        self.dx[0:3] = self.dr
        self.dx[3:6] = (1/self.m) * (R @ np.array([0, 0, -self.T]) + np.array([0, 0, self.m * self.g]))

        phi, theta, _ = self.euler
        E = np.array([
            [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
        ])
        self.dx[6:9] = E @ self.w

        self.dx[9:12] = np.linalg.inv(self.I) @ (self.M - np.cross(self.w, self.I @ self.w))

    def UpdateState(self, u):
        self.T = u[0]
        self.M = u[1:4]

        self.t = self.t + self.dt

        self.EvalEOM()
        self.x = self.x + self.dx * self.dt

        self.r = self.x[0:3]
        self.dr = self.x[3:6]
        self.euler = self.x[6:9]
        self.w = self.x[9:12]