import numpy as np

def RPY2Rot(angles):
    """
    RPY angles to Rotation Matrix
    Input: angles (phi, theta, psi) in radians
    Output: bRi (Rotation matrix from body to inertial frame)
    """
    phi = angles[0]
    theta = angles[1]
    psi = angles[2]
    
    # Z-axis rotation (psi)
    R_3 = np.array([
        [np.cos(psi), np.sin(psi), 0],
        [-np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    
    # Y-axis rotation (theta)
    R_2 = np.array([
        [np.cos(theta), 0, -np.sin(theta)],
        [0, 1, 0],
        [np.sin(theta), 0, np.cos(theta)]
    ])
    
    # X-axis rotation (phi)
    R_1 = np.array([
        [1, 0, 0],
        [0, np.cos(phi), np.sin(phi)],
        [0, -np.sin(phi), np.cos(phi)]
    ])
    
    # bRi = R_1 * R_2 * R_3 (Z-Y-X rotation sequence)
    # Note: MATLAB code used R_1 * R_2 * R_3, so we follow that.
    # This corresponds to a specific Euler angle convention.
    bRi = R_1 @ R_2 @ R_3
    return bRi