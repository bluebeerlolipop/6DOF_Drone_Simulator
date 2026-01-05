import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def plot_simulation_results(refSig):
    # load data
    R2D = 180.0 / np.pi
    D2R = np.pi / 180.0
    dt = 0.01
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_file = os.path.join(script_dir, 'stateHistory.npy')

    stateHistory = np.load(data_file)
    
    x_des = refSig[0]   # [m]
    y_des = refSig[1] # [m]
    z_des = refSig[2]   # [m]
    psi_des = refSig[3]  # [deg]

    # define time
    numPoints = stateHistory.shape[0]
    t = np.linspace(0, (numPoints - 1) * dt, numPoints)

    # command signals
    x_cmd = np.ones(numPoints) * x_des
    y_cmd = np.ones(numPoints) * y_des
    z_cmd = np.ones(numPoints) * z_des
    psi_cmd = np.ones(numPoints) * psi_des

    print(f"Complete loading. {numPoints} steps, {t[-1]:.2f}sec data.")

    # Plotting data
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Drone State(Roll, Pitch, Yaw, Z-dot)', fontsize=16, fontweight='bold')

    # Plot 1: Roll (phi)
    axs[0, 0].plot(t, stateHistory[:, 6] * R2D, 'r', linewidth=1.5, label=r'Roll ($\phi$)')
    axs[0, 0].grid(True)
    axs[0, 0].set_title(r'Roll ($\phi$) History')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Angle (deg)')
    axs[0, 0].legend(loc='lower right')

    # Plot 2: Pitch (theta)
    axs[0, 1].plot(t, stateHistory[:, 7] * R2D, 'r', linewidth=1.5, label=r'Pitch ($\theta$)')
    axs[0, 1].grid(True)
    axs[0, 1].set_title(r'Pitch ($\theta$) History')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Angle (deg)')
    axs[0, 1].legend(loc='lower right')

    # Plot 3: Yaw (psi)
    axs[1, 0].plot(t, stateHistory[:, 8] * R2D, 'r', linewidth=1.5, label=r'Yaw ($\psi$)')
    axs[1, 0].grid(True)
    axs[1, 0].set_title(r'Yaw ($\psi$) History')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Angle (deg)')
    axs[1, 0].legend(loc='lower right')

    # Plot 4: Z-dot (Vertical Speed)
    axs[1, 1].plot(t, stateHistory[:, 5], 'r', linewidth=1.5, label='Z-dot')
    axs[1, 1].grid(True)
    axs[1, 1].set_title('Vertical Speed (Z-dot) History')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Speed (m/s)')
    axs[1, 1].legend(loc='lower right')

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    save_filename = 'drone_state_results(1).png'
    plt.savefig(save_filename)
    

    fig1, axs1 = plt.subplots(2, 2, figsize=(12, 8))
    fig1.suptitle(r'Drone State(X, Y, Z,$\psi$))', fontsize=16, fontweight='bold')

    # Plot 1: X
    axs1[0, 0].plot(t, stateHistory[:, 0], 'r', linewidth=1.5, label='X')
    axs1[0, 0].plot(t, x_cmd, 'b--', linewidth=1.5, label='Command')
    axs1[0, 0].grid(True)
    axs1[0, 0].set_title('X History')
    axs1[0, 0].set_xlabel('Time (s)')
    axs1[0, 0].set_ylabel('Position (m)')
    axs1[0, 0].legend(loc='lower right')

    # Plot 2: Y
    axs1[0, 1].plot(t, stateHistory[:, 1], 'r', linewidth=1.5, label='Y')
    axs1[0, 1].plot(t, y_cmd, 'b--', linewidth=1.5, label='Command')
    axs1[0, 1].grid(True)
    axs1[0, 1].set_title('Y History')
    axs1[0, 1].set_xlabel('Time (s)')
    axs1[0, 1].set_ylabel('Position (m)')
    axs1[0, 1].legend(loc='lower right')

    # Plot 3: Z
    axs1[1, 0].plot(t, stateHistory[:, 2], 'r', linewidth=1.5, label='Z')
    axs1[1, 0].plot(t, z_cmd, 'b--', linewidth=1.5, label='Command')
    axs1[1, 0].grid(True)
    axs1[1, 0].set_title('Z History')
    axs1[1, 0].set_xlabel('Time (s)')
    axs1[1, 0].set_ylabel('Postion (m))')
    axs1[1, 0].legend(loc='lower right')

    # Plot 4: psi
    axs1[1, 1].plot(t, stateHistory[:, 8]*R2D, 'r', linewidth=1.5, label=r'($\psi$)')
    axs1[1, 1].plot(t, psi_cmd, 'b--', linewidth=1.5, label='Command')
    axs1[1, 1].grid(True)
    axs1[1, 1].set_title(r'Yaw($\psi$) History')
    axs1[1, 1].set_xlabel('Time (s)')
    axs1[1, 1].set_ylabel('Angle (deg))')
    axs1[1, 1].legend(loc='lower right')

    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    save_filename = 'drone_state_results(2).png'
    plt.savefig(save_filename)
    
    plt.show()


if __name__ == "__main__":
    refSig = [0.5, -1.0, -5.0, 0.0]
    plot_simulation_results(refSig)
