import numpy as np
import os
import sys
import matplotlib.pyplot as plt

script_dir = os.path.dirname(os.path.abspath(__file__))

lib_path = os.path.join(script_dir, 'lib')
ctrl_path = os.path.join(script_dir, 'ctrl')

if lib_path not in sys.path:
    sys.path.append(lib_path)
if ctrl_path not in sys.path:
    sys.path.append(ctrl_path)


from Drone_State import Drone_State
from RPY2Rot import RPY2Rot
from Control_Position import Control_Position
from Control_PID import Control_PID
from Control_LQR import Control_LQR
from plot_sim import plot_simulation_results

def main():
    R2D = 180 / np.pi
    D2R = np.pi / 180
    
    # Simulation time
    simulationTime = 20.0
    dt = 0.01
    numStep = int(simulationTime / dt)
    
    # Initial parameters
    drone1_params = {
        'mass': 1.25, 'armLength': 0.265, 
        'Ixx': 0.0232, 'Iyy': 0.0232, 'Izz': 0.0468
    }
    
    drone1_initStates = np.array([
        0, 0, -6,   # X, Y, Z
        0, 0, 0,    # dX, dY, dZ
        0, 0, 0,    # phi, theta, psi
        0, 0, 0     # p, q, r
    ])
    ###################### Set Controller Gains ######################
    # Position Controller Gains
    pos1_gains = {
        'P_x': 0.2, 'I_x': 0.0, 'D_x': 0.15,
        'P_y': 0.2, 'I_y': 0.0, 'D_y': 0.15,
        'P_z': 1.0, 'I_z': 0.0, 'D_z': 2.0 # 2.0
    }
    
    # Attitude Controller Gains
    drone1_gains = {
        'P_phi': 0.2, 'I_phi': 0.0, 'D_phi': 0.15,
        'P_theta': 0.2, 'I_theta': 0.0, 'D_theta': 0.15,
        'P_psi': 0.2, 'I_psi': 0.0, 'D_psi': 0.15
    }

    # LQR Controller Gains
    drone1_q = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    drone1_r = np.array([1, 1, 1, 1])

    ##################################################################
    
    # History
    stateHistory = np.zeros((numStep + 1, len(drone1_initStates)))
    stateHistory[0, :] = drone1_initStates
    
    # Command signal
    pos_cmd = [0.5, -1.0, -5]
    psi_cmd = 0.0 * D2R
    commandSig = np.array([pos_cmd[0], pos_cmd[1], pos_cmd[2], psi_cmd])

    ###################### Initialize Part ######################
    # Initialize Drone Dynamics
    drone1 = Drone_State(drone1_params, drone1_initStates, simulationTime, dt)
    # Initialize Position Controller
    controller_pos = Control_Position(pos1_gains, drone1_params, dt)
    # Initialize Attitude Controller
    controller1 = Control_PID(drone1_gains, dt)
    # Initialize LQR Controller
    controller2 = Control_LQR(drone1_q, drone1_r, drone1_params, commandSig)
    
    u = np.zeros(4) # [Thrust, M1, M2, M3]

    ##############################################################

    print("Starting simulation.")
    # SIMULATION LOOP
    for i in range(numStep):
        drone1_state = drone1.GetState()
        
        # # Run PID controllers
        # u_pos, cmd = controller_pos.PositionCtrl(drone1_state, commandSig)
        # u_control = controller1.AttitudeCtrl(drone1_state, cmd)
        
        # u[0] = u_pos            # thrust
        # u[1:4] = u_control      # M1, M2, M3
        
        # # Update drone state
        # drone1.UpdateState(u)
        # drone1_state = drone1.GetState()

        # # Store history
        # stateHistory[i + 1, :] = drone1.state

        # Run LQR controllers
        u = controller2.AttitudeCtrl(drone1_state, commandSig)
        drone1.UpdateState(u)
        drone1_state = drone1.GetState()
        stateHistory[i + 1, :] = drone1_state
        
        # Check for crash
        if drone1_state[2] >= 0:
            print(f"Crashed at step {i}!")
            stateHistory = stateHistory[:i+2, :] # Trim history
            break
            
    print("Simulation finished.")
    
    # Save results
    save_path = os.path.join(script_dir, 'stateHistory.npy')
    np.save(save_path, stateHistory)

# Standard Python entry point
if __name__ == "__main__":
    main()
    refSig = [0.5, -1.0, -5.0, 0.0]
    plot_simulation_results(refSig)
