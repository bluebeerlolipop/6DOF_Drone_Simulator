import numpy as np
import os
import sys
# from drone_state import Drone_State
# from control_position import Control_Position
# from control_pid import Control_PID
# import matplotlib.pyplot as plt # For plotting results

# --- (Import classes from other files) ---
# (위의 2, 3, 4번 코드들이 별도 파일로 저장되어 있다고 가정)
script_dir = os.path.dirname(os.path.abspath(__file__))

lib_path = os.path.join(script_dir, 'lib')
ctrl_path = os.path.join(script_dir, 'ctrl')

if lib_path not in sys.path:
    sys.path.append(lib_path)
if ctrl_path not in sys.path:
    sys.path.append(ctrl_path)

try:
    from Drone_State import Drone_State
    from RPY2Rot import RPY2Rot
    from Control_Position import Control_Position
    from Control_PID import Control_PID
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Ensure 'lib' contains drone_state.py and RPY2Rot.py")
    print("Ensure 'ctrl' contains control_position.py and control_pid.py")
    sys.exit(1) # 문제가 있으면 스크립트 중지

def main():
    R2D = 180 / np.pi
    D2R = np.pi / 180
    
    # Simulation time
    simulationTime = 10.0
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
    
    # Position Controller Gains
    pos1_gains = {
        'P_x': 0.2, 'I_x': 0.0, 'D_x': 0.15,
        'P_y': 0.2, 'I_y': 0.0, 'D_y': 0.15,
        'P_z': 1.0, 'I_z': 0.0, 'D_z': 2.0
    }
    
    # Attitude Controller Gains
    drone1_gains = {
        'P_phi': 0.2, 'I_phi': 0.0, 'D_phi': 0.15,
        'P_theta': 0.2, 'I_theta': 0.0, 'D_theta': 0.15,
        'P_psi': 0.2, 'I_psi': 0.0, 'D_psi': 0.15
    }
    
    # History
    stateHistory = np.zeros((numStep + 1, len(drone1_initStates)))
    stateHistory[0, :] = drone1_initStates
    
    # Command signal
    pos_cmd = [0.1, -0.2, -6]
    psi_cmd = 0.0 * D2R
    commandSig = np.array([pos_cmd[0], pos_cmd[1], pos_cmd[2], psi_cmd])

    # 1. Initialize Drone Dynamics
    drone1 = Drone_State(drone1_params, drone1_initStates, simulationTime, dt)
    # 2. Initialize Position Controller
    controller_pos = Control_Position(pos1_gains, drone1_params, dt)
    # 3. Initialize Attitude Controller
    controller1 = Control_PID(drone1_gains, dt)
    
    u_control = np.zeros(4) # [Thrust, M1, M2, M3]

    print("Starting simulation...")
    # SIMULATION LOOP
    for i in range(numStep):
        drone1_state = drone1.GetState()
        
        # Run controllers
        u_pos, cmd = controller_pos.PositionCtrl(drone1_state, commandSig)
        u_att = controller1.AttitudeCtrl(drone1_state, cmd)
        
        u_control[0] = u_pos      # thrust
        u_control[1:4] = u_att    # M1, M2, M3
        
        # Update drone state
        drone1.UpdateState(u_control)
        
        # Store history
        stateHistory[i + 1, :] = drone1.GetState()
        
        # Check for crash
        if drone1_state[2] >= 0: # Z position >= 0 (ground)
            print(f"Crashed at step {i}!")
            stateHistory = stateHistory[:i+2, :] # Trim history
            break
            
    print("Simulation finished.")
    
    # Save results
    np.save('stateHistory.npy', stateHistory)
    print("State history saved to 'stateHistory.npy'")

    # (Optional) Plotting
    # plt.figure()
    # plt.plot(stateHistory[:, 0], label='X')
    # plt.plot(stateHistory[:, 1], label='Y')
    # plt.plot(stateHistory[:, 2], label='Z')
    # plt.legend()
    # plt.title("Position")
    # plt.show()

# Standard Python entry point
if __name__ == "__main__":
    main()