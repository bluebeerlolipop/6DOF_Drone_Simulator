% main_refactored.m
clc;
clear;
close all
addpath('./lib');
%% DEFINE
R2D = 180/pi;
D2R = pi/180;
%% Simulation time
simulationTime = 2;
dt = 0.01;

%% INITIAL PARAMS
% can initialize parameter, when using PID control, you must initialize
% Gain K.
drone1_params = containers.Map({'mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468});
drone1_initStates = [0, 0, -6, ...       % X, Y, Z
    0, 0, 0, ...                        % dX, dY, dZ
    0, 0, 0, ...                        % phi, theta, psi
    0, 0, 0]';                          % p, q, r

drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';

%PID Gain(optional when you use PID controller)
drone1_gains = containers.Map(...
    {'P_phi', 'I_phi', 'D_phi', ...
    'P_theta', 'I_theta', 'D_theta', ...
    'P_psi', 'I_psi', 'D_psi', ...
    'P_zdot', 'I_zdot', 'D_zdot'}, ...
    {0.2, 0.0, 0.15, ...
    0.2, 0.0, 0.15, ...
    0.2, 0.0, 0.15, ...
    10.0, 0.05, 0.0});

%% Generate .mat file
numStep = simulationTime/dt;
stateHistory = zeros(numStep, length(drone1_initStates));
stateHistory(1, :) = drone1_initStates';

%% 객체 생성(초기화)
% 1. import drone dynamics
drone1 = Drone_State(drone1_params, drone1_initStates, simulationTime, dt);
% 2. import drone controller
controller1 = Control_PID(drone1_gains, drone1_params, dt);

%% command signal
commandSig(1) = 10.0 * D2R; % phi
commandSig(2) = 10.0 * D2R; % theta
commandSig(3) = 10.0 * D2R; % psi
commandSig(4) = -1.0; % z_dot

%% SIMULATION LOOP
for i = 1:simulationTime/dt
    drone1_state = drone1.GetState();
    u = controller1.AttitudeCtrl(drone1_state, commandSig);
    drone1.UpdateState(u);
    drone1_state = drone1.GetState();
    stateHistory(i+1, :) = drone1_state;

    if (drone1_state(3) >= 0)
        msgbox('Crashed!!', 'Error', 'error');
        break;
    end
end

save('stateHistory.mat', 'stateHistory');
plot_sim;