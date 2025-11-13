classdef Control_LQR < handle

    %% MEMBERS
    properties
        Q
        R
        A
        B

        m

        K
        S
        P
        N_ff

        u0
    end

    %% METHODS
    methods
        function obj = Control_LQR(q, r, droneParams,refSig)
            obj.Q = diag(q);
            obj.R = diag(r);

            obj.m = droneParams('mass');
            g = 9.81;
            obj.u0 = [obj.m*g;0;0;0]; % input of equilibrium position(linearize point)

            [obj.A, obj.B] = stateMatrix(droneParams,refSig);
            [obj.K, obj.S, obj.P] = lqr(obj.A, obj.B, obj.Q, obj.R);

        end

        %% CONTROLLER
        function u = AttitudeCtrl(obj, currentState, refSig)
            
            phi_des = refSig(1);
            theta_des = refSig(2);
            psi_des = refSig(3);
            zdot_des = refSig(4);
            
            ref_vector = [currentState(1), currentState(2), currentState(3),...
                         0, 0, zdot_des,...
                         phi_des, theta_des, psi_des,...
                         0, 0, 0]';

            current_x = currentState;
            error = current_x - ref_vector;

            u_fb =- obj.K*error;

            u = obj.u0 + u_fb;

        end
    end
end
            
