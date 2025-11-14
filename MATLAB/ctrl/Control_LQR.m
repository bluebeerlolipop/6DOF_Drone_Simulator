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
            
            x_des = refSig(1);
            y_des = refSig(2);
            z_des = refSig(3);
            psi_des = refSig(4);
            
            ref_vector = [x_des, y_des, z_des,...
                         0, 0, 0,...
                         0, 0, psi_des,...
                         0, 0, 0]';

            current_x = currentState;
            error = current_x - ref_vector;

            u_fb =- obj.K*error;

            u = obj.u0 + u_fb;

        end
    end
end
            
