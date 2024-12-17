classdef LonEstimator
    properties
        % continous sub-system
        sys
        % Extended linearization points
        xs_hat, us_hat
        % Extended system matrices
        A_hat, B_hat, C_hat
        % Observer gain matrix
        L
    end
    
    methods
        % Setup the longitudinal disturbance estimator
        function est = LonEstimator(sys, Ts)

            xs = sys.UserData.xs;
            us = sys.UserData.us;
            
            % Discretize the system and extract the A,B,C,D matrices
            [~, Ad, Bd, Cd, ~] = Car.c2d_with_offset(sys, Ts);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            A = Ad(2,2);
            B = Bd(2);
            C = Cd(2,2);

            xs = xs(2);
            
            [nx, nu] = size(B);
            ny = size(C,1);

            % Extended state dynamics
            est.xs_hat = [xs;0];
            est.us_hat = [us];
            est.A_hat = [A , B;
                        zeros(nu,nx), eye(nu)];
            est.B_hat = [B;zeros(1,nu)];
            est.C_hat = [C,zeros(ny,1)];
            
            est.L = -place(est.A_hat',est.C_hat',[0.7,0.8])';
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % This function takes in the the estimate, input, and measurement
        % at timestep i and predicts the next (extended) state estimate at
        % the next timestep i + 1.
        function z_hat_next = estimate(est, z_hat, u, y)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   z_hat      - current estimate (V, dist)
            %   u          - longitudinal input (u_T)
            %   y          - longitudinal measurement (x, V)
            % OUTPUTS
            %   z_hat_next - next time step estimate
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % Estimation equation
            z_hat_next = est.A_hat*(z_hat-est.xs_hat) + est.B_hat*(u-est.us_hat) + est.xs_hat - est.L*(y - est.C_hat*z_hat);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
