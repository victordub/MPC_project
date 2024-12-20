classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            load('tube_mpc_data.mat');

            A = mpc.A;
            B = mpc.B;
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE

            % Tightened constraints
            F_tilde = X_tilde.A;
            f_tilde = X_tilde.b;

            Ff_tilde = Xf_delta.A;
            ff_tilde = Xf_delta.b;

            M_tilde = U_tilde.A;
            m_tilde = U_tilde.b;

            u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
            %delta = sdpvar(repmat(1,1,N),repmat(1,1,N));

            con = [];
            obj   = 0;

            % initial conditions
            delta0 = x0other - x0 - [x_safe;0];
            delta = delta0;
            for k = 1:N
                delta = A*delta - B*u{k};
                obj   = obj + delta'*Q*delta + u{k}'*R*u{k};
                con = [con, M_tilde*u{k} <= m_tilde];
                con = [con, F_tilde*delta <= f_tilde];
            end
            con = [con, Ff_tilde*delta <= ff_tilde];
            obj = obj + delta'*Qf*delta;

            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            con = con + ( u0 == K*(x0 - delta0) + u{1} );

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lon.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            debugVars = {};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','quadprog'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state subsystem
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);

            % Subsystem linearization steady-state
            xs = mpc.xs(2);
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            f_xs_us = mpc.f_xs_us(2);

            [nx, nu] = size(B);

            A_prime = [eye(nx) - A, -B;
                        eye(nx), zeros(nx, nu)];
            b_prime = [f_xs_us - A*xs - B*us; ref];

            S = linsolve(A_prime, b_prime);

            Vs_ref = S(1:nx);
            us_ref = S(nx+1:nx+nu);
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
