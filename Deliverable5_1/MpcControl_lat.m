classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % Matrices
            A = mpc.A;
            B = mpc.B;
            f_xs_us = mpc.f_xs_us;
            
            % Steady-State
            xs = mpc.xs;
            us = mpc.us;

            % Cost matrices
            Q = 10*eye(2);
            R = 1;

            % Constrains
            M = [1;-1];
            m = [deg2rad(30) deg2rad(30)]';

            F = [1 0;
                -1 0;
                0 1;
                0 -1];
            f = [3.5 0.5 deg2rad(5) deg2rad(5)]';

            % Terminal invarience set
            sys = LTISystem('A',A,'B',B);
            sys.x.max = [3.5 deg2rad(5)];
            sys.x.min = [-0.5 deg2rad(-5)];
            sys.u.min = [deg2rad(-30)];
            sys.u.max = [deg2rad(30)];
            sys.x.penalty = QuadFunction(Q);
            sys.u.penalty = QuadFunction(R);
            Kf = sys.LQRGain;
            Qf = sys.LQRPenalty.weight;
            Xf = sys.LQRSet;
            Ff = Xf.A;
            ff = Xf.b;
            
            u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
            umax = deg2rad(30);
            umin = deg2rad(-30);
            xmax = [3.5 deg2rad(5)]';
            xmin = [-0.5 deg2rad(-5)]';

            con = [];
            obj   = 0;
            x = x0;
            for k = 1:N
                x = f_xs_us + A*(x-xs) + B*(u{k}-us);
                obj   = obj + (x-x_ref)'*Q*(x-x_ref) + (u{k}-u_ref)'*R*(u{k}-u_ref);
                con = [con, F*x <= f];
                con = [con, umin <= u{k}<= umax];
            end
            con = [con, Ff*x <= ff];
            obj = obj + (x-x_ref)'*Qf*(x-x_ref);

            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            con = con + ( u0 == u{1} );

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lat.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            debugVars = {x};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            ref = [ref 0]';

            f_xs_us = mpc.f_xs_us;

            [nx, nu] = size(B);

            A_prime = [eye(nx) - A, -B;
                        eye(nx), zeros(nx, nu)];
            b_prime = [f_xs_us - A*xs - B*us; ref];

            S = linsolve(A_prime, b_prime);

            xs_ref = S(1:nx);
            us_ref = S(nx+1:nx+nu);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
