classdef NmpcControl < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %   nmpc.sol.value(nmpc.X)
        % 
        X, U, cost
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Define your problem using the opti object created above

            % Discretize f
            f_discrete_RK4 = @(x,u) RK4(x,u,car.Ts,@car.f);

            %UMAX = [deg2rad(30) ;1];
            M = [1 0;
                -1 0;
                0 1;
                0 -1];
            m = [deg2rad(30) deg2rad(30) 1 1]';

            F = [1 0;
                -1 0;
                0 1;
                0 -1];
            f = [3.5 0.5 deg2rad(5) deg2rad(5)]';

%             YMIN = -0.5;
%             YMAX = 3.5;
% 
%             ThMIN = -deg2rad(5);
%             ThMAX = deg2rad(5);
                
            Q = 10*eye(nx-2);
            R = eye(nu);

            % Variables
            u  = opti.variable(nu, N);
            x  = opti.variable(nx, N+1);

            % Initial conditions
            opti.subject_to(x(:,1) == obj.x0(:,1));

            % Input constraints
            for k = 1:N
                opti.subject_to(M*u(:,k) <= m);
            end

            % State constraints
            for k = 1:N+1
                opti.subject_to(F*x(2:3,k) <= f);
            end

            % Dynamic constraints
            for k = 1:N
                opti.subject_to(x(:, k+1) == f_discrete_RK4(x(:,k), u(:,k)));
            end

            % Cost
            cost = 0;
            x_ref = obj.ref;
            for k = 1:N
                cost = cost + (x([2 4],k)-x_ref)'*Q*(x([2 4],k)-x_ref) + u(:,k)'*R*u(:,k);
            end

            % change this line accordingly
            opti.subject_to( obj.u0 == u(:,1) );

            opti.minimize(cost);

            % Debug variable
            obj.X = x;
            obj.U = u;
            obj.cost = cost;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
