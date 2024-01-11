classdef MpcControl_y < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            



             % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % Horizon and cost matrices
            Q = 10 * eye(4);
            R = 1;
            
            % Constraints
            
            
            % u in U = { u | Mu <= m }
            M = [1;-1]; m = [.26; .26];
            % x in X = { x | Fx <= f }
            F = [0 1 0 0; 0 -1 0 0]; f = [.1745; .1745];
            
            % Compute LQR controller for unconstrained system
            [~,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);

            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE

            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (M*U(:,1) <= m);
            obj = (U(:,1) - u_ref)' * R * (U(:,1) - u_ref);
            for i = 2:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i));
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
                obj = obj + (X(:,i) - x_ref)' * Q * (X(:,i) - x_ref) + (U(:,i) - u_ref)' * R * (U(:,i) - u_ref);
            end
            obj = obj + (X(:,N) - x_ref)' * Qf * (X(:,N) - x_ref);
            

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            Rs = 1;

            obj = us' * Rs * us;
            
            mat = [eye(size(mpc.A,1))-mpc.A -mpc.B; mpc.C zeros(size(mpc.C,1),size(mpc.B,2))];

            con = ((mat * [xs; us]) == [zeros(size(mpc.A,1),1) ; ref]);
            
            % u in U = { u | Mu <= m }
            M = [1;-1]; m = [.26; .26];
            % x in X = { x | Fx <= f }
            F = [0 1 0 0; 0 -1 0 0]; f = [.1745; .1745];
            
            con = con + (F*xs <= f) + (M*us <= m);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
