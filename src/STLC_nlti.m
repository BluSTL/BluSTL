classdef STLC_nlti < handle
    %STLC_nlti generic x' = f(x,u) system controller. Linearlized at each time step.
    
    % system properties
    properties
        sys      % system as control toolbox objec
        sysd
        nx
        ny
        nu
        nw
        x0
        K
        system_data
        is_det  % if this is 1, will use only Wref as disturbance (e.g., ignore
        % potential uncertainty defined in w_ub and w_lb).
        % If empty, this is decided in reset_data() based on w_ub and w_lb.
    end
    
    
    % controller properties
    properties
        u_lb
        u_ub
        u_delta
        ts         % sampling time
        L          % horizon
        time
        nb_stages
        var
        stl_list
        encoding  % specifies the technique for encoding TODO
        min_rob    % TODO: if rob==0 use non robust encoding
        lambda_rho %  weight of robustness in the cost function
        bigM
        solver_options
        model_data
        controller    % YALMIP parametric problem for the controller
        
    end
    
    % adversary properties
    properties
        Wref           % this defines a default or initial disturbance vector
        w_lb           % lower bound on w relative to Wref
        w_ub           % upper bound on w relative to Wref
        stl_w_list     % stl properties for environment (TODO)
        max_react_iter % maximum number of iterations
        adversary      % YALMIP parametric problem for the adversary
    end
    
    % plotting properties
    properties
        h
        xlabel  % labels (names) for x signals
        ylabel
        ulabel
        wlabel
        plot_x  % indices of states to plot
        plot_y  % indices of outputs to plot
        plot_u  % index of inputs to plot
        plot_w  % index of disturbances to plot
    end
    
    % misc
    properties
        stop_button
        verbose
    end
    
    methods
        % Constructor
        function Sys = STLC_nlti(varargin)
            
            switch nargin
                
                case 2
                    % constructor from AB system
                    A = varargin{1};
                    Bu = varargin{2};
                    Bw = [];
                    C = [];
                    Du = [];
                    Dw = [];
                    
                case 3
                    % constructor from ABuBw system
                    A = varargin{1};
                    Bu = varargin{2};
                    Bw = varargin{3};
                    C = [];
                    Du = [];
                    Dw = [];
                    
                case 4
                    % constructor from ABCD system
                    A = varargin{1};
                    Bu = varargin{2};
                    Bw = [];
                    C = varargin{3};
                    Du = varargin{4};
                    Dw = [];
                    
                    % constructor from ABuBwCDuDw system
                case 6
                    A = varargin{1};
                    Bu = varargin{2};
                    Bw = varargin{3};
                    C = varargin{4};
                    Du = varargin{5};
                    Dw = varargin{6};
                    
            end
            
            nx = max([size(A,1), size(Bu,1), size(Bw,1),1]);
            nu = max([size(Bu,2), size(Du,1)]);
            if nu == 0
                errormsg('System must have at least one input (non empty Bu or Du matrix)');
            end
            nw = max([size(Bw,2), size(Dw,1),1]);
            ny = max([size(C,1), size(Du,1), size(Dw,1),1]);
            
            % default plots everything
            Sys.plot_x = 1:max([size(A,1), size(Bu,1), size(Bw,1)]);
            Sys.plot_u = 1:nu;
            Sys.plot_y = 1:max([size(C,1), size(Du,1), size(Dw,1)]);
            Sys.plot_w = 1:max([size(Bw,2), size(Dw,1)]);
            
            
            % checks empty matrices
            if isempty(A) % no state
                A = zeros(nx,nx);
            end
            
            if isempty(Bu)
                Bu = zeros(nx,nu);
            end
            
            if isempty(Bw) % no disturbance
                Bw = zeros(nx,nw);
            end
            
            if isempty(C)
                C = zeros(ny,nx);
            end
            
            if isempty(Du)
                Du = zeros(ny,nu);
            end
            
            if isempty(Dw)
                Dw = zeros(ny,nw);
            end
            
            Sys.sys = ss(A,[Bu Bw],C,[Du Dw]);
            Sys.nx = size(A,2);
            Sys.nu = size(Bu,2);
            Sys.nw = size(Bw,2);
            Sys.ny = size(C,1);
            Sys.x0 = zeros(Sys.nx,1);
            Sys.K = zeros(Sys.nx,1); % additive component for control

            
            
            % default label names
            Sys.xlabel = cell(1,nx);
            for iX = 1:Sys.nx
                Sys.xlabel{iX} = ['x' num2str(iX)];
            end
            
            for iY = 1:Sys.ny
                Sys.ylabel{iY} = ['y' num2str(iY)];
            end
            
            for iU = 1:Sys.nu
                Sys.ulabel{iU} = ['u' num2str(iU)];
            end
            
            for iW = 1:Sys.nw
                Sys.wlabel{iW} = ['w' num2str(iW)];
            end
            
            % default options
            
            solver = 'gurobi';  % gurobi, cplex, glpk
            timeLimit = 2000;
            gapLimit = 0.1;
            solnLimit = Inf;
            verb = 1;
            Sys.solver_options = sdpsettings('verbose', verb,'solver', solver, ...
                          'gurobi.TimeLimit', timeLimit, ...
                          'gurobi.MIPGap', gapLimit, ...
                          'gurobi.SolutionLimit', solnLimit,'cachesolvers',1);

            Sys.min_rob = 0.01;
            Sys.lambda_rho = 0;
            Sys.bigM = 1000;
            Sys.u_delta = Inf;
            Sys.max_react_iter = 10;
            Sys.nb_stages = 1;
            Sys.stop_button = 0;
            
            % default values for input constraints - note, forces u to 0
            Sys.u_lb = zeros(1,Sys.nu);
            Sys.u_ub = zeros(1,Sys.nu);
            Sys.u_delta = Inf*ones(1,Sys.nu);
            
            % default values for disturbance constraints - note:w_lb and w_ub are relative to Wref
            % i.e.,   wref+w_lb  <= w <= wref + w_ub. Thus by default, w == Wref
            Sys.w_lb = zeros(1,Sys.nw);
            Sys.w_ub = zeros(1,Sys.nw);
            
        end
        
        % default objective function r is the robust sat. and wr a weight
        function obj = get_objective(Sys, X, Y, U,W, rho,wr)
            switch nargin
                case {4,5}
                    obj = sum(sum(abs(U))); % minimize U
                case 6
                    obj = sum(sum(abs(U)))-sum(sum(rho)); % minimize U penalized by r
                case 7
                    obj = sum(sum(abs(U)))-wr*sum(sum(rho));
            end
        end
               
        % System step, using the continuous time simulation
        function [Y,T,X] = system_step(Sys, u0, t, x0, w0)
            U = [u0; w0];
            [Y,T,X] = lsim(Sys.sys, U',t-t(1),x0);
        end
        
        % Get disturbance - default reads Wref, adds random value in
        % disturbance range
        function w = get_disturbance(Sys)
            
            it = Sys.system_data.time_index;
            w = [Sys.Wref(:,it) Sys.Wref(:,it+1)];
            
            if ~Sys.is_det
                dw = ((Sys.w_ub-Sys.w_lb)').*(2*rand(Sys.nw,1)-1)/3;
                w = w + [dw dw];
            end
            
        end
              
        % Applies input for one discrete step, updates the model data
        % requires that compute_input was called before
        function Sys = apply_input(Sys)
            Sys = STLC_apply_input(Sys);
        end
        
        % Computes the optimizer object for the controller
        function controller = get_controller(Sys,enc)
            if nargin < 2
                if isempty(Sys.encoding)
                    enc = 'robust';
                else
                    enc = Sys.encoding;
                end
            end
            Sys.sysd = c2d(Sys.sys, Sys.ts);
            controller = STLC_get_controller(Sys,enc);
        end
        
        % Computes the optimizer object for the adversary/disturbance
        function adversary = get_adversary(Sys)
            Sys.sysd = c2d(Sys.sys, Sys.ts);
            adversary = STLC_get_adversary(Sys);
        end
        
        % reset system and model data
        function Sys = reset_data(Sys)
            
            if isempty(Sys.Wref)
                Sys.Wref = 0*Sys.time;
            end
            
            if isempty(Sys.is_det)
                if any(Sys.w_ub-Sys.w_lb)
                    Sys.is_det = 0;
                else
                    Sys.is_det = 1;
                end
            end
            
            % implements nb_stages here
            if Sys.nb_stages>1
                fprintf('Updating time and Wref based on nb_stages=%d\n', Sys.nb_stages); % TODO unverbose/verbose that out
                nb_stages_=Sys.nb_stages;
                time_ = Sys.time;
                ntime = zeros(1, nb_stages_*numel(time_));
                for istage = 0:nb_stages_-1
                    ntime(istage*numel(time_)+1:(istage+1)*numel(time_))= time_+istage*(time_(end)+time_(2)) ;
                end
                Sys.time = ntime;
                Sys.Wref = repmat(Sys.Wref,1,Sys.nb_stages);
                Sys.nb_stages=1;
            end
            
            Sys.sysd = c2d(Sys.sys, Sys.ts);
            Sys.system_data=struct;
            Sys.model_data=struct;
            
            Sys.system_data.time = [];
            Sys.system_data.U = [];
            Sys.system_data.X = Sys.x0;
            Sys.system_data.Y = [];
            Sys.system_data.W = [];
            Sys.system_data.time_index = 1;
            
            Sys.model_data.time_index = 0;
            Sys.model_data.time = [];
            Sys.model_data.X = [];
            Sys.model_data.Y = [];
            Sys.model_data.U = [];
            Sys.model_data.W = [];
            Sys.model_data.Wbad = [];
            
        end
        
        %
        function [Sys, status] = compute_input(Sys, controller)
            % computes the next input and update model data
            % status is 0 if everything is OK
            [Sys, status] = STLC_compute_input(Sys, controller);
        end
        
        % computes the next disturbance and update model data
        % status is 0 if a bad disturbance was found, 1 if control is
        % good (no bad disturbance exist and -1 if some other error occured
        function [Sys, status] = compute_disturbance(Sys, adversary)
            [Sys, status] = STLC_compute_disturbance(Sys, adversary);
        end
        
        % Combines compute_input and compute_disturbance, and tries to
        % find an input valid for all possible disturbances.
        function [Sys, status_u, status_w] = compute_input_adv(Sys, controller, adversary)
            [Sys, status_u, status_w] = STLC_compute_input_adv(Sys, controller, adversary);
        end
        
        
        % Executes the controller in open loop mode
        function [Sys] = run_open_loop(Sys, controller)
            Sys = STLC_run_open_loop(Sys, controller);
        end
        
        % Executes the controller in a receding horizon (MPC)
        function [Sys] = run_deterministic(Sys, controller)
            STLC_run_deterministic(Sys, controller);
        end
        
        % Executes controller and adversary in open loop
        function Sys = run_open_loop_adv(Sys, controller, adversary)
            Sys = Sys.reset_data();
            [Sys, status_u, status_w] = compute_input_adv(Sys, controller, adversary);
            
            if (status_w ~= 1)
                warning('The control input might not be robust.')
            end
            
            if status_u==0
                current_time =0;
                while (current_time < Sys.model_data.time(end)-Sys.ts)
                    out = sprintf('time:%g', current_time );
                    rfprintf(out);
                    Sys = Sys.apply_input();
                    Sys = Sys.update_plot();
                    drawnow;
                    current_time= Sys.system_data.time(end);
                end
                fprintf('\n');
            end
            
        end
        
        % Executes controller and adversary in receding horizon mode (MPC)
        function Sys = run_adversarial(Sys, controller, adversary)
            global StopRequest;
            StopRequest=0;
            Sys = Sys.reset_data();
            rfprintf_reset();
            current_time =0;
            while ((current_time < Sys.time(end)-Sys.L*Sys.ts)&&StopRequest==0)
                out = sprintf('time:%g', current_time );
                rfprintf(out);
                [Sys, status_u, status_w] = Sys.compute_input_adv(controller, adversary);
                
                if status_u~=0
                    rfprintf_reset();
                    StopRequest=1;
                end
                
                Sys = Sys.apply_input();
                Sys = Sys.update_plot();
                drawnow;
                current_time= Sys.system_data.time(end);
            end
            fprintf('\n');
        end
        
        
        % Default plot function
        function Sys = update_plot(Sys)
            Sys = STLC_update_plot(Sys);
        end
        
        function Wn = sensing(Sys)
            Wn = STLC_sensing(Sys);
        end
        
        % linearlization
        function obj = get_objective(Sys, X, Y, U,W, rho,wr)
            switch nargin
                case {4,5}
                    obj = sum(sum(abs(U))); % minimize U
                case 6
                    obj = sum(sum(abs(U)))-sum(sum(rho)); % minimize U penalized by r
                case 7
                    obj = sum(sum(abs(U)))-wr*sum(sum(rho));
            end
        end
        
    end
end


