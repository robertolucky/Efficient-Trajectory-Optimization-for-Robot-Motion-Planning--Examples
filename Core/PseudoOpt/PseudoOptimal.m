classdef PseudoOptimal < handle
    %PSEUDOOPTIMAL pseudo optimal control for simulation, parameter estimation, and optimal control
    %   dependency: chebfun, casadi, ChebTest
    %   author: Yu Zhao, yzhao334@berkeley.edu
    
    properties
        npts;   % number of nodes
        nS;     % dim of states
        nU;     % dim of control
        nO;     % dim of output
        sGuess; % initial guess of states
        Pv0;    % initial value of variable parameters
        getP;   % function handle to get full parameter
        P;      % actually used parameter
        S;      % casadi fcn
    end
    
    properties (Hidden)
       orth;   % orthogonal nodes, weights, differential matrix
       guess;  % guess of initial states, states and parameter, or states and control
               % also time for nodes
       scale;  % time scale
       x;      % casadivariable for variable at nodes
       
       
       %Dynfcn; % Dynfcn:   dynamics function handle for vectorized discretized states and control
               %           format: Dynfcn( Xc,Uc,D,nS,scale,P )
               %                   Xc:    state at nodes
               %                   Uc:    control at nodes
               %                   D:     differential matrix
               %                   nS:    dim of states
               %                   scale: time scale
               %                   P:     parameter
       %Outfcn; % function handle of difference between output and target
               %           format: Outfcn( Xc )
               %           output nOutxN matrix(row for out, col for time)
        
       %target; % target of output, NxnO(row for time, col for var)
       %Pmin;   % lower bound of variable parameter
       %Pmax;   % upper bound of variable parameter
       %outerr; % acceptable difference between output and target
    end
    
    methods
        % function for optimal control
        function [Xc,Uc,Xopt,Uopt] = optimalControl(obj,Dynfcn,Costfcn,Confcn,conlb,conub,time,u,init,maxiter,displev)
            %NEVER USED
            
            %optimal control use collocation
            % input:
            %   Dynfcn:   dynamics function handle for vectorized discretized states and control
            %             format: Dynfcn( Xc,Uc,D,nS,scale,P )
            %                     Xc:    state at nodes
            %                     Uc:    control at nodes
            %                     D:     differential matrix
            %                     nS:    dim of states
            %                     scale: time scale
            %                     P:     parameter
            %   Costfcn:  function handle cost function
            %             format: Costfcn( Xc,Uc,tc,D,nS,nU,w,scale )
            %   Confcn:   function handle of constraint function !!!: need modification and consideration...
            %             format: Confcn( Xc,Uc,D,scale )
            %   conlb:    lower bound of constraint function
            %   conub:    upper bound of constraint function
            %   target:   target of output, NxnO(row for time, col for var)
            %   time:     simulation time
            %   u:        input, corresponding to time(col for control, row for time)
            %   init:     initial condition for states
            %   maxiter:  maximum iteration for optimization
            %   displev:  disp level for optimization
            % output:
            %   Pvid:     discretized states(col for state, row for time)
            %   Xsim:     simulated states corresponding to time(col for state, row for time)
            type = 'Optimal Control';
            import casadi.*;
            disp([type,': Initialize optimization problem ......']);
            init=init(:);% reshape to col vec
            time=time(:).';% reshape to row vec
            fullx=@(x)[init;x];% from xc to get full x with init cond
            [obj.orth.xx,obj.orth.ww,obj.orth.vv]=chebpts(obj.npts);% cheb pnts and weights
            obj.orth.D=ChebTest.getDifferentiationMatrix(obj.orth.xx,obj.orth.vv);% differential matrix
            obj.guess.tSpan=[time(1),time(end)];% get time span
            obj.guess.time=chebpts(obj.npts,obj.guess.tSpan).';% get time nodes
            ls = interp1(time(:),obj.sGuess,obj.guess.time(2:end).');% initial of states at nodes
            obj.guess.state=reshape(ls.',numel(ls),1);% get initial state vec
            Uc=interp1(time(:),u,obj.guess.time(:));% control at nodes
            Uc=reshape(Uc.',numel(Uc),1);
            obj.guess.state=[obj.guess.state;Uc];% get initial state and control vec
            obj.scale=(obj.guess.tSpan(2)-obj.guess.tSpan(1))/2;% time scale            
            ind=obj.nS*(obj.npts-1);% index seperation of state and control
            controleqn = @(x)Dynfcn(fullx(x(1:ind)),...
                x(ind+1:end),obj.orth.D,obj.nS,obj.nU,obj.scale,obj.P);% dynamics constraint
            
            cost = @(x)Costfcn( fullx(x(1:ind)),x(ind+1:end),obj.guess.time,...
                                obj.orth.D,obj.nS,obj.nU,...
                                obj.orth.ww,obj.scale );% cost for optimal control
            controlieq = @(x)Confcn(fullx(x(1:ind)),x(ind+1:end),...
                             obj.orth.D,obj.scale);% constraintfcn bound
            
            obj.x=SX.sym('x',(obj.npts-1)*obj.nS+obj.npts*obj.nU,1);% casadi variable for state at node and parameter
            nlp=struct('x',obj.x,'f',cost(obj.x),...% objective = minimize cost fcn
                'g',[controlieq(obj.x);controleqn(obj.x)]);% constraints = extconstr, dynamics
            
            opts.ipopt.max_iter=maxiter;% set maximum iteration number
            opts.ipopt.print_level=displev;% set display level
            obj.S=nlpsol('S','ipopt',nlp,opts);% optimization problem
            disp([type,': Initialize over, solving ......']);
            r=obj.S('x0',obj.guess.state,'lbg',[kron(ones(obj.npts,1),conlb);zeros(obj.npts*obj.nS,1)],...
                'ubg',[kron(ones(obj.npts,1),conub);zeros(obj.npts*obj.nS,1)]);% solve optimization problem
            xopt=full(r.x);% get numerical result
            
            Xc = fullx(xopt(1:ind));
            Xc = reshape(Xc,obj.nS,[]);
            Xc = Xc.';
            Uc = xopt(ind+1:end);
            Uc = reshape(Uc,obj.nU,[]);
            Uc = Uc.';
            
            Xopt=ChebTest.barycentricInterpolate(time(:),Xc,obj.guess.time(:),obj.orth.vv(:));
            Uopt=ChebTest.barycentricInterpolate(time(:),Uc,obj.guess.time(:),obj.orth.vv(:));
            % y = barycentricInterpolate(x,yk,xk,vk)
            %
            % Interpolates an orthogonal polynomial using barycentric interpolation
            %
            % INPUTS:
            %   x = [nTime, 1] = vector of points to evaluate polynomial at
            %   yk = [nGrid, nOut] = value of the function to be interpolated at each
            %       grid point
            %   xk = [nGrid, 1] = roots of orthogonal polynomial
            %   vk = [nGrid, 1] = barycentric interpolation weights
            
        end
        
        % function for optimal control
        function [Xc,Uc,Tc,Xopt,Uopt,Topt] = timeOptimal(obj,Dynfcn,Costfcn,Confcn,conlb,conub,time,u,init,maxiter,displev)
    
            %USATO IN TEST2D SIMPLE:  2D scara robot, time optimal under
            %kinematic constraints, AL POSTO DEL CONTROOLLO CON LA TORQUE,
            %VIENE CONTROLLATO IN JERK
            
            %USATO IN TEST2D KIN OBSTACLE: 2D scara robot, time optimal
            %under obstacle and kinematic constraints ANCHE QUI IL
            %CONTROLLO � IN JERK MA CI SONO ANCHE GLI OSTACOLI
            
            %USATO IN TEST2D DYN OBSTACLE: 2D scara robot, time optimal
            %under obstacle and dynamic constraints QUI IL CONTROLLO � IN
            %TORQUE VISTO CHE SI CONSIDERA ANCHE LA DINAMICA E NEL COSTO �
            %CONSIDERATO ANCHE L'ENERGIA E LA TORQUE
            
            %USATO IN WAFER2D 2D wafer handling robot, time optimal under
            %obstacle and kinematic constraints COME IL CASO PRECEDENTE MA
            %CON UN ROBOT E CONSTRAINTS DIVERSI
            
            % optimal control use collocation
            % input:
            %   Dynfcn:   dynamics function handle for vectorized discretized states and control
            %             format: Dynfcn( Xc,Uc,D,nS,scale,P )
            %                     Xc:    state at nodes
            %                     Uc:    control at nodes
            %                     D:     differential matrix
            %                     nS:    dim of states
            %                     scale: time scale
            %                     P:     parameter
            %   Costfcn:  function handle cost function
            %             format: Costfcn( Xc,Uc,tc,D,nS,nU,w,scale )
            %   Confcn:   function handle of constraint function !!!: need modification and consideration...
            %             format: Confcn( Xc,Uc,D,scale )
            %   conlb:    lower bound of constraint function
            %   conub:    upper bound of constraint function
            %   target:   target of output, NxnO(row for time, col for var)
            %   time:     simulation time
            %   u:        input, corresponding to time(col for control, row for time)
            %   init:     initial condition for states
            %   maxiter:  maximum iteration for optimization
            %   displev:  disp level for optimization
            % output:
            %   Pvid:     discretized states(col for state, row for time)
            %   Xsim:     simulated states corresponding to time(col for state, row for time)
            type = 'Time Optimal Control';
            import casadi.*;
            disp([type,': Initialize optimization problem ......']);
            init=init(:);% reshape to col vec
            time=time(:).';% reshape to row vec
            fullx=@(x)[init;x];% from xc to get full x with init cond
            [obj.orth.xx,obj.orth.ww,obj.orth.vv]=chebpts(obj.npts);% cheb pnts and weights
            obj.orth.D=ChebTest.getDifferentiationMatrix(obj.orth.xx,obj.orth.vv);% differential matrix
            
            obj.guess.tSpan=[time(1),time(end)];% get time span
            obj.guess.time=chebpts(obj.npts,obj.guess.tSpan).';% get time nodes
            
            ls = interp1(time(:),obj.sGuess,obj.guess.time(2:end).');% initial of states at nodes
            %14x6        1:100     100x6       1x14
            %vq = interp1(x,v,xq) 
            %Vector x contains the sample points, and v contains the corresponding values, v(x). 
            %Vector xq contains the coordinates of the query points.
            %If you have multiple sets of data that are sampled at the same point coordinates,
            %then you can pass v as an array. Each column of array v contains a different set of 1-D sample values.
            %--> prima creiamo 100 valori tra iniziale e
            %target e poi li riduciamo a 10 seguendo il tempo cheb 
            obj.guess.state=reshape(ls.',numel(ls),1);% get initial state vec
            %stessa procedura er inizalizzare il controllo (jerk)
            Uc=interp1(time(:),u,obj.guess.time(:));% control at nodes
            Uc=reshape(Uc.',numel(Uc),1);
            
            obj.guess.state=[obj.guess.state;Uc];% get initial state and control vec
            obj.guess.state=[obj.guess.state;time(end)];
            %^^ accoda stati, controllo e valore del tempo finale, in modo da avere x0
            ind=obj.nS*(obj.npts-1);% index seperation of state and control
            controleqn = @(x)Dynfcn(fullx(x(1:ind)),...
                x(ind+1:end-1),obj.orth.D,obj.nS,obj.nU,x(end)/2,obj.P);% dynamics constraint
            
            cost = @(x)Costfcn( fullx(x(1:ind)),x(ind+1:end-1),obj.guess.time,...
                                obj.orth.D,obj.nS,obj.nU,...
                                obj.orth.ww,x(end)/2 );% cost for optimal control
            controlieq = @(x)Confcn(fullx(x(1:ind)),x(ind+1:end-1),...
                             obj.orth.D,x(end)/2);% constraintfcn bound
            
            obj.x=SX.sym('x',(obj.npts-1)*obj.nS+obj.npts*obj.nU+1,1);% casadi variable for state at node and parameter
                                                 % il secondo � il numero
                                                 % di optimization variable
                
            tic;
            nlp=struct('x',obj.x,'f',cost(obj.x),...% objective = minimize cost fcn
                'g',[controlieq(obj.x);controleqn(obj.x)]);% constraints = extconstr, dynamics
              % qui si imposta effettivamente il problema
            timeinit=toc;
            
            opts.ipopt.max_iter=maxiter;% set maximum iteration number
            opts.ipopt.print_level=displev;% set display level
            obj.S=nlpsol('S','ipopt',nlp,opts);% optimization problem
               %  NLPSOL(char name, char solver, struct:SX nlp, struct opts)
               % stand for nlp solver, so obj.S is the solver
            disp([type,': Initialize over, solving ......']);
            tic;
            %%  AFTER INITIALIZING THERE IS THE REAL SOLLUTION
            r=obj.S('x0',obj.guess.state,'lbg',[conlb;zeros(obj.npts*obj.nS,1)],...
                'ubg',[conub;zeros(obj.npts*obj.nS,1)]);% solve optimization problem
              % res = solver('x0' , [2.5 3.0],...      % solution guess
              % 'lbx', -inf,...           % lower bound on x
              % 'ubx',  inf,...           % upper bound on x
              % 'lbg', -inf,...           % lower bound on g
              % 'ubg',  r);               % upper bound on g
              % avendo messo zero e zero in upper e lower abbiamo equality
              % constraints
            timeop=toc;
            disp(['Initialization time: ',num2str(timeinit)]);
            disp(['Optimization time: ',num2str(timeop)]);
            xopt=full(r.x);% get numerical result
            
            Xc = fullx(xopt(1:ind));
            Xc = reshape(Xc,obj.nS,[]);
            Xc = Xc.';
            Uc = xopt(ind+1:end-1);
            Uc = reshape(Uc,obj.nU,[]);
            Uc = Uc.';
            Tfinal = xopt(end);
            Tc=chebpts(obj.npts,[0,Tfinal]).';% get time nodes
            
            dt = mean(diff(time)); % assume input time uniformally distributed
            
            Topt = linspace(0,Tfinal,ceil(Tfinal/dt)+1);
            Xopt=ChebTest.barycentricInterpolate(Topt(:),Xc,Tc(:),obj.orth.vv(:));
            Uopt=ChebTest.barycentricInterpolate(Topt(:),Uc,Tc(:),obj.orth.vv(:));
            % y = barycentricInterpolate(x,yk,xk,vk)
            %
            % Interpolates an orthogonal polynomial using barycentric interpolation
            %
            % INPUTS:
            %   x = [nTime, 1] = vector of points to evaluate polynomial at
            %   yk = [nGrid, nOut] = value of the function to be interpolated at each
            %       grid point
            %   xk = [nGrid, 1] = roots of orthogonal polynomial
            %   vk = [nGrid, 1] = barycentric interpolation weights
            
        end
        
        % function for time optimal control, initialization
        function [obj] = timeOptimalInit(obj,Dynfcn,Costfcn,Confcn,maxiter,displev)
            %USED IN ALL THE 6 AXIS EXAMPLE TO HAVE A FAST RESOLUTION WHEN
            %TARGET CHANGE, THE NEXT FUNCTION CALLED IS timeOptimalOptimize
            type = 'Time Optimal Control';
            import casadi.*;
            disp([type,': Initialize optimization problem ......']);
            [obj.orth.xx,obj.orth.ww,obj.orth.vv]=chebpts(obj.npts);% cheb pnts and weights
            obj.orth.D=ChebTest.getDifferentiationMatrix(obj.orth.xx,obj.orth.vv);% differential matrix
            
            ind=obj.nS*obj.npts;% index seperation of state and control
            controleqn = @(x)Dynfcn(x(1:ind),...
                x(ind+1:end-1),obj.orth.D,obj.nS,obj.nU,x(end)/2,obj.P);% dynamics constraint
            
            cost = @(x)Costfcn( x(1:ind),x(ind+1:end-1),[],...
                                obj.orth.D,obj.nS,obj.nU,...
                                obj.orth.ww,x(end)/2 );% cost for optimal control
            controlieq = @(x)Confcn(x(1:ind),x(ind+1:end-1),...
                             obj.orth.D,x(end)/2);% constraintfcn bound
            
            obj.x=SX.sym('x',obj.npts*obj.nS+obj.npts*obj.nU+1,1);% casadi variable for state at node and parameter
            tic;
            nlp=struct('x',obj.x,'f',cost(obj.x),...% objective = minimize cost fcn
                'g',[controlieq(obj.x);controleqn(obj.x)]);% constraints = extconstr, dynamics
            timeinit=toc;
            opts.ipopt.max_iter=maxiter;% set maximum iteration number
            opts.ipopt.print_level=displev;% set display level
            obj.S=nlpsol('S','ipopt',nlp,opts);% optimization problem
            disp(['Initialization time: ',num2str(timeinit)]);
            
        end
        
        % function for time optimal control, optimization
        function [Xc,Uc,Tc,Xopt,Uopt,Topt] = timeOptimalOptimize(obj,conlb,conub,time,u)
            %USATO DOPO timeOptimalInit
            type = 'Time Optimal Control';
            disp([type,': Solving ......']);
            time=time(:).';% reshape to row vec
            obj.guess.tSpan=[time(1),time(end)];% get time span
            obj.guess.time=chebpts(obj.npts,obj.guess.tSpan).';% get time nodes
            ls = interp1(time(:),obj.sGuess,obj.guess.time(:).');% initial of states at nodes
            obj.guess.state=reshape(ls.',numel(ls),1);% get initial state vec
            Uc=interp1(time(:),u,obj.guess.time(:));% control at nodes
            Uc=reshape(Uc.',numel(Uc),1);
            obj.guess.state=[obj.guess.state;Uc];% get initial state and control vec
            obj.guess.state=[obj.guess.state;time(end)];
            
            tic;
            r=obj.S('x0',obj.guess.state,'lbg',[conlb;zeros(obj.npts*obj.nS,1)],...
                'ubg',[conub;zeros(obj.npts*obj.nS,1)]);% solve optimization problem
            timeop=toc;
            disp(['Optimization time: ',num2str(timeop)]);
            
            % a qui in poi � chiarissimo
            xopt=full(r.x);% get numerical result
            
            ind = obj.nS*obj.npts;
            Xc = xopt(1:ind);
            Xc = reshape(Xc,obj.nS,[]);
            Xc = Xc.';
            Uc = xopt(ind+1:end-1);
            Uc = reshape(Uc,obj.nU,[]);
            Uc = Uc.';
            Tfinal = xopt(end);
            Tc=chebpts(obj.npts,[0,Tfinal]).';% get time nodes
            
            dt = mean(diff(time)); % assume input time uniformally distributed
            
            Topt = linspace(0,Tfinal,ceil(Tfinal/dt)+1);
            Xopt=ChebTest.barycentricInterpolate(Topt(:),Xc,Tc(:),obj.orth.vv(:));
            Uopt=ChebTest.barycentricInterpolate(Topt(:),Uc,Tc(:),obj.orth.vv(:));
            % y = barycentricInterpolate(x,yk,xk,vk)
            %
            % Interpolates an orthogonal polynomial using barycentric interpolation
            %
            % INPUTS:
            %   x = [nTime, 1] = vector of points to evaluate polynomial at
            %   yk = [nGrid, nOut] = value of the function to be interpolated at each
            %       grid point
            %   xk = [nGrid, 1] = roots of orthogonal polynomial
            %   vk = [nGrid, 1] = barycentric interpolation weights
            
        end
        
        % function for optimal control, initialization
        function [obj] = optimalInit(obj,Dynfcn,Costfcn,Confcn,time,maxiter,displev)
            % NEVER USED
            type = 'Optimal Control';
            import casadi.*;
            disp([type,': Initialize optimization problem ......']);
            time=time(:).';% reshape to row vec
            obj.guess.tSpan=[time(1),time(end)];% get time span
            obj.guess.time=chebpts(obj.npts,obj.guess.tSpan).';% get time nodes
            obj.scale=(obj.guess.tSpan(2)-obj.guess.tSpan(1))/2;% time
            [obj.orth.xx,obj.orth.ww,obj.orth.vv]=chebpts(obj.npts);% cheb pnts and weights
            obj.orth.D=ChebTest.getDifferentiationMatrix(obj.orth.xx,obj.orth.vv);% differential matrix
            ind=obj.nS*obj.npts;% index seperation of state and control
            controleqn = @(x)Dynfcn(x(1:ind),...
                x(ind+1:end),obj.orth.D,obj.nS,obj.nU,obj.scale,obj.P);% dynamics constraint
            
            cost = @(x)Costfcn( x(1:ind),x(ind+1:end),obj.guess.time,...
                                obj.orth.D,obj.nS,obj.nU,...
                                obj.orth.ww,obj.scale );% cost for optimal control
            controlieq = @(x)Confcn(x(1:ind),x(ind+1:end),...
                             obj.orth.D,obj.scale);% constraintfcn bound
            
            obj.x=SX.sym('x',obj.npts*obj.nS+obj.npts*obj.nU,1);% casadi variable for state at node and parameter
            tic;
            nlp=struct('x',obj.x,'f',cost(obj.x),...% objective = minimize cost fcn
                'g',[controlieq(obj.x);controleqn(obj.x)]);% constraints = extconstr, dynamics
            timeinit=toc;
            opts.ipopt.max_iter=maxiter;% set maximum iteration number
            opts.ipopt.print_level=displev;% set display level
            obj.S=nlpsol('S','ipopt',nlp,opts);% optimization problem
            disp(['Initialization time: ',num2str(timeinit)]);
            
        end
        
        % function for optimal control, optimization
        function [Xc,Uc,Tc,Xopt,Uopt,Topt] = optimalOptimize(obj,conlb,conub,time,u)
            %NEVER USED
            type = 'Optimal Control';
            disp([type,': Solving ......']);
            ls = interp1(time(:),obj.sGuess,obj.guess.time(:).');% initial of states at nodes
            obj.guess.state=reshape(ls.',numel(ls),1);% get initial state vec
            Uc=interp1(time(:),u,obj.guess.time(:));% control at nodes
            Uc=reshape(Uc.',numel(Uc),1);
            obj.guess.state=[obj.guess.state;Uc];% get initial state and control vec
            
            tic;
            r=obj.S('x0',obj.guess.state,'lbg',[conlb;zeros(obj.npts*obj.nS,1)],...
                'ubg',[conub;zeros(obj.npts*obj.nS,1)]);% solve optimization problem
            timeop=toc;
            disp(['Optimization time: ',num2str(timeop)]);
            xopt=full(r.x);% get numerical result
            
            ind = obj.nS*obj.npts;
            Xc = xopt(1:ind);
            Xc = reshape(Xc,obj.nS,[]);
            Xc = Xc.';
            Uc = xopt(ind+1:end);
            Uc = reshape(Uc,obj.nU,[]);
            Uc = Uc.';
            Tc=obj.guess.time(:);% get time nodes
            
            Topt=time(:);
            Xopt=ChebTest.barycentricInterpolate(Topt(:),Xc,Tc(:),obj.orth.vv(:));
            Uopt=ChebTest.barycentricInterpolate(Topt(:),Uc,Tc(:),obj.orth.vv(:));
            % y = barycentricInterpolate(x,yk,xk,vk)
            %
            % Interpolates an orthogonal polynomial using barycentric interpolation
            %
            % INPUTS:
            %   x = [nTime, 1] = vector of points to evaluate polynomial at
            %   yk = [nGrid, nOut] = value of the function to be interpolated at each
            %       grid point
            %   xk = [nGrid, 1] = roots of orthogonal polynomial
            %   vk = [nGrid, 1] = barycentric interpolation weights
            
        end
        
    end
    
end

