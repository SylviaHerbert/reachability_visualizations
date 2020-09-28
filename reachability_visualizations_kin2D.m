function reachability_visualizations_kin2D()

% visualize options: 'function','set','none'
visualize = 'function';
video = 0;
obstacle = 1;
simulation = 1;

% initial state for online simulation
xinit = [-2, -1];

%% Grid
grid_min = [-5; -5]; % Lower corner of computation domain
grid_max = [5; 5];    % Upper corner of computation domain
N = [81; 81];         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set

% first goal
% format: data = shapeRectangleByCorners(grid, lower, upper)
shape1 = shapeRectangleByCorners(g,[-5,-1],[-4,1]);

% second goal (same format)
shape2 = shapeRectangleByCorners(g,[4,-1],[5,1]);

% combine goals
data0=shapeUnion(shape1,shape2);

%% Obstacle
if obstacle
    % create obstacle
    % format: data = shapeSphere(grid, center, radius)
    HJIextraArgs.obstacleFunction = shapeSphere(g, [-4 -2], 1);
end

%% time vector

t0 = 0;
tMax = 20; % max time horizon
dt = 0.05;
tau = t0:dt:tMax;

%% Pack problem parameters

% Define dynamic system
% format: obj = KinVehicleND(x, vMax)
obj = KinVehicleND(xinit, 1);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = obj;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = 'min'; % control wants to minimize distance to goal


%% Compute value function
if strcmp(visualize,'function') || strcmp(visualize,'set')
    HJIextraArgs.visualize.figNum = 1; %set figure number
    HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
    HJIextraArgs.visualize.xTitle = 'x';
    HJIextraArgs.visualize.yTitle = 'y';
    HJIextraArgs.visualize.fontSize = 15;
    HJIextraArgs.visualize.lineWidth = 3;
    
    
    if strcmp(visualize,'function')
        % visualize functions over 2D set
        HJIextraArgs.visualize.valueFunction = 1;
        HJIextraArgs.visualize.initialValueFunction = 1;       
        if obstacle
            HJIextraArgs.visualize.obstacleFunction = 1;
        end
        
        % set axis limits
        HJIextraArgs.visualize.viewAxis = ...
            [g.min(1) g.max(1) g.min(2) g.max(2) -2 2];
    end
    
    if strcmp(visualize,'set2D')
        
        % visualize sets
        HJIextraArgs.visualize.valueSet = 1;
        HJIextraArgs.visualize.initialValueSet = 1;
        if obstacle
            HJIextraArgs.visualize.obstacleSet = 1;
        end
        
        HJIextraArgs.visualize.viewAxis = ...
            [g.min(1) g.max(1) g.min(2) g.max(2)];
    end
    
    if video
        HJIextraArgs.makeVideo = 1;
    end
end

HJIextraArgs.stopInit = xinit;

% Compute reachability analysis
% format: [data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, compMethod, extraArgs)
[data, tau, extraOuts] = ...
    HJIPDE_solve(data0, tau, schemeData, 'minVOverTime', HJIextraArgs);

%% Show trajectory
if strcmp(visualize,'none')
    % don't show simulation if you're not visualizing
else
    if simulation
        %check if this initial state is in the BRS/BRT
        %value = eval_u(g, data, x)
        data = flip(data,3); % flip time to go forward
        value = eval_u(g,data(:,:,1),xinit);
        
        disp(['the value at state [' num2str(xinit(1)) ','...
            num2str(xinit(2)) '] is ' num2str(value)])
        
        if strcmp(visualize,'function')
            figure(2)
            clf
            hL = visSetIm(g, data0, 'g');
            hold on
            hV = visSetIm(g, data(:,:,1), 'b');
            hO = visSetIm(g ,HJIextraArgs.obstacleFunction, 'r');
            
            hL.LineWidth = 3;
            hV.LineWidth = 3;
            hO.LineWidth = 3;
            
            xlabel('$x$','interpreter','latex');
            ylabel('$y$','interpreter','latex');
            set(gca,'FontSize',15)
            grid on
            set(gcf,'Color','w');
            axis square
            
        elseif strcmp(visualize,'set')
            figure(1)
            hold on
        end
        
        if video
            videoFilename = ...
                [datestr(now,'YYYYMMDD_hhmmss') '.mp4'];
            vout = VideoWriter(videoFilename,'MPEG-4');
            vout.Quality = 100;
            vout.FrameRate = 30;
            vout.open;
        end
        
        if value <= 0 %if initial state is in BRS/BRT
            % find optimal trajectory
            
            obj.x = xinit; %set initial state of the dubins car
            
            tEarliest = 1;
            tauLength = size(tau,2);
            iter = 1;
            
            while iter <= tauLength
                % Determine the earliest time that the current state is in the reachable set
                % Binary search
                upper = tauLength;
                lower = tEarliest;
                
                tEarliest = find_earliest_BRS_ind(g, data, obj.x, upper, lower);
                
                % BRS at current time
                BRS_at_t = data(:,:, tEarliest);
                
                % Visualize BRS corresponding to current trajectory point
                s = scatter(obj.x(1), obj.x(2));
                s.SizeData = 70;
                s.MarkerEdgeColor = 'k';
                s.LineWidth = 2;
                hold on
                if iter > 1
                    delete(h)
                end
                
                h = visSetIm(g,BRS_at_t);
                h.Color = 'b';
                tStr = sprintf('t = %.3f; tEarliest = %.3f', tau(iter), tau(tEarliest));
                title(tStr)
                
                if tEarliest == tauLength
                    % Trajectory has entered the target
                    break
                end
                
                % Update trajectory
                Deriv = computeGradients(g, BRS_at_t);
                deriv = eval_u(g, Deriv, obj.x);
                u = obj.optCtrl(tau(tEarliest), obj.x, deriv, 'min');
                obj.updateState(u, dt, obj.x);
                
                % Record new point on nominal trajectory
                iter = iter + 1;
                if video
                    current_frame = getframe(gcf); %gca does just the plot
                    writeVideo(vout,current_frame);
                else
                    pause
                end
            end
        else
            error("initial state isn't in reachable set!")
        end
        
        if video
            vout.close
        end
    end
end
end