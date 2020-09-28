function reachability_visualizations_dubins()

% visualize options: 'function','set2D', 'set3D','none'
visualize = 'function';
video = 0;
obstacle = 1;
simulation = 1;

% initial state for online simulation
xinit = [-2, -1, pi];

%% Grid
grid_min = [-5; -5; -pi]; % Lower corner of computation domain
grid_max = [5; 5; pi];    % Upper corner of computation domain
N = [81; 81; 51];         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N,3);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set

% first goal
% format: data = shapeRectangleByCorners(grid, lower, upper)
shape1 = shapeRectangleByCorners(g,[-5,-1,-inf],[-4,1,inf]);

% second goal (same format)
shape2 = shapeRectangleByCorners(g,[4,-1,-inf],[5,1,inf]);

% combine goals
data0=shapeUnion(shape1,shape2);

%% Obstacle
if obstacle
    % create obstacle
    % format: data = shapeCylinder(grid, ignoreDims, center, radius)
    HJIextraArgs.obstacleFunction = shapeCylinder(g,3,[-4 -2 0], 1);
end

%% time vector

t0 = 0;
tMax = 20; % time horizon
dt = 0.05;
tau = t0:dt:tMax;

%% Pack problem parameters

% Define dynamic system
% format: obj = DubinsCar(x, uRange, speed, dRange)
obj = DubinsCar([0,0,0], [-1,1], 1, [-.1, .1,0]);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = obj;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = 'min'; % control wants to minimize distance to goal


%% Compute value function
if strcmp(visualize,'function') ||...
        strcmp(visualize,'set2D') ||...
        strcmp(visualize,'set3D')
    HJIextraArgs.visualize.figNum = 1; %set figure number
    HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
    HJIextraArgs.visualize.xTitle = 'x';
    HJIextraArgs.visualize.yTitle = 'y';
    HJIextraArgs.visualize.fontSize = 15;
    HJIextraArgs.visualize.lineWidth = 3;
    
    
    if strcmp(visualize,'function')
        % project to 2D
        HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
        HJIextraArgs.visualize.plotData.projpt = pi; %value to project theta
        
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
        % project to 2D
        HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
        HJIextraArgs.visualize.plotData.projpt = pi; %value to project theta
        
        % visualize sets
        HJIextraArgs.visualize.valueSet = 1;
        HJIextraArgs.visualize.initialValueSet = 1;
        
        if obstacle
            HJIextraArgs.visualize.obstacleSet = 1;
        end
        
        HJIextraArgs.visualize.viewAxis = ...
            [g.min(1) g.max(1) g.min(2) g.max(2)];
    end
    
    if strcmp(visualize,'set3D')
        % visualize sets
        HJIextraArgs.visualize.valueSet = 1;
        HJIextraArgs.visualize.initialValueSet = 1;
        
        if obstacle
            HJIextraArgs.visualize.obstacleSet = 1;
        end
        
        % set information for third (theta) axis
        HJIextraArgs.visualize.zTitle = '$\pi$';
        HJIextraArgs.visualize.viewAxis = ...
            [g.min(1) g.max(1) g.min(2) g.max(2) g.min(3) g.max(3)];
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

if simulation
        %check if this initial state is in the BRS/BRT
    %value = eval_u(g, data, x)
    data = flip(data,4); % flip time to go forward
    value = eval_u(g,data(:,:,:,1),xinit);
  
    disp(['the value at state [' num2str(xinit(1)) ','...
        num2str(xinit(2)) ',' num2str(xinit(3)) '] is ' num2str(value)])
    
    if strcmp(visualize,'function') ||...
        strcmp(visualize,'set3D')
    figure(2)
    clf
    [g2D, data02D] = proj(g,data0,[0 0 1],pi);
    [~, obs2D] = proj(g,HJIextraArgs.obstacleFunction,[0 0 1],pi);
    [~, data2D] = proj(g,data(:,:,:,1),[0 0 1],pi);
    hL = visSetIm(g2D, data02D, 'g');
    hold on
    hV = visSetIm(g2D, data2D, 'b');
    hO = visSetIm(g2D ,obs2D, 'r');
    
    hL.LineWidth = 3;
    hV.LineWidth = 3;
    hO.LineWidth = 3;
    
    xlabel('$x$','interpreter','latex');
    ylabel('$y$','interpreter','latex');
    set(gca,'FontSize',15)
    grid on
    set(gcf,'Color','w');
    axis square

    elseif strcmp(visualize,'set2D')
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
            BRS_at_t = data(:,:,:, tEarliest);
            
            % Visualize BRS corresponding to current trajectory point
            s = scatter(obj.x(1), obj.x(2));
            s.SizeData = 70;
            s.MarkerEdgeColor = 'k';
            s.LineWidth = 2;
            hold on
            if iter > 1
                delete(h)
            end
            [g2D, data2D] = proj(g,BRS_at_t,[0 0 1],obj.x(3));
            h = visSetIm(g2D,data2D);
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