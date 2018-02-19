function runsim(wp_num)
% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** MEAM 620 QUADROTOR SIMULATION *****************
close all
clear functions
clear global
clearvars -except wp_num
%clear all

addpath('utils')

% You need to implement trajhandle and controlhandle

% trajectory generator
trajhandle = @trajectory_generator;

% controller
controlhandle = @controller;

% the list of test paths to fly
all_test_paths = {
    [0, 0, 1.0;
     0, 1.0, 0.5;
     1.0, 0.0, 0.5;],
    
    [0.5, 0., 0.5;
     1., 1., 1.0;
     -5, -2, 0.5;],

    
    [0,     0, 0.25;
     0,     0, 1.25;
     0.1, 0.1, 1.25;
     0.2, 0.2, 1.25;
     0.3, 0.3, 1.25;
     0.4, 0.4, 1.25;
     0.5, 0.5, 1.25;
     0.6, 0.6, 1.25;
     0.7, 0.7, 1.25;
     0.8, 0.8, 1.25;
     0.9, 0.9, 1.25;
     1.0, 1.0, 1.25;],

    [0, 0, 0.5;
     0, 0, 1.5;
     0, 3, 1.5;
     3, 3, 1.5;
     3, 0, 1.5;
     0, 0, 1.5;
     0, 0, 0.6;],
                 };
if nargin < 1
    error('must give the waypoint test number as argument!')
end
if wp_num <=0 | wp_num > length(all_test_paths)
    error('must specify waypoint test number between 1 and %d', ...
          length(all_test_paths))
end    
waypts = all_test_paths{wp_num};


% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********
% real-time 
real_time = true;
% number of quadrotors
nquad = 1;

% max time
time_tol = 30;

% parameters for simulation
params = crazyflie();

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors

for qn = 1:nquad
    desired_state = trajhandle(0, qn, [], waypts);
    % Get start and stop position
    des_start = trajhandle(0, qn);
    disp(des_start);
    des_stop  = trajhandle(inf, qn);
    stop{qn}  = des_stop.pos;
    x0{qn}    = init_state( des_start.pos, 0 );
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep;

    tic;
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
            desired_state = trajhandle(time, qn);
            %            desired_state.pos
            QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
            h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
        end

        % Run simulation
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn}    = xsave(end, :)';
        
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

        % Update quad plot
        desired_state = trajhandle(time + cstep, qn);
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
        set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    end
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

% Plot the saved position and velocity of each robot

for qn = 1:nquad
    % Truncate saved variables
    QP{qn}.TruncateHist();
    % Plot position for each quad
    h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
    plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
    plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
    % Plot velocity for each quad
    h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
    plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
    plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, ...
               'vel', 'des');
    figure('Name', ['Tracking']);
    pos      = QP{qn}.state_hist(1:3,:); % actual position
    pos_des = QP{qn}.state_des_hist(1:3,:); % student desired
    scatter3(pos_des(1,:),pos_des(2,:),pos_des(3,:),'bx');
    hold on;
    scatter3(pos(1,:),pos(2,:),pos(3,:),'rx');
    scatter3(waypts(:,1), waypts(:,2),waypts(:,3), 50, 'MarkerFaceColor', 'g');
    legend('generated trajectory', 'flown', 'waypoints');
    hold off;
end

if(~isempty(err))
    error(err);
end

fprintf('finished.\n')
end
