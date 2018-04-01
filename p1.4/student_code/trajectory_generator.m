function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%

desired_state = [];
persistent traj_segments_x traj_segments_y traj_segments_z
persistent total_time time_fractions


% use the "persistent" keyword to keep your trajectory around
% inbetween function calls
if isempty(varargin) == false
    time_fractions = 0;
    traj_segments_x = [];
    traj_segments_y = [];
    traj_segments_z = [];
    % compute trajectories
    path = varargin{2};
    [n, ~] = size(path);
    
    segments = n-1;
    path_differences = diff(path);
    
    segment_lengths = sqrt(sum(path_differences.^2, 2))';
%     segment_lengths = vecnorm(path_differences');
    total_path_length = sum(segment_lengths);
    total_time = total_path_length / 0.2;
    time_fractions = segment_lengths / total_path_length;
    
    
    for i = 1:segments
        time_fraction = time_fractions(i);
        traj_x = gen_min_jerk(time_fraction*total_time, path(i, 1), path(i+1, 1), 0, 0, 0, 0);
        traj_y = gen_min_jerk(time_fraction*total_time, path(i, 2), path(i+1, 2), 0, 0, 0, 0);
        traj_z = gen_min_jerk(time_fraction*total_time, path(i, 3), path(i+1, 3), 0, 0, 0, 0);
        
        traj_segments_x = [traj_segments_x, traj_x];
        traj_segments_y = [traj_segments_y, traj_y];
        traj_segments_z = [traj_segments_z, traj_z];
    end
else
    
    idx = find(cumsum(time_fractions * total_time) < t, 1, 'last');
    
    if t < total_time
        if isempty(idx)
            segment = 1;
            t0 = 0;
        else
            segment = idx + 1;
            times = cumsum(time_fractions * total_time);
            t0 = times(idx);
        end

        [x, v, a] = eval_poly(t - t0, traj_segments_x(:, segment), traj_segments_y(:, segment), traj_segments_z(:, segment));

        desired_state.pos = x;
        desired_state.vel = v;
        desired_state.acc = a;
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
        times = cumsum(time_fractions * total_time);
        [x, v, a] = eval_poly(total_time - times(end-1), traj_segments_x(:, end), traj_segments_y(:, end), traj_segments_z(:, end));
        desired_state.pos = x;
        desired_state.vel = v;
        desired_state.acc = a;
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
        
end

%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%



end

function [C] = gen_min_jerk(t, x0, xf, v0, vf, a0, af)
    A = [0 0 0 0 0 1;
        t^5 t^4 t^3 t^2 t 1;
        0 0 0 0 1 0;
        5*t^4 4*t^3 3*t^2 2*t 1 0;
        0 0 0 2 0 0;
        20*t^3 12*t^2 6*t 2 0 0];

    b = [x0, xf, v0, vf, a0, af]';

    C = A\b;
end

function [x, v, a] = eval_poly(t, C_x, C_y, C_z)
    T = [t^5, t^4, t^3, t^2, t, 1];
    x = [T*C_x; T*C_y; T*C_z];
    
    dT = [5*t^4 4*t^3 3*t^2 2*t 1 0];
    v = [dT*C_x; dT*C_y; dT*C_z];
    
    ddT = [20*t^3 12*t^2 6*t 2 0 0];
    a = [ddT*C_x; ddT*C_y; ddT*C_z];
end
