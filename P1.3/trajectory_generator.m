function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path0 into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path0) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path0
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path00
% map0 = map;
% path00 = path0;

persistent map0; 
persistent path0; 

if (nargin == 4)
    map0 = map; 
    path0 = path;      
    pos=[0;0;0];
    vel=[0;0;0]; 
    acc=[0;0;0];
    
elseif (nargin ==2)
    currPath = path0{qn};
    [n,~] = size(currPath); 

    %% Find the maxT 
    maxSpeed = .55; %m/s
    totalDist = 0; 

    %% find all the vectors and maxtime
    vectors = zeros(n-1,3); 
    for i = 1:n-1
        waypoint1 = currPath(i,:);
        waypoint2 = currPath(i+1,:); 
        vector = [waypoint2(1) - waypoint1(1), waypoint2(2) - waypoint1(2), waypoint2(3) - waypoint1(3)];
        vectors(i,:) = vector; 
        distOfVector = norm(vector);
        totalDist = totalDist + distOfVector; 
    end

    maxT = totalDist * (1/maxSpeed); 

    %% Find the vector for this timestamp t 
    [vector,index,timeSoFar,cs] = findVector(vectors, maxT, totalDist,t); 

    %% Starting point
    if (t==0)
        pos=[0;0;0];
        vel=[0;0;0];
        acc=[0;0;0];
    elseif (t < maxT) 
        init = currPath(index,:)';
        pos=(vector*theta(t-timeSoFar,cs))' + init; 
        vel=(vector*omega(t-timeSoFar,cs))'; 
        acc=(vector*alpha(t-timeSoFar,cs))';
    else
        pos=currPath(n,:);
        vel=[0;0;0]; 
        acc=[0;0;0];
    end
end 

desired_state = [];

yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end

function [val] = theta(t,cs)
val = dot(cs, [1, t, t^2, t^3]'); 
end

function [val] = omega(t,cs)
val = dot(cs, [0, 1, 2*t, 3*t^2]'); 
end

function [val] = alpha(t,cs)
val = dot(cs, [0, 0, 2, 6*t]'); 
end 

function [cs] = findCS(T, a, b)
cs = [1, 0, 0, 0; 1, T, T^2, T^3;0, 1, 0, 0; 0, 1, 2*T, 3*T^2]\[a, b, 0, 0]';
end 

function [vector,index,totalTimeSoFar,cs] = findVector(vectors,maxT,totalDist,t)
totalTimeSoFar = 0; 
vector = vectors(1,:);
timeToSpendOnSegment = 0;
index = 1; 
n = size(vectors,1); 

for i = 1:n
    vector = vectors(i,:); 
    index = i;
    timeToSpendOnSegment = maxT*((norm(vector))/totalDist);
    totalTimeSoFar = totalTimeSoFar + timeToSpendOnSegment; 
    if t <= totalTimeSoFar
        break; 
    end 
end 
cs = findCS(timeToSpendOnSegment,0,1); 
totalTimeSoFar = totalTimeSoFar - timeToSpendOnSegment; 
end 







