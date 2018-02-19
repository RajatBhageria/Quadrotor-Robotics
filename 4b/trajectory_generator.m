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
persistent map;
persistent path; 
if isempty(varargin) == false
    map = varargin{1};
    path = varargin{2};
    path = [zeros(1,3); path];
end 

[n,~] = size(path); 
desired_state = [];

%% Find the maxT 
maxSpeed = .1; %m/s
totalDist = 0; 

%% find all the vectors and maxtime
vectors = zeros(n-1,3); 
for i = 1:n-1
    waypoint1 = path(i,:);
    waypoint2 = path(i+1,:); 
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
    init = path(index,:)';
    pos=(vector*theta(t-timeSoFar,cs))' + init; 
    vel=(vector*omega(t-timeSoFar,cs))'; 
    acc=(vector*alpha(t-timeSoFar,cs))';
else
    pos=path(n,:);
    vel=[0;0;0]; 
    acc=[0;0;0];
end

yaw = 0;
yawdot = 0;

% use the "persistent" keyword to keep your trajectory around inbetween function calls

%vectors for each of the segments 

%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

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
    timeToSpendOnSegment = maxT*((norm(vector)+0.0)/totalDist);
    totalTimeSoFar = totalTimeSoFar + timeToSpendOnSegment; 
    if t <= totalTimeSoFar
        break; 
    end 
end 
cs = findCS(timeToSpendOnSegment,0,1); 
totalTimeSoFar = totalTimeSoFar - timeToSpendOnSegment; 
end 