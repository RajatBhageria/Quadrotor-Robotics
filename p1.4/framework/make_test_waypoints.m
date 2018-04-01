function [waypts] = make_test_waypoints
%
% [waypts] = make_test_waypoints
%
% returns cell array of size 3. Each cell contains
% a sequence of waypoints, which are stored in
% a nx3 matrix. Each row contains one waypoint;

waypts{1} = [[-1:(1/10):1]', [-1:(1/10):1]', 1.2*ones(21,1)];
waypts{2} = [-1.2, -1.2, 1;
           0.2, -0.9, 1;
           0.2, -0.9, 2;
           0.5,  0.5, 2;
           0.5,  0.5, 1;
           1.7,  1.2, 1];
waypts{3} = [0, 0, 1;
            .25, -.35, 1.5;
            .5, 0, 2;
            .75, .35, 1.5;
            1, 0, 1];
end