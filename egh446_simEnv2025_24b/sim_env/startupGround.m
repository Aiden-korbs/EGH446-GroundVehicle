%% close previously open model
close_system('sl_groundvehicleDynamics',0);
clear all;
 
% generate random waypoints
unordered_waypoints.x = randi([-500 500],1,10);
unordered_waypoints.y = randi([-500 500],1,10);

% order waypoints with order_waypoints method
[waypoints.x, waypoints.y] = order_waypoints(unordered_waypoints.x, unordered_waypoints.y);

%% create optimised route for generated waypoints
function [ordered_wp_x, ordered_wp_y] = order_waypoints(wp_x, wp_y)
    % organise the waypoints in the way the function expects
    waypoints = [wp_x(:), wp_y(:)];
    % add robot origin to the start of the waypoint list
    waypoints = [0, 0; waypoints];
    num_waypoints = size(waypoints, 1);

    % set up waypoint index array and empty route array
    waypoint_idx = 1:num_waypoints;
    route = zeros(1, num_waypoints);
    
    % set the starting point as the first waypoint in the list
    route(1) = 1;
    waypoint_idx(1) = [];

    % nearest neighbour
    % nearest neighbour loop
    for i = 1:num_waypoints-1
        % compute distances to all remaining waypoints
        current_waypoint = waypoints(route(i), :);
        distances = zeros(1, length(waypoint_idx));
        for j = 1:length(waypoint_idx)
            next_potential_point = waypoints(waypoint_idx(j), :);
            distances(j) = pdist2(current_waypoint, next_potential_point);
        end
        % select closest point to travel to next
        [~, min_idx] = min(distances);
        route(i+1) = waypoint_idx(min_idx);
        waypoint_idx(min_idx) = [];
    
    end

    % 2-opt swap
    % optimise n-n generated route with 2-opt swap
    improved = true;
    tolerance = 1e-12;  % tolerance set to avoid floating point errors
    
    % helper method for computing distance
    get_dist = @(a,b) pdist2(waypoints(a, :), waypoints(b, :));
    
    % compute original length of n-n generated route
    len_before = 0;
    for i=1:num_waypoints-1
        len_before = len_before + pdist2(waypoints(route(i), :), waypoints(route(i+1), :));
    end
    %disp("Total length of n-n generated route: " + len_before);
    
    num_iterations = 0;
    while improved
        num_iterations = num_iterations + 1;
        improved = false;
        for i = 1:num_waypoints-3
            a = route(i);
            b = route(i+1);
            for j = i+2:num_waypoints-1
                c = route(j);
                d = route(j+1);
                
                % swap if it makes the route shorter
                delta = (get_dist(a,c) + get_dist(b,d)) - (get_dist(a,b) + get_dist(c,d));
                if delta < -tolerance
                    route(i+1:j) = route(j:-1:i+1);
                    improved = true;
                end
            end
        end
    end
    
    len_after = 0;
    for i=1:num_waypoints-1
        len_after = len_after + pdist2(waypoints(route(i), :), waypoints(route(i+1), :));
    end
    %disp("Total length of 2-opt swap optimised route: " + len_after);
    
    %disp("Total number of 2-opt swap iterations: " + num_iterations);

    % return the ordered waypoint vectors
    ordered_wp_x = waypoints(route, 1);
    ordered_wp_y = waypoints(route, 2);

end

%% add toolboxes to path
homedir = pwd; 
addpath( genpath(strcat(homedir,[filesep,'toolboxes'])));

cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;

cd(homedir);

%% open current model
open_system('sl_groundvehicleDynamics'); %differential robot

cd(homedir);





