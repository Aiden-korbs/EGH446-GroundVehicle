%% script setup
close all; clear all; clc;

% randomised waypoint list
num_waypoints = 10;
waypoints = randi([-500 500], num_waypoints, 2);

% set up waypoint index array and empty route array
waypoint_idx = 1:num_waypoints;
route = zeros(1, num_waypoints);

% set the starting point as the first waypoint in the list
route(1) = 1;
waypoint_idx(1) = [];

%% nearest neighbour
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
% make list for n-n route
n_n_route = route;

%% 2-opt swap
% optimise n-n generated route with 2-opt swap
improved = true;
tolerance = 1e-12;

% helper method for computing distance
get_dist = @(a,b) pdist2(waypoints(a, :), waypoints(b, :));

% compute original length of n-n generated route
len_before = 0;
for i=1:num_waypoints-1
    len_before = len_before + pdist2(waypoints(route(i), :), waypoints(route(i+1), :));
end
disp("Total length of n-n generated route: " + len_before);

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
disp("Total length of 2-opt swap optimised route: " + len_after);

disp("Total number of 2-opt swap iterations: " + num_iterations);

%% plotting
% plot optimised route
figure();
hold on;
plot(waypoints(route, 1), waypoints(route, 2), 'b--');
plot(waypoints(route, 1), waypoints(route, 2), 'r*');
p_route(waypoints, 1:num_waypoints, num_waypoints);
title('Optimised N-N with 2-opt swap route');

% plot unoptimised route
figure();
hold on;
plot(waypoints(n_n_route, 1), waypoints(n_n_route, 2), 'b--');
plot(waypoints(n_n_route, 1), waypoints(n_n_route, 2), 'r*');
p_route(waypoints, 1:num_waypoints, num_waypoints);
title('Un-optimised N-N route');

% plot randomly generated waypoint list
figure();
hold on;
plot(waypoints(:, 1), waypoints(:, 2), 'b--');
plot(waypoints(:, 1), waypoints(:, 2), 'r*');
p_route(waypoints, 1:num_waypoints, num_waypoints);
title('Randomly generated waypoint route');

% function for drawing text at each point
function p_route(route_array, route, num_waypoints)
    for i = 1:num_waypoints
        text(route_array(route(i), 1),route_array(route(i), 2),num2str(route(i)))
    end
end