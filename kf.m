%% EGH446 - Kalman Filter Parameters Setup
% This script defines the Kalman filter matrices and parameters for 
% vehicle state estimation in the autonomous systems project
% 
% State vector: [x_pos; y_pos; vx; vy; theta; theta_dot]
% Control input: [desired_velocity; desired_angular_velocity]
% Measurements: [x_measured; y_measured; theta_measured]

clear; clc;

%% System Parameters
dt = 0.1;           % Sample time (seconds) - adjust based on your simulation
n_states = 6;       % Number of states
n_inputs = 2;       % Number of control inputs
n_measurements = 3; % Number of measurements

%% State Transition Matrix (A)
% Discrete-time model for vehicle dynamics
A = [1  0  dt  0  0  0;     % x_pos = x_pos + vx*dt
     0  1  0   dt 0  0;     % y_pos = y_pos + vy*dt  
     0  0  1   0  0  0;     % vx (constant velocity model)
     0  0  0   1  0  0;     % vy (constant velocity model)
     0  0  0   0  1  dt;    % theta = theta + theta_dot*dt
     0  0  0   0  0  1];    % theta_dot (constant angular velocity)

%% Control Input Matrix (B)
% Maps control inputs to state changes
B = [0  0;      % position not directly controlled
     0  0;      % position not directly controlled
     1  0;      % vx controlled by desired velocity
     0  0;      % vy assumed zero for forward motion
     0  0;      % heading not directly controlled
     0  1];     % theta_dot controlled by desired angular velocity

%% Measurement Matrix (C)
% Maps states to measurements
C = [1  0  0  0  0  0;      % measure x_pos (GPS)
     0  1  0  0  0  0;      % measure y_pos (GPS)
     0  0  0  0  1  0];     % measure theta (IMU/compass)

%% Process Noise Covariance Matrix (Q)
% Represents uncertainty in the system model
% Tune these values based on your vehicle dynamics confidence
Q_pos = 0.01;       % Position process noise (m^2)
Q_vel = 0.1;        % Velocity process noise ((m/s)^2)
Q_heading = 0.005;  % Heading process noise (rad^2)
Q_angular = 0.01;   % Angular velocity process noise ((rad/s)^2)

Q = diag([Q_pos, Q_pos, Q_vel, Q_vel, Q_heading, Q_angular]);

%% Measurement Noise Covariance Matrix (R)
% Based on project specifications
R_pos = 0.1;                    % Position measurement noise (m^2) - from spec
R_heading = (2 * pi/180)^2;     % Heading measurement noise (rad^2) - 2° from spec

R = diag([R_pos, R_pos, R_heading]);

%% Initial Conditions
% Initial state estimate (adjust based on known starting conditions)
x0 = [0;    % Initial x position (m)
      0;    % Initial y position (m)
      0;    % Initial x velocity (m/s)
      0;    % Initial y velocity (m/s)
      0;    % Initial heading (rad)
      0];   % Initial angular velocity (rad/s)

% Initial state covariance matrix
% Represents uncertainty in initial state estimate
P0 = diag([1,      % Position uncertainty (m^2)
           1,      % Position uncertainty (m^2)
           1,      % Velocity uncertainty ((m/s)^2)
           1,      % Velocity uncertainty ((m/s)^2)
           0.1,    % Heading uncertainty (rad^2)
           0.1]);  % Angular velocity uncertainty ((rad/s)^2)

%% Display System Information
fprintf('=== Kalman Filter Setup Complete ===\n');
fprintf('Sample time: %.2f seconds\n', dt);
fprintf('State dimension: %d\n', n_states);
fprintf('Input dimension: %d\n', n_inputs);
fprintf('Measurement dimension: %d\n', n_measurements);
fprintf('\nState vector: [x_pos, y_pos, vx, vy, theta, theta_dot]\n');
fprintf('Control input: [desired_velocity, desired_angular_velocity]\n');
fprintf('Measurements: [x_measured, y_measured, theta_measured]\n');

%% Verify System Properties
% Check observability
Obs = obsv(A, C);
if rank(Obs) == n_states
    fprintf('\n✓ System is observable\n');
else
    fprintf('\n✗ WARNING: System is not fully observable\n');
end

% Check controllability
Cont = ctrb(A, B);
if rank(Cont) == n_states
    fprintf('✓ System is controllable\n');
else
    fprintf('✗ WARNING: System is not fully controllable\n');
end

%% Optional: Display matrices for verification
fprintf('\n=== Matrix Dimensions ===\n');
fprintf('A: %dx%d\n', size(A));
fprintf('B: %dx%d\n', size(B));
fprintf('C: %dx%d\n', size(C));
fprintf('Q: %dx%d\n', size(Q));
fprintf('R: %dx%d\n', size(R));

%% Save to workspace
% These variables will be available for your Simulink model
fprintf('\n=== Variables saved to workspace ===\n');
fprintf('A, B, C, Q, R, x0, P0, dt\n');

%% Tuning Guidelines (Comments)
% If filter is too slow to respond to changes:
%   - Decrease Q values (trust model less)
%   - Increase R values (trust measurements more)
%
% If filter output is too noisy:
%   - Increase Q values (trust model more)
%   - Decrease R values (trust measurements less)
%
% Monitor the innovation sequence (measurement - prediction) to assess
% filter performance. Innovations should be zero-mean and white noise.

%% Alternative State Models (Commented Out)
% For more complex vehicle dynamics, consider these alternatives:

% % Bicycle model (more accurate for car-like vehicles)
% % State: [x, y, v, theta, delta] where delta is steering angle
% % A_bicycle = [1  0  dt*cos(theta)  -dt*v*sin(theta)  0;
% %              0  1  dt*sin(theta)   dt*v*cos(theta)  0;
% %              0  0  1               0                 0;
% %              0  0  dt*tan(delta)/L  1                0;
% %              0  0  0               0                 1];

% % Constant acceleration model (if acceleration measurements available)
% % State: [x, y, vx, vy, ax, ay]
% % A_accel = [1  0  dt  0  0.5*dt^2  0;
% %            0  1  0   dt  0        0.5*dt^2;
% %            0  0  1   0   dt       0;
% %            0  0  0   1   0        dt;
% %            0  0  0   0   1        0;
% %            0  0  0   0   0        1];

fprintf('\nReady for Simulink implementation!\n');