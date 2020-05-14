clear all
close all 
clc

%% MILESTONE 2: VEHICLE CONTROL
%% MILESTONE 2.1: SISO Controllers
%% 1. Identify model based on simulated response to step input 
%% 1.1 Select the parameters for the step response and the vehicle
% Initialization
startup_rvc
s = tf('s');

% Selected rvc paramerter 
v = 1.0;                   % Forward velocity
gamma = 0.5;               % Steer angle
n_max = 500;                % Number of observations
dt = 0.1;                   % Time intervam

%% 1.2 Creating a robot instance - Vehicle  
% Vehicle object
X = [1; 2; 0.5];
Q = diag([0.005,0.005].^2);  % noise on travelled distance[m] and the heading angle [rad]
vehicle = Bicycle('covar', Q,'x0',X);% 'accelmax',1, 'speedmax',1, 'steermax',1, 'dt',dt);

% Initialize collections
time = zeros(1,n_max);       % Timestamps
pos_o = zeros(1,n_max);         % Actual angle of robot

% Define the range where the vehicle will moves (x,y)
figure('Name','The vehicle movement')
xlim([-40,40])                               % X limitations [m]
ylim([-40,40])                               % Y limitations [m]

%% 2. Defining the PID controller (Ziegler-Nichols)
%% 2.1 Defining the PID controllers ZN2
%% 2.1.1 Angular velocity PID controller
tf1 = (1.56*exp(-0.15*s)/(0.48*s+1));
Ku1 = -3.65; 
P1 = 0.51; 
% Assuming we use the PID controller

%% 2.2 Tuning the Controller
% Kp1 = 0.6*Ku1 % -2.19
Kp1 = -0.5;
Ti1 = 0.5*P1; 
Td1 = 0.125*P1; 
% Ki1 = Kp1/Ti1 % -8.59
Ki1 = -5;
% Kd1 = Td1*Kp1 %-0.14
Kd1 = -1;

%% 3. Forcing the vehicle to move in a straight line
%% 3.1 Initial conditions 
% The car needs to follow the line at different angles. 
% Line equation is defined by the user with a, b and c parameters.
% Line parameters a*pos_x(i) + b*pos_y(i) + c == 0;
a = 1; b = 1; c = 5;
    
%% 3.2 Simulation parameters 
% Setting the values of error for I and D actions
error_d = 0;
error_i = 0;

%% 3.3 Creating loop for vehicle movement
for i = 2:n_max
    
    odom = vehicle.step(v, gamma);      % Simulate with initial velocity and angle

    % Extract y, x and theta positions from car real position
    pos_x = vehicle.x(1);
    pos_y = vehicle.x(2);
    pos_o = vehicle.x(3);

    % Create two vectors that define the movement of the vehicle and find
    % error
    vector1 = [a b c];
    vector2 = [pos_x pos_y 1];
    error = dot(vector1,vector2)/sqrt(a^2+b^2);    
    
    % Find the error on the integral
    error_avg = (error + error_d)/2;
    error_i = -(dt * error_avg + error_i);
    
    % Implement the controller
    gamma = Kp1*(error) + Ki1*error_i + Kd1*(error-error_d)/dt; 
    
    % REcord the last error 
    error_d = error;
    
    % Plot results
    vehicle.plot();
    hold on
    x = 0:1:40;
    y = (-c - a*x)/b;
    plot(x,y,'color','y');
end

%% 4. Plotting the results
%% 4.1 Plotting the vehicle path 
figure
vehicle.plot_xy()
hold on 

%% 4.2 Plotting the trajectory 
x = 0:1:40;
y = (-c - a*x)/b;
plot(x,y,'color','y');
xlim([-20 20])
ylim([-20 20])
legend('Path','Trajectory');
title('Desired versus Followed Path of Vehicle')
