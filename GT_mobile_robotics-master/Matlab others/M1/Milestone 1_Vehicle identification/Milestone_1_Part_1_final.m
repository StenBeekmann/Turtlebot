%% MILESTONE 1: VEHICLE MODEL IDENTIFICATION

clear all 
close all 
clc

%% 1. Identify model based on simulated response to step input 
%% 1.1 Select the parameters for the step response and the vehicle
% Initialization
startup_rvc

% Selected rvc parameters
v = 1.0;                                    % Forward velocity [m/s] - used as an input step 
gamma = 0.0;                                % Steer angle [rad] - the car moves in a straight line when steer angle is set to zero
% gamma = 1.0; When step input for angular velocity is needed 
n = 100;                                    % Number of observations [] 
dt = 0.1;                                   % Time interval [s] - defines how fast the values are updated
                                            
%% 1.2 Creating a robot instance - Vehicle  
% Vehicle object                            
X = [0; 0; 0];                              % Initial position for the start(x,y,theta) [m,m,rad]
Q = diag([0.005,0.005].^2);                 % Odemetry noise(distance,angle) [m,rad] - values given by mentor  
vehicle = Bicycle("covar",Q,'x0',X, 'accelmax',1, 'speedmax',4, 'steermax',1, 'dt',dt); % Creating the object, setting the maximum values
vehicle.V_IMU = 0.01;                       % Create noise on gyroscope measurements [rad]

% Initialize collections - creating arrays for the gyroscope, robot and
% time for recording the data in the for-loop 
time = zeros(1,n);                          % Timestamps [s]
theta_gyro = zeros(1,n);                    % Measurements returned by gyroscope [rad]
theta_robot = zeros(1,n);                   % Actual angle of robot [m]

% Define the range where the vehicle will moves (x,y)
figure('Name','The vehicle movement')
xlim([-2,10])                               % X limitations [m]
ylim([-2,5])                                % Y limitations [m]

%% 1.3 Simulate the vehicle movement and record the sensors mesurements 
valueOfD = zeros(1,n);                      % Displacement array [m] - to record linear displacement between two time points
valueOfO = zeros(1,n);                      % Distortion array [rad] - to record distortion between two time points
for i=2:n
    odom = vehicle.step(v, gamma);          % Simulate a step for the vehicle(speed,steer angle) [m/s ,rad]

    theta_gyro(i) = vehicle.get_IMU();      % Get measured robot orientation [rad]
    time(i) = time(i-1) + dt;               % The current time [s] - in relation to the previous time 

    % Plot results and prepare for next loop
    vehicle.plot();
    
    % Inserting the data into the arrays 
    ds = odom(1);                           % Driven distance [m]
    dth = odom(2);                          % Driven distortion [rad]
    valueOfD(i) = ds;                       % Displacement [m]
    valueOfO(i) = dth;                      % Distortion [rad]
end                                         
                                            
%% 2. Display the resulted measured step response 
%% 2.1 Calculate the velocity from the displacement 
v_final = valueOfD/dt;                      % Linear velocity [m/s]
w_final = valueOfO/dt;                      % Angular velocity [rad/s]

%% 2.2 Plot the result
figure('Name','Velocity') 
subplot(2,1,1);
plot(time, v_final)
title('Linear velocity')
xlabel('time [s]')
ylabel ('speed [m/s]')

subplot(2,1,2);
plot(time, w_final)
title('Angular velocity')
xlabel('time [s]')
ylabel ('omega [rad/s]')

