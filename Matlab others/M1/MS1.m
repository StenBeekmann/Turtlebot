%% RVC example
% Example file to drive a robot with a constant speed and steering angle,
% record the measured and the true angle and plot the results

%% Initialization
clear all;
close all;

startup_rvc

% Parameters
v = 1; % forward velocity
gamma = 1.5; % steer angle
n = 100; % Number of iterations to simulate
dt = 0.1; % Timsteps

% Vehicle object
X = [0; 0; 0];
Q=diag([0.005,0.005].^2); % noise on travelled distance[m] and the heading angle [rad]
vehicle = Bicycle('covar',Q,'x0',X, 'accelmax',1, 'speedmax',1, 'steermax',1, 'dt',dt);
vehicle.V_IMU = 0.05^2; % Noise on gyroscope measurements

% Initialize collections
time = zeros(1,n); % Timestamps
% verplaatsing = zeros(1,n); % Measurements returned by gyroscope
% hoekverplaatsing = zeros(1,n); % Actual angle of robot

% Run simulation
for i=2:n 
odom = vehicle.step(v, gamma); % Simulate 1 timestep and return odometry
time(i) = time(i-1) + dt; % Get timestep

verplaatsing(i) = odom(1); % Get measured robot orientation
hoekverplaatsing(i) = odom(2); % Get real orienatation

end

%Plots
dv= verplaatsing/dt;
figure()
plot(time,dv)
xlabel("time [s]")
ylabel("velocity [m/s]");

dw=hoekverplaatsing/dt;
figure
plot(time,dw);
xlabel("time [s]")
ylabel("angular velocity [rad/s]");


