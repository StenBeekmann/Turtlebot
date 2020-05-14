%% MILESTONE 1: VEHICLE MODEL IDENTIFICATION
% IDENTIFY MODEL BASED ON PHYSICAL DATA FROM PREVIOUS EXPERIMENTS
clear all 
close all 
clc

%% 1 Data input 1: 0.1 v-step and 0 w-step 
%% A. Load the response of the first data set (0.1 v-step and 0 w-step)
load('v_step_0.1_w_step_0_enc.mat'); 
whos -file v_step_0.1_w_step_0_enc.mat

%% B. Define the velocities
%% B.1 Define the parameters of each wheel 
% Global constants
ticks_to_rad = 0.001533981;                                     % A constant from tick to rad [rad/ticks]
B = 0.16;                                                       % Distance between wheels [m]
R = 0.033;                                                      % Wheel radius [m]

%% B2. Calculation of the velocities
%% B2.1 Find the ticks of each wheel per time interval
for i=1:length(all_enc_left)-1                                  
    time(i) = all_time(i+1)-all_time(i);                        % Time interval [s] - time between to consequentive time points
    vel_left(i)=((all_enc_left(i+1)-all_enc_left(i))*ticks_to_rad)/time(i);    % Angular velocity of the left wheel [m/s] 
    vel_right(i)=((all_enc_right(i+1)-all_enc_right(i))*ticks_to_rad)/time(i); % Angular velocity of the right wheel [m/s]
end

%% B2.2 Calculate the linear and angular velocities
%The rotational velocity of the robot is (w1-w2)/B and the forward velocity is the average between the two linear velocities 
v_calc_left = vel_left*R;                                       % Linear velocity of left wheel [rad*m/s]
v_calc_right = vel_right*R;                                     % Linear velocity of right wheel [rad*m/s]
v_result = (v_calc_right + v_calc_left)/2;                      % Linear velocity of the robot [m/s]
w_result = (v_calc_right - v_calc_left)/B;                      % Angular velocity of the robot [rad/s]

%% C. Plot the response to 0.1 v-step and 0 w-step
figure('Name','The velocitys of the vehicle')                   
subplot(2,1,1)                                                  
plot(all_time(1:length(all_time)-1),w_result)                   
title('Angular Velocity')                                           
xlabel('time [s]')
ylabel ('omega [rad/s]')
grid on                                                         
grid minor                                                      

subplot(2,1,2)                                                             
plot(all_time(1:length(all_time)-1),v_result)                   
title('Forward Velocity')                                       
xlabel('time [s]')
ylabel ('speed [m/s]')
grid on
grid minor                                                      

%% D. Determine the System Parameters from the step response and plot the resaults
%% D.1 Parameters (Concluded from the plot)
delay = 0.119;                                                  % Time delay [s]
K = 0.1005/v_step;                                              % Steady state gain [] - change in output/ change in input 
el_output = (63.2/100)*v_step;                                  % Output of one elapsed time constant [m/s] (= 0.0632 m/s)
t = 0.24;                                                       % Time [s] - at which 63.2% of output is reached
tau = t - delay;                                                % Time constant [s]

%% D.2 Define the transfer function 
s = tf('s');                                                    % Enabling 's' 
G = K*exp(-delay*s)/(tau*s+1);                                  % Transfer function of first order - output/ input
sys = v_step*G;                                                 % Output of system [m/s]                                               

%% D.3 Plotting the resaults
figure('Name','Step response')
step(sys)
title('Approximated Transfer Function for Linear Velocity')
grid on
grid minor
xlabel('time [s]')
ylabel ('speed [m/s]')

%% 2 Data input 2: 0.0 v-step and 2.0 w-step 
%% A. Load the response of the first data set (0.0 v-step and 2.0 w-step)
load('v_step_0_w_step_2_enc.mat'); 
whos -file v_step_0_w_step_2_enc.mat

%% B. Define the velocities
%% B.1 Define the parameters of each wheel 
% Global constants
ticks_to_rad = 0.001533981;                                     % A constant from tick to rad [rad/ticks]
B = 0.16;                                                       % Distance between wheels [m]
R = 0.033;                                                      % Wheel radius [m]

%% B2. Calculation of the velocities
%% B2.1 Find the ticks of each wheel per time interval
for i=1:length(all_enc_left)-1                                  
    time(i) = all_time(i+1)-all_time(i);                        % Time interval [s] - time between to consequentive time points
    vel_left(i)=(all_enc_left(i+1)-all_enc_left(i))/time(i);    % Ticks of the left wheel per time interval [ticks/s] 
    vel_right(i)=(all_enc_right(i+1)-all_enc_right(i))/time(i); % Ticks of the right wheel per time interval [ticks/s] 
end

%% B2.2 Calculate the linear and angular velocities
%The rotational velocity of the robot is (w1-w2)/B and the forward velocity is the average between the two linear velocities 
v_calc_left = vel_left*ticks_to_rad*R;                          % Linear velocity of left wheel [rad*m/s]
v_calc_right = vel_right*ticks_to_rad*R;                        % Linear velocity of right wheel [rad*m/s]
v_result = (v_calc_right + v_calc_left)/2;                      % Linear velocity of the robot [m/s]
w_result = (v_calc_right - v_calc_left)/B;                      % Angular velocity of the robot [rad/s]

%% C. Plot the response to 0.0 v-step and 0.5 w-step
figure('Name','The velocitys of the vehicle')                   
subplot(2,1,1)                                                  
plot(all_time(1:length(all_time)-1),w_result)                   
title('Angular Velocity')                                           
xlabel('time [s]')
ylabel ('omega [rad/s]')
grid on                                                         
grid minor                                                      

subplot(2,1,2)                                                             
plot(all_time(1:length(all_time)-1),v_result)                   
title('Forward Velocity')                                       
xlabel('time [s]')
ylabel ('speed [m/s]')
grid on
grid minor                                                      

%% D. Determine the System Parameters from the step response and plot the resaults
%% D.1 Parameters (Concluded from the plot)
delay = 0.08882;                                                % Time delay [s]
K = 2.027/w_step;                                               % Steady state gain [] - change in output/ change in input 
el_output = (63.2/100)*w_step;                                  % Output of one elapsed time constant [m/s] (= 1.264 m/s)
t = 0.195;                                                      % Time [s] - at which 63.2% of output is reached
tau = t - delay;                                                % Time constant [s]

%% D.2 Define the transfer function 
s = tf('s');                                                    % Enabling 's' 
G = K*exp(-delay*s)/(tau*s+1);                                  % Transfer function of first order - output/ input
sys = w_step*G;                                                 % Output of system [m/s]                                               

%% D.3 Plotting the resaults
figure('Name','Step response')
step(sys)
title('Approximated Transfer Function for Angular Velocity')
grid on
grid minor
xlabel('time [s]')
ylabel ('omega [rad/s]')
