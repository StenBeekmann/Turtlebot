clear all 
close all 
clc

%% MILESTONE 2. Ziegler Nicholson 
%% 1. Angular velocity
%% 1.1 Function definition 
s = tf('s');
tf1 = (1.56*exp(-0.1*s)/(0.15*s+1));
Ku1 = 1.94; 
sys_cl_1 = feedback(Ku1*tf1,1); 
P1 = 0.33;
%% 1.2 Step responses
figure
step(sys_cl_1,10); % For the first 100 seconds 
grid on; 
figure
step(tf1)

%% 2. Linear velocity
%% 2.1 Function definition 
% tf2 = (1*exp(-0.365*s)/(0.265*s+1));
tf2 = (1*exp(-0.2*s)/(0.18*s+1));
Ku2 = 2.11; 
sys_cl_2 = feedback(Ku2*tf2,1); 
P2 = 0.613;
%% 2.2 Step response
figure
step(tf2,10)
figure
step(sys_cl_2,10); % For the first 100 seconds 
grid on; 