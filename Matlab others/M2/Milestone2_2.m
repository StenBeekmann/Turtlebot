close all
clear all 
clc

%% MILESTONE 2: VEHICLE CONTROL
%% MILESTONE 2.2: MIMO Controllers

%% 1. Identify model based on simulated response to step input 
%% 1.1 Select the parameters for the step response and the vehicle
% Initialization
startup_rvc
s = tf('s');

%% Parameters
v = 5.0;        % forward velocity
gamma = 0.5;    % steer angle
n = 500;        % Number of iterations to simulate
dt = 0.1;       % Timsteps
steermax = 1;

%% 1.2 Creating a robot instance - Vehicle 
% Vehicle object
X = [-0.5;0; 0];
Q=diag([0.005,0.005].^2);  % noise on travelled distance[m] and the heading angle [rad]
vehicle = Bicycle('covar', Q,'x0',X, 'accelmax',1, 'speedmax',2, 'steermax',steermax, 'dt',dt);


%% 2. Defining the PID controller (Ziegler-Nichols)
%% 2.1 Defining the PID controllers ZN2
%% 2.1.1 Angular velocity PID controller
tf1 = (1.56*exp(-0.15*s)/(0.48*s+1));
Ku1 = 3.65; 
P1 = 0.51; 
% Assuming we use the PID controller
Kp1 = 0.6*Ku1;
Ti1 = 0.5*P1; 
Td1 = 0.125*P1; 
% Ki1 = Kp1/Ti1;
Ki1 = 17;
Kd1 = Td1*Kp1;

%% 2.1.2 Linear velocity PID controller
tf2 = (1*exp(-0.365*s)/(0.265*s+1));
Ku2 = 1.8;
P2 = 1.07; 
% Assuming we use the PID controller
Kp2 = 0.6*Ku2;
Ti2 = 0.5*P2; 
Td2 = 0.125*P2; 
Ki2 = Kp2/Ti2;
Kd2 = Td2*Kp2;

%% 3. Forcing the vehicle to move in a trajectory with the set of points
%% 3.1 Initial conditions 
% The car needs to follow the points and then ones it reaches the point it
% moves to another point. This is done by defining the line segments.
trajectory = [0, 0; 0, 10; 10, 10; 0, 10; 0, 0];
counter = 1;             % To define at which location the car is at the moment 

%% 3.2 Simulation parameters 
% Setting the arrays
%% 3.2.1 Parameters for distance
error_distance_deriv = 0;          % Error on the distance, the previous error for derivative action.
error_distance_cumul = 0;          % Error on distance, cumulative for the integral action.
compensation = 1.0;                % Range from the car to target point at which the landmark is still recognized

%% 3.2.2 Parameters for angle
error_angle_deriv = 0;             % Last error on the derivative action
error_angle_cumul = 0;             % Cumulative error on the integral action

%% 3.3 Running the simulation 
%% 3.3.1 Defining the wall 
% The simulation runs the vehicle that has to follow the path defined by
% trajectory points. Define the wall parameters on the figure. 
figure
hold on
xlim([-5,15])
ylim([-5,15])
line([-3 -3], [-3 13],'color','g','LineWidth',2);
line([3 3], [-3 7],'color','g','LineWidth',2);
line([13 13], [13 7],'color','g','LineWidth',2);
line([3 13],[7 7],'color','g','LineWidth',2);
line([-3 13],[13 13],'color','g','LineWidth',2);

%% 3.3.2 Simulating the vehicle movement
for i=2:n
    %% A. Start vehicle movement 
    odom = vehicle.step(v, gamma); % Simulate vehicle movement for timestep and return odometry
    
    %% B. Extracting the coordinates from the real position of the vehicle
    pos_x = vehicle.x(1);              % Extract the x-coordinate 
    pos_y = vehicle.x(2);              % Extract the y-coordinate
    pos_o = vehicle.x(3);              % Extract the angle at which the car is directed
    
    %% C. Looping through the trajectory points with the help of the counter
    % First, check if the car is within the range on the x-axis in relation
    % to the car real position and the next point of the trajectory
    if (trajectory(counter+1,1)-compensation < pos_x) && (pos_x < trajectory(counter+1,1)+compensation) 
    % Second, conduct the same ccomparision for the y-positions
    if   (trajectory(counter+1,2)-compensation < pos_y) && (pos_y < trajectory(counter+1,2)+compensation)
    % Then increase the counter, if the counter reaches the fifth element
    % stop the looping.
       counter = counter + 1;
    if counter == 5
    break
    end
    end
    end
    
    %% D. The PID controller for the steering angle based on the error
    % Given the desired point find the angle based on Matlab atan2
    % function, dy/dx. Implement the controller for each output separately.
    dy = trajectory(counter+1,2) - pos_y; % Expressing the y coordinates as a vector 
    dx = trajectory(counter+1,1) - pos_x; 
    angle_target = atan2(dy,dx);          % Find the angle to the target  
    
    % Find the error between the target angle and the current angle of the
    % vehicle
    error = (angle_target-pos_o);
    % Taking care of correct turning. Always turn in the direction that
    % will be the smallest angle.
    % If the angle is above 180°, like 200 instead of turning 200°, turn
    % 160 etc. 
    if(abs(error) > pi)
        error = error - 2*pi; 
    end
    
    % Find the error for the integral action and add the previous error over the time span dt 
    error_angle_cumul = (dt *((-error + error_angle_deriv)/2) + error_angle_cumul);
    % Implementing the controller, Kp on error, Ki on cumulative error, Kd
    % on derivative error
    gamma = Kp1*error + Ki1*error_angle_cumul + Kd1*(-error + error_angle_deriv)/dt;
    error_angle_deriv = error;            % Record the error to the previous derivative error
    
    %% E. PID controller for the distance (x, y coordinates)
    % First, find the distance between the two consequentive points.
    % Distance to the trajectory point to which the car is heading
    dy_current = pos_x - trajectory(counter,1); 
    dx_current = pos_y - trajectory(counter,2);
    d_current = sqrt(dy_current.^2 + dx_current.^2);
    
    % Distance to the first following trajectory point
    dy_next = pos_x - trajectory(counter + 1,1); 
    dx_next = pos_y - trajectory(counter + 1,2);
    d_next= sqrt(dy_next.^2 + dx_next.^2);

    % We add compensation here to speed up the turning process of the
    % vehicle. The compensation can be added for faster movement, otherwise
    % car is slow at the turns. Compute the fastest way to get to the
    % trajectory.
    d_trajectory = min(d_next,d_current) + compensation;
    
    % Repeat the same procedure as for the angle controller
    error_distance_cumul = dt*(((-d_trajectory + error_distance_deriv)/2) + error_distance_cumul);
    v = Kp2*d_trajectory + Ki2*error_distance_cumul + Kd2*(-d_trajectory + error_distance_deriv)/dt;
    error_distance_deriv = d_trajectory;
    
    % Plot the vehicle movement
    vehicle.plot()
end

%% 4. Plotting the vehicle trajectory
figure()
vehicle.plot_xy()
title('Trajectory of the vehicle');


