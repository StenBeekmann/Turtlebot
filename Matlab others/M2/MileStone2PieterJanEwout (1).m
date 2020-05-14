%% 2.1 SISO controller
%%%% Initialization

clear all;
close all;
clc

startup_rvc

s = tf('s');

% Parameters
v0 = 1.0;    % forward velocity
gamma0 = 0.5;    % steer angle
n = 400; % number of iterations to simulate
dt = 0.1; % timsteps

% Vehicle object
X = [0; 0; 0];
Q=diag([0.005,0.005].^2);  % noise on travelled distance[m] and the heading angle [rad]
P = diag([0.5, 0.5, 0.5].^2);
vehicle = Bicycle('covar', Q,'x0',X);% 'accelmax',1, 'speedmax',1, 'steermax',1, 'dt',dt);

% Initialize collections
time = zeros(1,n); % Timestamps
th = zeros(1,n); % Actual angle of robot
ds = zeros(1,n);
dth = zeros(1,n);
% Initialize figure
%                                                       
figure()
hold on
xlim([-5,50])
ylim([-50,5]) 

%%%% Run simulation
v = v0;
gamma = gamma0;
Kp = -0.5; %Proportional gain
Ki= -1;   %integral gain
Kd = -1.0; %Derivative gain
error_prev = 0;
int_sum=0;
%Plotting line trajectory
m =-1; 
b = -5;
x = linspace(0,50,n);
y_path = m*x+b;
plot(x, y_path), color='red';
title('Following line trajectory starting at random position')
grid on
hold on


%Updating steering angle and plotting travelling robot
for i = 2:n
    
    odom = vehicle.step(v, gamma); % Simulate 1 timestep and return odometry
    
    x = vehicle.x(1);
    y = vehicle.x(2);
    th = vehicle.x(3);
    
    %error = sqrt((x(i)-x2).^2+(y-(m*x(i)+b)).^2);
    
    error = -(m*x - y + b)/sqrt(m^2 + 1^2);
    
    %errorlist(i)=error;
    int_sum = -(dt * ((error + error_prev)/2) + int_sum);
    
   
    gamma = Kp*(error)+Ki*int_sum+Kd*(error-error_prev)/dt; %Updating steering angle with controller
    error_prev = error;
    
    % Plot results
    vehicle.plot()
    grid on

end

%Plotting the followed path
vehicle.plot_xy()
legend('trajectory','followed path');
%title(['Kpg=', num2str(Kp),'  Ki=',num2str(Ki) ,'  Kd=', num2str(Kd)]);



%% 2.2
close all
clear all
clc

startup_rvc
s = tf('s');

% Parameters
v0 = 1.0;    % forward velocity
gamma0 = 0.5;    % steer angle
n = 500; % Number of iterations to simulate
dt = 0.1; % Timsteps
steermax = 1;


% Vehicle object
X = [-0.5;0; 0];
Q=diag([0.005,0.005].^2);  % noise on travelled distance[m] and the heading angle [rad]
vehicle = Bicycle('covar', Q,'x0',X, 'accelmax',1, 'speedmax',2, 'steermax',steermax, 'dt',dt);

% Waypoints
wps = [0, 0;
    0, 10;
    10, 10;
    0, 10;
    0, 0];

% Initieer collecties
Kp = -3; % Proportionele gain
Ki = -.2; % Integratie gain
Kd = -0.3; % Differentiele gain

Kpv=1.8; 
Kiv=0.1;
Kdv=2;

j = 1;
rad = 0.7; 
distance_error_previous=0;
dist_sum=0;
angle_sum=0;
angle_error_previous=0;

% Initialize figure
figure
hold on
xlim([-5,15])
ylim([-5,15])
line([-2 -2], [-2 12]);
line([2 2], [-2 8]);
line([12 12], [12 8]);
line([2 12],[8 8]);
line([-2 12],[12 12]);
% Run simulation
v = v0;
gamma = gamma0;
for i=2:n
    
    odom = vehicle.step(v, gamma); % Simulate 1 timestep and return odometry
    
    x = vehicle.x(1);
    y = vehicle.x(2);
    th = vehicle.x(3);
    
    if (wps(j+1,1)-rad < x) && (x < wps(j+1,1)+rad) && (wps(j+1,2)-rad < y) && (y < wps(j+1,2)+rad)
        j = j+1;
        if j == 5
            break
        end
    end
    
    
    %angle PID
    angle_to_next_waypoint=atan2((wps(j+1,2)-y),(wps(j+1,1)-x));
    error=(angle_to_next_waypoint-th);
 
    if(error> pi())
        error=error-2*pi(); 
    end
    if(error< -pi())
        error=error+2*pi();
    end
    
    angle_sum= -(dt *((error + angle_error_previous)/2) + angle_sum);
    gamma=-Kp*error +Ki*angle_sum+Kd*(error-angle_error_previous)/dt;
    angle_error_previous=error;
    
    %speed pid 
    distance_tonextwaypoint= sqrt((x-wps(j+1,1)).^2+(y-wps(j+1,2)).^2);
    distance_tocurrentwaypoint=sqrt((x-wps(j,1)).^2+(y-wps(j,2)).^2);
    minimum_dist=min(distance_tonextwaypoint,distance_tocurrentwaypoint)+0.1;
    
    dist_sum=-(dt * ((minimum_dist + distance_error_previous)/2) + dist_sum);
    
    v=Kpv*minimum_dist+ Kiv*dist_sum +Kdv*(minimum_dist-distance_error_previous)/dt;
    distance_error_previous= minimum_dist;
    
    %vehicle.plot()
    %grid on

end

vehicle.plot_xy();


