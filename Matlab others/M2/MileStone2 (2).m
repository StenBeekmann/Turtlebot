close all
clear all
clc

startup_rvc
s = tf('s');

% Parameters
v0 = 1;    % forward velocity
gamma0 = 0;    % steer angle
n = 1500; % Number of iterations to simulate
dt = 0.1; % Timsteps
steermax = 0.8;

% Vehicle object
X = [0;0;0];
Q = diag([0.005,0.005].^2);  % noise on travelled distance[m] and the heading angle [rad]
vehicle = Bicycle('covar', Q,'x0',X, 'accelmax',1, 'speedmax',2, 'steermax',steermax, 'dt',dt);

% Waypoints
points = [0, 0;
 0, 10;
 10, 10;
 0, 10;
 0, 0];

distance_to_line=zeros(1,n);

% Initieer collecties

j = 1;
rad = 0.3;  %gaat een fout op het punt toelaten
factor_error = 1;

% Initialize figure
figure
hold on
xlim([-5,15])
ylim([-5,15])
line([0 0],[0 10],'color','r');
line([0 10],[10 10],'color','r');
line([-3 -3], [-3 13],'color','g');
line([3 3], [-3 7],'color','g');
line([13 13], [13 7],'color','g');
line([3 13],[7 7],'color','g');
line([-3 13],[13 13],'color','g');
% Run simulation
v = v0;
gamma = gamma0;
for i=2:n
    
    odom = vehicle.step(v, gamma); % Simulate 1 timestep and return odometry
    
    x = vehicle.x(1);
    y = vehicle.x(2);
    th = vehicle.x(3);
    
    %Bepalen welk punt je naartoe moet op basis van array met waypoints
    if (points(j+1,1)-rad < x) && (x < points(j+1,1)+rad) && (points(j+1,2)-rad < y) && (y < points(j+1,2)+rad)
        j = j+1;
        if j == 5
            break
        end
    end
    
    
    % PID voor hoek
    angle_to_next_point = atan2((points(j+1,2)-y),(points(j+1,1)-x));
    error = (angle_to_next_point-th);
    
    EindPunt(1) = points(j+1,1);
    EindPunt(2) = points(j+1,2);
    StartPunt(1) = points(j,1);
    StartPunt(2) = points(j,2);
    
    
    distance_to_line(i) = (abs(EindPunt(2)-StartPunt(2))*x-(EindPunt(1)-StartPunt(1))*y+(EindPunt(1)*StartPunt(2)-EindPunt(2)*StartPunt(1)))/(sqrt((EindPunt(2)-StartPunt(2))^2+(EindPunt(1)-StartPunt(1))^2));
      
    distance_tonextwaypoint= sqrt((x-points(j+1,1)).^2+(y-points(j+1,2)).^2);    

    if(error> pi())
        error=error-2*pi(); 
    end
    if(error< -pi())
        error=error+2*pi();
    end
    if j==4
        distance_to_line(i)=abs(distance_to_line(i));
    end
    if distance_to_line(i)>1.1
       v=-0.7;
       if v>0
           gamma = steermax;
       end 
       if v<0
           gamma = -steermax;
       end
    else 
        v=0.8*distance_tonextwaypoint ;
        gamma=2*error+0.7*distance_to_line(i);
    end
        
      
    %vehicle.plot()
    %grid on

end

vehicle.plot_xy();


