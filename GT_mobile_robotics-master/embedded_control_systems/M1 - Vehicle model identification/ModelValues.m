function [s_car,th_car,sdot,thdot,t] = ModelValues(tick_left,tick_right,t)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% parameters
tick2rad = 0.001533981;  % [radians/ encoder tick]
radius = 0.033 ;            % [meter]
B_wheel = 0.16;             % [meter]

%% encoder to wheel conversion
%left conversion

th_left = (tick_left-(tick_left(1)))*tick2rad;      % radians
s_left = th_left*radius;           % meter 

%right conversion
th_right = (tick_right-(tick_right(1)))*tick2rad;    % radians
s_right = th_right*radius;         % meter 


%% position: wheel to vehicle conversion
s_car   = (s_left+s_right)/2;      % position straight
th_car  = (s_right-s_left)/B_wheel; % angular position wrt normal

%% position to velocity conversion
i = 1 
 ds = zeros(1,length(t));
 dth = zeros(1,length(t));
while i < length(t)+1
    if i == 1
         ds(1,1) = 0;
         dth(1,1) = 0;
    else 
    ds(1,i+1git) = (s_car(i)-s_car(i-1))/(t(i)-t(i-1));
    dth(1,i) = (th_car(i)-th_car(i-1))/(t(i)-t(i-1));
    end
i=i+1;
end
sdot = ds;
thdot = dth;

end

