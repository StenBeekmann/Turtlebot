function [ds,dth,t] = ModelValues(tick_left,tick_right,t)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% parameters
tick2rad = 0.001533981;  % [radians/ encoder tick]
radius = 0.033 ;            % [meter]
B_wheel = 0.16;             % [meter]

%% encoder to wheel conversion
%left conversion
th_left = tick_left*tick2rad;      % radians
s_left = th_left*radius;           % meter 

%right conversion
th_right = tick_right*tick2rad;    % radians
s_right = th_right*radius;         % meter 

%% position: wheel to vehicle conversion
s_car   = (s_left+s_right)/2;      % position straight
th_car  = (s_right-s_left)/B_wheel % angular position wrt normal

%% position to velocity conversion
i = 1 
while i < size(t)+1
    if i = 1
         ds(1) = 0;
         dth(1) = 0;
    else 
    ds(i) = (s_car(i)-s_car(i-1))/(t(i)-t(i-1));
    dth(i) = (th_car(i)-th_car(i-1))/(t(i)-t(i-1));
    end
i=i+1;
end


end

