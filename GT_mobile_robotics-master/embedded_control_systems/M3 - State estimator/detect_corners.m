%% Detect corners
% Example file to show how detect_corners can be used to extract the
% coordinates of corners from recorded LIDAR scans. 

close all
clear all
clc

% Load example file
load('v_0.1_w_1_EKF_data.mat')

for i=1:size(scans,2)
    
    %% Preprocessing
    scan = scans(i); % Get current scan
    scan = filter_scan(scan); % Remove zeroes from scan
    [x,y]=pol2cart(scan.Angles,scan.Ranges);    % Convert to Carthesian coordinates
    
    %% Detect corners from scan
    [corners] =  get_corners(scan);

    %% Plotting
    scatter(x, y); % Plot scan
    hold on
    scatter(corners(:,1), corners(:,2), 'marker','+','LineWidth', 15, 'MarkerFaceColor','r');  % Plot detected corners
    axis(5*[-1,1,-1,1]);
    grid on
    pause(0.1)
    hold off
end
