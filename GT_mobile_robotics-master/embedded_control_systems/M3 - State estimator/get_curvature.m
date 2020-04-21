function [kappa] = get_curvature(data_carthesian, Uk)
% Return the curvature values based on carthesian measurements
%
% INPUTS
% data_carthesian =  LIDAR measurements converted to a Carthesian coordinate system with getCarthesian
%
% OUTPUTS
% kappa = value of the curvature function (as defined in Natural landmark extraction for mobile robot navigation based on an adaptive curvature estimation)

%% INITIALIZATION
%Uk = getParameter({'Uk'}); % tolerated noise levels

Kf = nan(size(data_carthesian,1),1); % maximum length of laser scan presenting no discontinuities on the right side of the working range reading
Kb = nan(size(data_carthesian,1),1); % maximum length of laser scan presenting no discontinuities on the left side of the working range reading
f = nan(2,1); % local vector (presents variation in the x- and y-axis between reading i and i+kfi)
b = nan(2,1); % local vector (presents variation in the x- and y-axis between reading i and i-kbi)
kappa = nan(size(data_carthesian,1),1); % value of curvature function

%% CALCULATION OF CURVATURE VALUES
% For more information see 'Natural landmark extraction for mobile robot navigation based on an adaptive curvature estimation' and paper

for i = 1:size(data_carthesian,1)
    j = i;
    xi = data_carthesian(i,1); % x-coordinate of considered LIDAR data point
    yi = data_carthesian(i,2); % y-coordinate of considered LIDAR data point
    l = 0; % laser scan length see 'Natural landmark extraction for mobile robot navigation based on an adaptive curvature estimation'
    d = 0; % Euclidian distance see 'Natural landmark extraction for mobile robot navigation based on an adaptive curvature estimation'
    k = 0;
    
    %% FIND DISCONTINUITIES
    while j < size(data_carthesian,1) && l - d <= Uk
        xj = data_carthesian(j,1);
        yj = data_carthesian(j,2);
        xj2 = data_carthesian(j+1,1);
        yj2 = data_carthesian(j+1,2);
        l = l + sqrt((xj2-xj)^2+(yj2-yj)^2);
        d = sqrt((xj2-xi)^2+(yj2-yi)^2);
        j = j+1;
        k = k+1;
    end
    if j < size(data_carthesian,1)-1
        Kf(i)= k-1;
    elseif l - d > Uk
        Kf(i) = k-1;
    else
        Kf(i) = k;
    end
    j = i;
    l = 0;
    d = 0;
    k = 0;
    
    while j > 1 && l - d <= Uk
        xj = data_carthesian(j,1);
        yj = data_carthesian(j,2);
        xj2 = data_carthesian(j-1,1);
        yj2 = data_carthesian(j-1,2);
        l = l + sqrt((xj2-xj)^2+(yj2-yj)^2);
        d = sqrt((xj2-xi)^2+(yj2-yi)^2);
        j = j-1;
        k = k+1;
    end
    
    if j > 1
        Kb(i) = k-1;
    elseif l - d > Uk
        Kb(i) = k-1;
    else
        Kb(i) = k;
    end
    %% OBTAIN LOCAL VECTORS
    if i + Kf(i) <= size(data_carthesian,1)
        f(1) = data_carthesian(i+Kf(i),1)-data_carthesian(i,1);
        f(2) = data_carthesian(i+Kf(i),2)-data_carthesian(i,2);
    else
        f(1) = data_carthesian(end,1)-data_carthesian(i,1);
        f(2) = data_carthesian(end,2)-data_carthesian(i,2);
    end
    
    if i - Kb(i) >= 1
        b(1) = data_carthesian(i-Kb(i),1)-data_carthesian(i,1);
        b(2) = data_carthesian(i-Kb(i),2)-data_carthesian(i,2);
    else
        b(1) = data_carthesian(1,1)-data_carthesian(i,1);
        b(2) = data_carthesian(1,2)-data_carthesian(i,2);
    end
    
    %% OBTAIN CURVATURE VALUES
    kappa_i = acos(dot(f,-b)/(norm(f)*norm(b)));
    kappa(i) = kappa_i;
end

end
