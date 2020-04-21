function [corners_carthesian] = get_corners(scan)
% Return a features matrix from LIDAR measurements, first find straight walls, then find curved walls between straight walls, then find corners between curved walls
% only corners are returned, as they are the only type of feature that is
% used
%
% INPUTS
% scan = range measurements from LIDAR sensor
%
% OUTPUTS
% corners = matrix of corner features, in carthesian coordinates

%% INITIALIZATION
% Constants
kappa_min = 0.5; % minimum curvature value above which features are no longer considered walls
lmin = 10; % minimum number of consecutive measurements to be considered a wall or curve
corner_lower_threshold = 1.5; % lower limit for feature to be considered a corner
corner_upper_threshold = 1.75*100; % upper limit for feature to be considered a corner
angle_range = 60*pi/180; % range of measurements within which peaks are considered to be a single feature
Uc = 1.0; % threshold on cornerity index

% Matrices
walls_index = []; % indices of wall features
walls  = []; % wall features
corners = []; % corner features
curves = []; % curve features

%% PROCESS DATA
[x,y]=pol2cart(scan.Angles,scan.Ranges);    % Convert to Carthesian coordinates
scan_carthesian = [x, y];
kappa = get_curvature(scan_carthesian, 0.5); % get curvature

% format data
data = [scan.Ranges, scan.Angles];
index =  [1:1:size(data,1)]';
data  = [index,data]; % Add index to data_radius to be used in the loops later

%% DETECT STRAIGHT WALLS

count = 0; % number of consecutive measurements below kappa_min
for i = 1:size(kappa,2)
    % If kappa is smaller than minimum, continue adding wall segment
    if kappa(i) <= kappa_min
        count = count + 1;
        
        % If kappa becomes too large and the minimum length is reached, add to wall matrix
    elseif count >= lmin
        
        % Add indices for looping
        walls_index(1,end+1) = i-count; %Beginpoint of detected wall
        walls_index(2,end) = i; %Endpoint of detected wall
        
        % Add radii and angles to walls matrix
        walls(end+1,1) = data(i-count,2); % radius of beginpoint
        walls(end,2) = data(i-count,3); % angle of beginpoint
        
        walls(end+1,1) = data(i,2); % radius of beginpoint
        walls(end,2) = data(i,3); % angle of beginpoint
        
        count = 0; % start looking for new walls
        
    else
        count = 0; % start looking for new walls
    end
end

% If to seperate wall segments start at +-0 and end at +-360, then join segments into one wall
if ~isempty(walls_index) && data(walls_index(1,1)) <= 2*pi/180 && data(walls_index(2,end)) >= 359*pi/180
    walls_index(2,end) = walls_index(2,1);
    walls_index = walls_index(:,2:end);
end

%% DETECTION OF CURVED WALLS
% Intermediate matrix based on walls for calculation purposes
inter_walls = walls_index;
if ~isempty(walls_index)
    inter_walls(1,end+1) = walls_index(1,1);
    inter_walls(2,end) = walls_index(2,1);
else
    inter_walls(1,end+1) = 0;
    inter_walls(2,end) = 1;
    inter_walls(1,end+1) = size(data,1);
    inter_walls(2,end) = 0;
end

% Detect curves between wall segments
for i = 1:size(inter_walls,2)-1
    elements = []'; % intermediate matrix of kappa values which shall be used to calculate cornerity index
    
    % Construct index vector to loop over
    % Make sure to also loop over transition from last to first measurement
    if inter_walls(2,i) < inter_walls(1,i+1)
        s = [inter_walls(2,i):1:inter_walls(1,i+1)];
    else
        s1 = [inter_walls(2,i):1:size(data,1)];
        s2 = [1:1:inter_walls(1,i+1)];
        s = [s1 s2];
    end
    
    % Loop over index vector and dectect curves
    for j = s
        cornerity_index = mean(elements,'omitnan')./max(elements,[],'omitnan'); %Cornerity index as defined in Natural landmarks paper
        if (cornerity_index < Uc | j == inter_walls(1,i+1) | isnan(data(j))) & size(elements,2) >= lmin %Add curve segment if it is long enough, and ci is below threshold
            
            % Add begin and endpoints to curves matrix
            curves(1,end+1) = j - size(elements,2);
            curves(2,end) = j;
            elements = []; % empty elements matrix
        end
        
        if isempty(elements) % if elements is empty then add kappa no matter what
            elements(end+1) = kappa(j);
        elseif cornerity_index > Uc  % add kappa if ci is above threshold
            elements(end+1) = kappa(j);
        else
            elements = [];
        end
    end
end

%% DETECTION OF CORNERS

% Intermediate matrix based on walls for calculation purposes
inter_curves = curves;
if ~isempty(curves)
    inter_curves(1,end+1) = curves(1,1);
    inter_curves(2,end) = curves(2,1);
else
    inter_curves = inter_walls;
end

% Look for corners between curve segments
for i = 1:size(inter_curves,2)-1
    % Constructing index vector to loop over
    % Make sure to also loop over transition from last to first measurement
    if inter_curves(2,i) < inter_curves(1,i+1)
        q = [inter_curves(2,i):1:inter_curves(1,i+1)];
    else
        q1 = [inter_curves(2,i):1:size(data,1)];
        q2 = [1:1:inter_curves(1,i+1)];
        q = [q1 q2];
    end
    
    % Loop over index vector and detect corner
    for j = q
        
        if kappa(j) >= corner_lower_threshold && kappa(j) <= corner_upper_threshold %Condition to be considered a corners
            
            if isempty(corners) %If corners is empty then make new row
                corners(end+1,1) = data(j,2);
                corners(end,2) = data(j,3);
                corners(end,3) = kappa(j);
                
            elseif ~any(abs(corners(:,2) - data(j,3)) <= angle_range) % If corners does not contain current element, make a new row
                
                corners(end+1,1) = data(j,2);
                corners(end,2) = data(j,3);
                corners(end,3) = kappa(j);
                
            else
                
                % Make sure the current corners is not already in the corners matrix
                [~,index] = sort(abs(corners(:,2) - data(j,3)) <= angle_range,'descend'); % Sort matrix of angle comparisons (matrix of logical 1's and 0's)
                corners_index = index(1,1); %  First element in index matrix is the row of the corner element which is already in the matrix
                kappa_in_matrix = corners(corners_index,3); % Get kappa value of corner element in matrix for comparison
                
                if kappa(j) >= kappa_in_matrix
                    
                    corners(corners_index,1) = data(j,2);
                    corners(corners_index,2) = data(j,3);
                    corners(corners_index,3) = kappa(j);
                end
            end
        end
    end
end

%% FORMATTING OUTPUT
% Delete kappa column
if ~isempty(corners)
    corners = [corners(:,1),corners(:,2)];
    [x_corners, y_corners] = pol2cart(corners(:,2), corners(:,1));
    corners_carthesian = [x_corners, y_corners];
else
    corners_carthesian = [nan, nan];
end

% Delete index from data_radius
if ~isempty(data)
    data = data(:,2:3);
end
end