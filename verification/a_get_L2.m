clc; clear; 

%% Set Labels
labels = ["left-hip-roll","left-hip-yaw","left-hip-pitch","left-knee", ...
        "right-hip-roll","right-hip-yaw", "right-hip-pitch", "right-knee", ...
        "left-toe-A", "left-toe-B", "right-toe-A", "right-toe-B"];

%% Load Bullet Data
% Read the CSV file
% filename_bullet = 'checkJoints-21.csv';
filename_bullet = 'test-drop-fricdamp.csv';

data_bullet = readmatrix(filename_bullet);

% Get the number of columns in the data_bullet
numColumns = size(data_bullet, 2);

% Split the data_bullet and labels into three features(pos,vel,torq)
feature = (numColumns-1)/2;
col = 2;

pos_bullet = data_bullet(:, 1:feature);
vel_bullet = data_bullet(:, feature+1:2*feature);
% torq_bullet = data_bullet(:, 2*feature+1:end-1);
time_bullet  = data_bullet(:,end);

time = 0.1;
t_ix = find(time_bullet > time , 1);

pos_bullet = data_bullet(1:t_ix, 1:feature);
time_bullet  = data_bullet(1:t_ix,end);

%% Load Matlab Data
% Read the CSV file
% filename_mat = 'matlab-pd-bul-21.csv';
filename_mat = 'matlab-drop-fricdamp.csv';

data_mat = readmatrix(filename_mat);

% Get the number of columns in the data
numColumns = size(data_mat, 2);

% Split the data and labels into two halves
half = floor(numColumns / 2);
col = 2;


vel_mat = data_mat(:, half+1:end-1);
time_mat  = data_mat(:,end);
t_ix = find(time_mat > time , 1);

pos_mat = data_mat(1:t_ix, 1:half);
pos_mat = pos_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
time_mat  = time_mat(1:t_ix);

%% Get L2 Norm of POS data of Bullet/Matlab

for i = 1:feature
    
    x1 = time_bullet;
    y1 = pos_bullet(:,i);
    x2 = time_mat;
    y2 = pos_mat(:,i);
      
    % Check for unique sample points in x2
    [unique_x2, unique_idx] = unique(x2,'stable');
    unique_y2 = y2(unique_idx);
    
    % Interpolate data points of unique_y2 to align with x1
    x2_reduced = linspace(x2(1),x2(end),length(x1));
    y2_reduced = interp1(unique_x2, unique_y2, x2_reduced, 'spline');

    l2_norm = norm(y1 - y2_reduced)/numel(y1);
    disp([labels(i),l2_norm]);
    
    figure_position = [100, 100, 1200, 400];
    figure(7);set(gcf, 'Position', figure_position);
    subplot(col, feature/2, i);
    plot(x1, y1);hold on;
    title(labels(i));
    xlabel('Time (s)');
    sgtitle('Bullet Position');

    figure(8);set(gcf, 'Position', figure_position);
    subplot(col, feature/2, i);
    plot(x2, y2); hold on;
%     plot(x2_reduced, y2_reduced); hold on;
    title(labels(i));
    xlim([x2_reduced(1), x2_reduced(end)]);
    xlabel('Time (s)');
    sgtitle('Matlab Position');

end





