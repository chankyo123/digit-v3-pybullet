clc; clear;close all;

%% Set Labels
labels = ["left-hip-roll","left-hip-yaw","left-hip-pitch","left-knee", ...
        "right-hip-roll","right-hip-yaw", "right-hip-pitch", "right-knee", ...
        "left-toe-A", "left-toe-B", "right-toe-A", "right-toe-B"];
% labels = ["left-knee","left-hip-pitch","left-hip-yaw","left-hip-roll", "left-toe-A", "left-toe-B"];
% labels = ["right-hip-roll","right-hip-yaw", "right-hip-pitch", "right-knee", "right-toe-A", "right-toe-B"];

%% Load Bullet Data
% Read the CSV file
% filename_bullet = 'euler-Joints-2.csv';t
filename_bullet = 'data/checkJoints-10.csv';
% filename_bullet = 'pd-1.csv';

data_bullet = readmatrix(filename_bullet);

% Get the number of columns in the data
numColumns = size(data_bullet, 2);

% Split the data and labels into two halves
half = floor(numColumns / 3);
col = 2;
time_raw = data_bullet(:,end);
pos_bullet = data_bullet(:, 1:half);
rhr_raw = pos_bullet(:, 5);

% time = 2;
time = time_raw(end);
% rhr_min = min(rhr_raw);
% rhr_ix = find(rhr_raw==rhr_min, 1);


% t_ix = length(time_raw);
% t_ix = rhr_ix;
t_ix = find(time_raw >= time, 1);


pos_bullet = data_bullet(1:t_ix, 1:half);
vel_bullet = data_bullet(1:t_ix, half+1:half+12);
time_bullet  = data_bullet(1:t_ix,end);

figure;figure_position = [100, 100, 1200, 400];set(gcf, 'Position', figure_position);
for i = 1:half
    
    x2 = time_bullet;
    y2 = pos_bullet(:,i);
    
    
    subplot(2,6, i);
    plot(x2, y2);
    title(labels(i));
    xlim([x2(1), x2(end)]);
    xlabel('Time (s)');
    sgtitle('Bullet Position');


end


