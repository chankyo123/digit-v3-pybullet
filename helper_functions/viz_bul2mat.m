clc; clear;

%% Set Labels
labels = ["left-knee","left-hip-pitch","left-hip-yaw","left-hip-roll", ...
        "right-hip-roll","right-hip-yaw", "right-hip-pitch", "right-knee", ...
        "left-toe-A", "left-toe-B", "right-toe-A", "right-toe-B"];

%% Load Matlab Data
% Read the CSV file
filename_mat = 'matlab-drop-1.5.csv';

data_mat = readmatrix(filename_mat);

% Get the number of columns in the data
numColumns = size(data_mat, 2);

% Split the data and labels into two halves
half = floor(numColumns / 2);
col = 2;
time_raw  = data_mat(:,end);
pos_mat = data_mat(:, 1:half);
rhr_raw = pos_mat(:, 9);

time = 1.2;
rhr_min = min(rhr_raw);
rhr_ix = find(rhr_raw==rhr_min, 1);

% t_ix = length(time_raw);
t_ix = find(time_raw > time, 1);
% t_ix = rhr_ix;

pos_mat = data_mat(1:t_ix, 1:half);
vel_mat = data_mat(1:t_ix, half+1:end-1);
time_mat  = data_mat(1:t_ix,end);

pos_mat = pos_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
% vel_mat = data_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);

%% Load Stiff-modified Matlab Data
% Read the CSV file
filename_mat2 = 'matlab-drop-1.5-stiff.csv';

data_mat2 = readmatrix(filename_mat2);

% Get the number of columns in the data
numColumns2 = size(data_mat2, 2);

% Split the data and labels into two halves
half2 = floor(numColumns2 / 2);
col = 2;
time_raw2  = data_mat2(:,end);
pos_mat2 = data_mat2(:, 1:half);
rhr_raw2 = pos_mat2(:, 9);

rhr_min2 = min(rhr_raw2);
rhr_ix2 = find(rhr_raw2==rhr_min2, 1);

% t_ix = length(time_raw);
t_ix2 = find(time_raw2 > time, 1);
% t_ix = rhr_ix;

pos_mat2 = data_mat2(1:t_ix2, 1:half);
vel_mat2 = data_mat2(1:t_ix2, half+1:end-1);
time_mat2  = data_mat2(1:t_ix2,end);

pos_mat2 = pos_mat2(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
% vel_mat = data_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);

%% Load Stiff-modified Matlab Data
% Read the CSV file
filename_mat3 = 'matlab-drop-1.5-stiff-2.csv';

data_mat3 = readmatrix(filename_mat3);

% Get the number of columns in the data
numColumns3 = size(data_mat3, 2);

% Split the data and labels into two halves
half3 = floor(numColumns3 / 2);
col = 2;
time_raw3  = data_mat3(:,end);
pos_mat3 = data_mat3(:, 1:half);
rhr_raw3 = pos_mat3(:, 9);

rhr_min3 = min(rhr_raw3);
rhr_ix3 = find(rhr_raw3==rhr_min3, 1);

% t_ix = length(time_raw);
t_ix3 = find(time_raw3 > time, 1);
% t_ix = rhr_ix;

pos_mat3 = data_mat3(1:t_ix3, 1:half);
vel_mat3 = data_mat3(1:t_ix3, half+1:end-1);
time_mat3  = data_mat3(1:t_ix3,end);

pos_mat3 = pos_mat3(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
% vel_mat = data_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);

%% Viz
% figure;
% for i = 1:12
%     
%     x1 = time_mat;
%     y1 = pos_mat(:,i);
%     
%     figure_position = [100, 100, 1200, 400];set(gcf, 'Position', figure_position);
% 
% %     figure;
%     
%     subplot(2,6, i);
%     plot(x1, y1);
%     title(labels(i));
%     xlim([x1(1), x1(end)]);
%     xlabel('Time (s)');
%     sgtitle('Matlab Position');
% 
% 
% end

figure;
for i = 1:12
    
    x1 = time_mat;
    y1 = pos_mat(:,i);

    x2 = time_mat2;
    y2 = pos_mat2(:,i);

    x3 = time_mat3;
    y3 = pos_mat3(:,i);

    
    figure_position = [100, 100, 1200, 400];set(gcf, 'Position', figure_position);
    
    subplot(2,6, i);
    plot(x1, y1); hold on;
    plot(x2, y2); hold on;
    plot(x3, y3); hold on;
    title(labels(i));
    xlim([x1(1), x1(end)]);
    xlabel('Time (s)');
    sgtitle('Matlab Position');

    legend('original','modified stiffness','modified stiffness2');


end

