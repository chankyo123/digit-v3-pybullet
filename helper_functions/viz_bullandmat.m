clc; clear;
close all;

%% Set Labels
robot = createFloatingBaseDigit();
labels = ["left-hip-roll","left-hip-yaw","left-hip-pitch","left-knee", ...
        "right-hip-roll","right-hip-yaw", "right-hip-pitch", "right-knee", ...
        "left-toe-A", "left-toe-B", "right-toe-A", "right-toe-B"];
% left_hip_roll, left_hip_yaw, left_hip_pitch, left_knee, right_hip_roll, right_hip_yaw, right_hip_pitch, right_knee, left_toe_A, left_toe_B, right_toe_A, right_toe_B
%% Load Bullet Data
% Read the CSV file
% filename_bullet = 'checkJoints-zerofriction.csv';
% filename_bullet = 'checkJoints-1.csv';
% filename_bullet = 'test-drop-fricdamp.csv';

filename_bullet = 'data/pd-4.csv';

data_bullet = readmatrix(filename_bullet);

% Get the number of columns in the data
numColumns = size(data_bullet, 2);

% Split the data and labels into two halves
half = floor(numColumns / 3);
col = 2;
time_raw = data_bullet(:,end);
pos_bullet = data_bullet(:, 1:half);
rhr_raw = pos_bullet(:, 5);

time = 0.05;

t_ix = find(time_raw > time, 1);


pos_bullet = data_bullet(1:t_ix, 1:half);
% pos_bullet = pos_bullet(:, [4,3,2,1,5,6,7,8,9,10,11,12]);

vel_bullet = data_bullet(1:t_ix, half+1:half*2);

% tau_bullet = data_bullet(1:t_ix, 2*half+1:half*3);
time_bullet  = data_bullet(1:t_ix,end);
acc_bullet = gradient(vel_bullet)/time_bullet(2);

%% Load Matlab Data
% Read the CSV file
% filename_mat = 'matlab-drop-1.5.csv';
% filename_mat = 'matlab-pd-bul-zerofriction-check.csv';
filename_mat = 'data/matlab-pd-bul-4.csv';
% filename_mat = 'matlab-drop-fricdamp.csv';
% filename_mat = 'matlab-pd-bul-5-rmfield.csv';

% filename_mat = 'matlab-pd.csv';
% filename_mat = 'matlab-pd-notrans.csv';



data_mat = readmatrix(filename_mat);

% Get the number of columns in the data
numColumns = size(data_mat, 2);

% Split the data and labels into two halves
half = floor(numColumns / 2);
col = 2;
time_raw  = data_mat(:,end);
pos_mat = data_mat(:, 1:half);
rhr_raw = pos_mat(:, 9);

t_ix = find(time_raw > time, 1);

pos_mat = data_mat(1:t_ix, 1:half);
vel_mat = data_mat(1:t_ix, half+1:half*2);

% acc_mat = data_mat(1:t_ix, half*2+1:half*3);
% tau_mat = data_mat(1:t_ix, half*3+1:half*4);
time_mat  = data_mat(1:t_ix,end);
acc_mat = gradient(vel_mat)/time_mat(2);

% pos_mat = pos_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
pos_mat = pos_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
vel_mat = vel_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
% acc_mat = acc_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
% tau_mat = tau_mat(:, [1,2,3,4,9,10,11,12,17,18,19,20]);

% %% Load Matlab Data
% % Read the CSV file
% % filename_mat = 'matlab-drop-1.5.csv';
% filename_mat2 = 'matlab-pd-bul-14.csv';
% 
% 
% 
% data_mat2 = readmatrix(filename_mat2);
% 
% % Get the number of columns in the data
% numColumns = size(data_mat2, 2);
% 
% % Split the data and labels into two halves
% half = floor(numColumns / 3);
% col = 2;
% time_raw2  = data_mat2(:,end);
% 
% 
% t_ix = find(time_raw2 > time, 1);
% 
% pos_mat2 = data_mat2(1:t_ix, 1:half);
% vel_mat2 = data_mat2(1:t_ix, half+1:half*2);
% acc_mat2 = data_mat2(1:t_ix, half*2+1:half*3);
% time_mat2  = data_mat2(1:t_ix,end);
% 
% pos_mat2 = pos_mat2(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
% vel_mat2 = vel_mat2(:, [1,2,3,4,9,10,11,12,17,18,19,20]);
% acc_mat2 = acc_mat2(:, [1,2,3,4,9,10,11,12,17,18,19,20]);

%% Viz
fig_size = [100, 100, 1600, 600];

figure;
for i = 1:12
    
    x1 = time_mat;
    y1 = pos_mat(:,i);
    x2 = time_bullet;
    y2 = pos_bullet(:,i);
%     x3 = time_mat2;
%     y3 = pos_mat2(:,i);
    
    figure_position = fig_size;set(gcf, 'Position', figure_position);
    
    subplot(2,6, i); hold on;
    plot(x1, y1,'Color', [0 0.4470 0.7410]);
    title(labels(i));
    xlim([x1(1), x1(end)]);
    xlabel('Time (s)');
   
    subplot(2,6, i);hold on;
    plot(x2, y2, 'Color',[0.8500 0.3250 0.0980]);

%     subplot(2,6, i);hold on;
%     plot(x3, y3);
    
    sgtitle('Matlab and Bullet Position (rad)');
    legend('Matlab', 'PyBullet');


end

figure;
for i = 1:12
    
    x1 = time_mat;
    y1 = vel_mat(:,i);
    
    x2 = time_bullet;
    y2 = vel_bullet(:,i);

    
    figure_position = fig_size;set(gcf, 'Position', figure_position);
    
    subplot(2,6, i); hold on;
    plot(x1, y1,'Color', [0 0.4470 0.7410]);
    title(labels(i));
    xlabel('Time (s)');
   
    subplot(2,6, i);hold on;
    plot(x2, y2);
    title(labels(i));
        
    sgtitle('Matlab and Bullet Velocity (rad/s)');
    legend('Matlab', 'PyBullet');

end

figure;
for i = 1:12
    
    x1 = time_mat;
    y1 = acc_mat(:,i);
    
    x2 = time_bullet;
    y2 = acc_bullet(:,i);

    
    figure_position = fig_size;set(gcf, 'Position', figure_position);
    
    subplot(2,6, i); hold on;
    plot(x1, y1,'Color', [0 0.4470 0.7410]);
    title(labels(i));
    xlabel('Time (s)');
   
    subplot(2,6, i);hold on;
    plot(x2, y2);
    title(labels(i));
        
    sgtitle('Matlab and Bullet Acceleration');
    legend('Matlab', 'PyBullet');

end


% figure;
% for i = 1:12
%     
%     x1 = time_bullet;
%     y1 = acc_bullet(:,i);
%     y2 = tau_bullet(:,i);
% 
%     
%     figure_position = fig_size;set(gcf, 'Position', figure_position);
%     
% %     subplot(2,6, i); hold on;
% %     plot(x1, y1,'Color', [0 0.4470 0.7410]);
% %     title(labels(i));
%     
%    
%     subplot(2,6, i);hold on;
%     plot(x1, y2);
%     title(labels(i));
%     
%     xlabel('Time (s)');
%     sgtitle('Bullet Torque');
% %     legend('Acc', 'Tau');
% 
% end

