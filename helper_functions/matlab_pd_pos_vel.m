clc; clear;
% Read the CSV file
% filename = 'bullet2matlab-torque-2.csv';
filename = 'bullet2matlab-torque-3.csv';
% filename = 'bullet2matlab-torque-3-right-neg.csv';

data = readmatrix(filename);
labels = ["left-knee","left-hip-pitch","left-hip-yaw","left-hip-roll", ...
        "right-hip-roll","right-hip-yaw", "right-hip-pitch", "right-knee", ...
        "left-toe-A", "left-toe-B", "right-toe-A", "right-toe-B"];

% Get the number of columns in the data
numColumns = size(data, 2);

% Split the data and labels into two halves
half = floor(numColumns / 2);
col = 2;

data1 = data(:, 1:half);
data2 = data(:, half+1:end-1);
time  = data(:,end);

% Create Figure 1 and plot the first half of the columns
figure(4);
for i = 1:12
    subplot(col, half/2, i);
    plot(time, data1(:, i));
    title(labels(i));
    xlabel('Time (s)');
    xlim([time(1), time(end)]);
    sgtitle('Position');
end

% Create Figure 2 and plot the second half of the columns
figure(5);
for i = 1:12
    subplot(col, half/2, i);
    plot(time, data2(:, i));
    title(labels(i));
    xlabel('Time (s)');
    xlim([time(1), time(end)]);
    sgtitle('Velocity');
end







