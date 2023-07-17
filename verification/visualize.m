clear; clc; close all;
robot = createFloatingBaseDigit();
% q0 = [0.03, 0.02, 0.01, 0.360407, -0.360407, -0.01, -0.02, -0.03, 0.0, 0.0, 0.0, 0.0]';
% filename_mat = 'checkJoints-fricdamp.csv';
filename_mat = 'matlab-pd-bul-zerofriction-check.csv';
% filename_mat = 'matlab-drop-fricdamp.csv';
% filename_mat = 'matlab-drop-1.csv';

% filename_mat = 'pd-1.csv';
% filename_mat = 'matlab-pd.csv';

data_mat = readmatrix(filename_mat);

% Get the number of columns in the data
numColumns = size(data_mat, 2);
numRows = size(data_mat,1);
% Split the data and labels into two halves
half = floor(numColumns / 2);
col = 2;

pos_mat = data_mat(:, 1:half);
pos_mat_mod = pos_mat;
% pos_mat_mod = zeros(numRows, robot.NB);
% pos_mat_mod(:,[1,2,3,4,9,10,11,12,17,18,19,20]) = pos_mat;

%% left -> right
% pos_mat_mod(:,1) = -pos_mat_mod(:,9);
% pos_mat_mod(:,2) = -pos_mat_mod(:,10);
% pos_mat_mod(:,3) = -pos_mat_mod(:,11);
% pos_mat_mod(:,4) = -pos_mat_mod(:,12);
% pos_mat_mod(:,5) = -pos_mat_mod(:,13);
% pos_mat_mod(:,6) = -pos_mat_mod(:,14);
% pos_mat_mod(:,7) = -pos_mat_mod(:,15);
% pos_mat_mod(:,8) = -pos_mat_mod(:,16);
% pos_mat_mod(:,17) = -pos_mat_mod(:,19);
% pos_mat_mod(:,18) = -pos_mat_mod(:,20);
% pos_mat_mod(:,21) = -pos_mat_mod(:,25);
% pos_mat_mod(:,22) = -pos_mat_mod(:,26);
% pos_mat_mod(:,23) = -pos_mat_mod(:,27);
% pos_mat_mod(:,24) = -pos_mat_mod(:,28);
% pos_mat_mod(:,29) = -pos_mat_mod(:,32);
% pos_mat_mod(:,30) = -pos_mat_mod(:,33);
% pos_mat_mod(:,31) = -pos_mat_mod(:,34);

%% right -> left
% pos_mat_mod(:,9) = -pos_mat_mod(:,1);
% pos_mat_mod(:,10) = -pos_mat_mod(:,2);
% pos_mat_mod(:,11) = -pos_mat_mod(:,3);
% pos_mat_mod(:,12) = -pos_mat_mod(:,4);
% pos_mat_mod(:,13) = -pos_mat_mod(:,5);
% pos_mat_mod(:,14) = -pos_mat_mod(:,6);
% pos_mat_mod(:,15) = -pos_mat_mod(:,7);
% pos_mat_mod(:,16) = -pos_mat_mod(:,8);
% pos_mat_mod(:,19) = -pos_mat_mod(:,17);
% pos_mat_mod(:,20) = -pos_mat_mod(:,18);
% pos_mat_mod(:,25) = -pos_mat_mod(:,21);
% pos_mat_mod(:,26) = -pos_mat_mod(:,22);
% pos_mat_mod(:,27) = -pos_mat_mod(:,23);
% pos_mat_mod(:,28) = -pos_mat_mod(:,24);
% pos_mat_mod(:,32) = -pos_mat_mod(:,29);
% pos_mat_mod(:,33) = -pos_mat_mod(:,30);
% pos_mat_mod(:,34) = -pos_mat_mod(:,31);

% pos_mat = pos_mat(:, [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34]);
time_mat  = data_mat(:,end);

showmotion(robot, time_mat, pos_mat_mod');