clear; clc;

robot = a_createFloatingBaseDigit();

q0_act = [0.03, 0.02, 0.01, 0.360407, -0.360407, -0.01, -0.02, -0.03, 0.0, 0.0, 0.0, 0.0]';
q0 = coordinateConvert(robot, robot, q0_act);
q0_d = zeros(robot.NB,1);
q0_dd = zeros(robot.NB, 1);
% fill unactuated joints
[q0, q0_d, q0_dd] = robot.fillUnactJoints(robot, q0, q0_d, q0_dd);

%% simulate
% parameters

Array = readtable('/home/chankyo/roahm/ROAHM_Robust_Control/Matlab/RobotModel/checkJoints-14.csv');
Array=table2array(Array);
time_cp = Array(:, end);
tau=Array(:,end-12:end-1);
tau_cp=tau;
data_pd = struct('tau',tau_cp,'time',time_cp);


T = 0.25;
dt = 0.0002;
tspan = 0:dt:T;
% tspan = [0 T];
% x0 = [q0; q0_d];
x0 = [q0; q0_d; 0];  % euler method

% controller
controlFunc = @(robot, q, q_d, eAcc, t) controller(robot, q, q_d, eAcc, t, data_pd);
disp('Start simDigit')
[t, e, q, q_d, q_dd, qd, qd_d, qd_dd, tau] = a_simDigit(tspan, robot, [], controlFunc, x0);
disp('Finished simulation')
%% Store in csv for further analysis
mat_data = [q', q_d', t];
col_names = [robot.jointNames, robot.jointNames];
for ii = 1 : numel(col_names)
    if (ii <= robot.NB)
        col_names(ii) = col_names(ii) + '-d0';
    else
        col_names(ii) = col_names(ii) + '-d1';
    end
end
col_names = [col_names, 'time'];
% filename_mat = 'matlab-pd-mujoco.csv';
filename_mat = 'matlab-pd-bul-0.001-14.csv';
writematrix(mat_data,filename_mat);


disp('stored!')

%% functions
function [u] = controller(robot, q, q_d, eAcc, t, data)
    % zero-order-hold
    idx_tau = find(data.time >= t, 1); %- 1;
    u = zeros(34, 1);
    disp(idx_tau);
    disp(data.time(idx_tau));
    curr_tau = data.tau(idx_tau, :)';
    if(numel(curr_tau)~=12)
        disp(size(curr_tau));
        disp(idx_tau);
    end
    u(robot.actJoints) = curr_tau;
    
    % u = ones(12, 1);
end