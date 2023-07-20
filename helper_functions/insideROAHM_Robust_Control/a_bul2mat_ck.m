clear; clc;

robot = a_createFloatingBaseDigit();
% if isfield(robot, 'transmissionInertia')
%     robot = rmfield(robot, 'transmissionInertia');
% end
q0 = [0.360407,0.01, 0.02, 0.03, 0.0, 0.0, 0.0, 0.0, -0.360407, -0.01, -0.02, -0.03, 0.0, 0.0, 0.0, 0.0, 0.001, 0.001, -0.001, -0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]';
% q0_act = [0.03, 0.02, 0.01, 0.360407, -0.360407, -0.01, -0.02, -0.03, 0.0, 0.0, 0.0, 0.0]';
% q0 = coordinateConvert(robot, robot, q0_act);
q0_d = zeros(robot.NB,1);
q0_dd = zeros(robot.NB, 1);
% fill unactuated joints
% [q0, q0_d, q0_dd] = robot.fillUnactJoints(robot, q0, q0_d, q0_dd);

%% simulate
% parameters

Array = readtable('/home/chankyo/roahm/ROAHM_Robust_Control/Matlab/RobotModel/checkJoints-8.csv');
Array=table2array(Array);
time_cp = Array(:, end);
tau=Array(:,end-12:end-1);
tau_cp=tau;
data_pd = struct('tau',tau_cp,'time',time_cp);


T = 0.12;
dt = 0.002;
% tspan = 0:dt:T;
tspan = [0 T];
x0 = [q0; q0_d];
% x0 = [q0; q0_d; 0];  % euler method
torque = tau_cp;

disp('Start simDigit')
% controller
[t_sol,x_sol_pinned] = ode15s(@(t,x)robotEoM(robot, x, t, torque), tspan, x0);
q_pinned = x_sol_pinned(:,1:robot.NB);
qd_pinned = x_sol_pinned(:,robot.NB+1:end); 
disp('Finished simulation');
%% store in csv
mat_data = [q_pinned, qd_pinned, t_sol];
col_names = [robot.jointNames, robot.jointNames];

for i = 1 : numel(col_names)
    if (i <= robot.NB)
        col_names(i) = col_names(i) + '-q';
    else
        col_names(i) = col_names(i) + '-qd';
    end
end

writematrix(mat_data,'matlab-pd-bul-8.csv');

% Result Visualization
% showmotion(robot, t_sol,q_pinned');

%% helper functions
function x_dot = robotEoM(robot, x, t, torque)
    nJ = robot.NB;
    
    % Copy state variables
    q = x(1:nJ);
    q_d = x(nJ + 1:2*nJ);

    tau = zeros(robot.NB,1); % zero torque control

    %using torque from pybullet
    torq = evalin('base','tau_cp');
    time = evalin('base','time_cp');

    idx = find (time <= t, 1, 'last');
    if isempty(idx)
        idx = 1;
    end

    currentTorque = torq(idx,:);
    tau([1,2,3,4,9,10,11,12,17,18,19,20]) = currentTorque;
    q_dd = robot.FDcrb_constraints(robot, q, q_d, tau);    
    
    x_dot = [q_d; q_dd];
    disp([idx,t]);
end