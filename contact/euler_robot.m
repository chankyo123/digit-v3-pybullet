clear; clc;

%% Import the URDF robot model
robot = importrobot('/Users/ckkim/Chankyo Kim/Michigan/pybullet/urdf/kinova-4dof.urdf');
robot.DataFormat = 'column';
array = readtable('/Users/ckkim/Chankyo Kim/Michigan/pybullet/contact/eul-checkJ-20.csv');
array=table2array(array);
time_cp = array(:,end);
tau_cp = array(:,end-4:end-1)';
q_cp = array(:,1:4)';
dq_cp = array(:,5:8)';
dt = time_cp(2) - time_cp(1);
%% Set the initial conditions
q0 = q_cp(:,1);  % Initial joint positions
dq0 = dq_cp(:,1); % Initial joint velocities

% Preallocate arrays to store joint positions and velocities
q = zeros(numel(q0), length(time_cp));
dq = zeros(numel(dq0), length(time_cp));
q(:,1) = q0;
dq(:,1) = dq0;

% Loop through the time steps
for i = 2:length(time_cp)
    % Compute the forward dynamics to obtain acceleration
%     acc = robot.forwardDynamics(q(:,i-1), dq(:,i-1), tau_cp(:,i-1));
    acc = robot.forwardDynamics(q(:,i-1), dq(:,i-1), [0,0,20000,0]');
    
    % Perform backward Euler integration
    dq(:,i) = dq(:,i-1) + dt * acc;  % Update velocity
    q(:,i) = q(:,i-1) + dt * dq(:,i); % Update position
%     disp([size(dt),size(acc),size(dq(:,i-1)),size(q(:,i-1))]);
end

% Plot the joint positions over time
t = time_cp;
% plot(t, q(1,:), 'r', t, q(2,:), 'g', t, q(3,:), 'b', t, q(4,:));
plot(t, q_cp(1,:), 'r', t, q_cp(2,:), 'g', t, q_cp(3,:), 'b', t, q_cp(4,:));
xlabel('Time');
ylabel('Joint Position');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');


