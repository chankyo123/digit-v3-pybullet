function [t, e, q, q_d, q_dd, qd, qd_d, qd_dd, tau] = a_simDigit(tspan, robot, trajFunc, controller, x0)
% simDigit: simulate Digit forward given a controller (and a desired trajectory)
% Input:
%   1. tspan: a row vector with the time steps for ode solvers.
%   2. robot: a Roy Featherstone robot model.
%   3. trajFunc: a function handle to compute desired trajectories. can be [] if no desired trajectory needed
%   4. controller: a function handle to compute control inputs. can be [] if no controller needed
%   5. x0: the initial condition containing inital postion and initial velocity
% Output:
%   1. t: time steps solved by ode
%   2. e: integral of tracking error solved by ode
%   3. q: position solved by ode
%   4. q_d: velocity solved by ode
%   5. q_dd: acceleration solved by ode
%   6. qd: desired position specified by trajFunc
%   7. qd_d: desired velocity specified by trajFunc
%   8. qd_dd: desired acceleration specified by trajFunc
%   9. tau: control torque input specified by controller
% Usage:
%   1. desired trajectory and controller provided
%      [t, e, q, q_d, q_dd, qd, qd_d, qd_dd, tau] = simDigit(tspan, robot, trajFunc, controller, x0);
%   2. desired trajectory not provided, controller solely depends on time and system states
%      [t, e, q, q_d, q_dd, ~, ~, ~, tau] = simDigit(tspan, robot, [], controller, x0);
%   3. zero control input all the time
%      [t, e, q, q_d, q_dd] = simDigit(tspan, robot, [], [], x0);

    % Initial condition (if not specified)
    if nargin < 5
        [q, q_d, ~] = trajFunc(tspan(1));
        x0 = [q; q_d; 0; 0];
    end
    
    % Option
%     opts = odeset('AbsTol', 1e-8, 'RelTol', 1e-8, 'Events',@(t,y) eventFunc(robot,t,y));
    opts = odeset('AbsTol', 1e-8, 'RelTol', 1e-8);
    
    % Simulate
    odeFunc = @(t, x) robotEoM(robot, x, t, trajFunc, controller);
    % [T, X, te, ~] = ode15s(odeFunc, tspan, x0, opts);
    [T, X] = ode15s(odeFunc, tspan, x0, opts);
    disp('finished solving ode');
    
    % Extract state variables
    t = T;
    q = X(:, 1:robot.NB)';
    q_d = X(:, robot.NB + 1:2*robot.NB)';
    e = X(:, 2*robot.NB + 1:end)';
    
    % Record all data
    q_dd = zeros(size(q));

    if ~isempty(trajFunc)
        qd = zeros(size(q));
        qd_d = zeros(size(q));
        qd_dd = zeros(size(q));
    else
        qd = zeros(size(q));
        qd_d = zeros(size(q));
        qd_dd = zeros(size(q));
        % qd = [];
        % qd_d = [];
        % qd_dd = [];
    end

    if ~isempty(controller)
        tau = zeros(size(q)); 
    else
        tau = [];
    end

end

function x_dot = robotEoM(robot, x, t, trajFunc, controller)
    % fprintf("Time: %2.4f\n", t);
    nB = robot.NB;
    
    % Copy state variables
    q = x(1:nB);
    q_d = x(nB + 1:2*nB);

    % Integral of tracking error
    eAcc = x(2*nB + 1 : end);
    
    % Desired trajectory
    if ~isempty(trajFunc)
        [qd, qd_d, qd_dd] = trajFunc(t);
    else
        qd = zeros(robot.NB,1);
        qd_d = zeros(robot.NB,1);
        qd_dd = zeros(robot.NB,1);
    end
    
    % Control
    if ~isempty(controller)
        if ~isempty(trajFunc)
            u = controller(robot, q, q_d, qd, qd_d, qd_dd, eAcc, t);
        else
            u = controller(robot, q, q_d, eAcc, t);
        end
    else
        u = zeros(robot.NB,1);
    end
    
    % Get acceleration update and error
    q_dd = robot.FDcrb_constraints(robot, q, q_d, u);
    
    % Get tracking error (over actuated joints)
    e = norm(qd(robot.actJoints) - q(robot.actJoints));
    % disp('here');
    % x_dot = [q_d; q_dd; e];
    x_dot = [q_d; q_dd];
    % disp(size(q_d));
    % disp(size(q_dd));
end

% Used to determine if the constraint violation grows too big
function [value, isterminal, direction] = eventFunc(robot, t, x)
    nB = robot.NB;
    
    % Copy state variables
    q = x(1:nB);
    
    % Get constraint violation
    c = robot.posConstraints(robot, q(robot.depJoints), q);
    
    % Keep norm of constraint violation below 10^-2
    value = vecnorm(c) - 1e-2;
    
    isterminal = true;
    direction = 0;
end