function [robot] = a_createFloatingBaseDigit(armq, includeArms)
%     path = mfilename('fullpath');
%     path = path(1:find(path == '/', 1, 'last')) + "floatingBaseDigit.mat";

    % Check if we should load the arms
    if nargin < 2
        armq = zeros(8, 1);
    elseif nargin >= 2 && length(armq) ~= 8
        armq = [-0.1148    1.0852   -0.0023   -0.1459    0.1133   -1.0836    0.0010    0.1387];
    end

    if nargin < 2
        includeArms = false;
    elseif includeArms
        warning("Dynamics and constraint functions have not been developed for " + ...
                "the arm model and will not be included");
%         path = mfilename('fullpath');
%         path = path(1:find(path == '/', 1, 'last')) + "floatingBaseDigit_ArmsInc.mat";
    end
    
%     if (nargin >= 1 && fromScratch) || ~isfile(path)

%         robot = modelFromMuJoCo("MuJoCoModels/digit_floatingbase.txt", 41);
        robot = modelFromMuJoCo("MuJoCoModels/zeromotor_digit_floatingbase.txt", 41);
%         robot = modelFromMuJoCo("MuJoCoModels/notrans_digit_floatingbase.txt", 41);
%         robot = modelFromMuJoCo("MuJoCoModels/zerofriction_digit_floatingbase.txt", 41);
        
%         robot = parseMuJoCoXML('digit-v3.xml');
    
        jointNames = ["world" "left-hip-roll" "left-hip-yaw" "left-hip-pitch" "left-knee" ...
        "left-shin" "left-tarsus" "left-toe-pitch" "left-toe-roll" ...
        "right-hip-roll" "right-hip-yaw" "right-hip-pitch" "right-knee" ...
        "right-shin" "right-tarsus" "right-toe-pitch" "right-toe-roll" ...
        "left-toe-A" "left-toe-B" "right-toe-A" "right-toe-B" ...
        "left-toe-A-rod" "left-toe-B-rod" "right-toe-A-rod" "right-toe-B-rod" ...
        "left-achilles-rod" "left-heel-spring" "right-achilles-rod" "right-heel-spring"];
    
        if includeArms
            armJointNames = ["left-shoulder-roll" "left-shoulder-pitch" "left-shoulder-yaw" "left-elbow" ...
                "right-shoulder-roll" "right-shoulder-pitch" "right-shoulder-yaw" "right-elbow"];
            jointNames = [jointNames armJointNames];
        end

        for i = 1:robot.NB
            if ~any(robot.jointNames(i) == jointNames)
                robot.jtype{i} = 'fixed';
            end
        end

        if includeArms
            for i = 1:8
                armInd = find(robot.jointNames == armJointNames(i));
                robot.Xtree{armInd} = plux(rz(-armq(i)), zeros(3, 1))*robot.Xtree{armInd};
            end
        end
        robot.armq = armq; %Keep a record so the robot can be identified later

        robot.jtype{1} = 'Rz'; %Prevent removal, needed to change pin pit    
        robot = removeFixedJoints(robot); %Change pin point wasn't made to handle fixed joints
        robot = changePinPoint(robot, 1, jointNames);
        robot.jtype{1} = 'fixed';
        robot = removeFixedJoints(robot); %Remove extra joint needed for pin model change

        % Add left toe rods
        leftARodInd = find(robot.jointNames == "left-toe-A-rod");
        robot = insertJoint(robot, leftARodInd, 'left-A2', 'Ry', eye(6), zeros(6), 0, 0.01, 0);
        leftBRodInd = find(robot.jointNames == "left-toe-B-rod");
        robot = insertJoint(robot, leftBRodInd, 'left-B2', 'Ry', eye(6), zeros(6), 0, 0.01, 0);

        % Add right toe rods
        rightARodInd = find(robot.jointNames == "right-toe-A-rod");
        robot = insertJoint(robot, rightARodInd, 'right-A2', 'Ry', eye(6), zeros(6), 0, 0.01, 0);
        rightBRodInd = find(robot.jointNames == "right-toe-B-rod");
        robot = insertJoint(robot, rightBRodInd, 'right-B2', 'Ry', eye(6), zeros(6), 0, 0.01, 0);
        
        % Add left and right knee rod 2nd DoF        
        leftKneeRodInd = find(robot.jointNames == "left-achilles-rod");
        robot = insertJoint(robot, leftKneeRodInd, 'left-ach2', 'Ry', eye(6), zeros(6), 0, 0.01, 0);
        rightKneeRodInd = find(robot.jointNames == "right-achilles-rod");
        robot = insertJoint(robot, rightKneeRodInd, 'right-ach2', 'Ry', eye(6), zeros(6), 0, 0.01, 0);

        % Fix appearances, specifying tree connection preferences for
        % multi-child nodes
        connectPref = zeros(robot.NB, 1); connectPref([3 6 11 14]) = [4 7 12 15];
        robot = fix_appearances(robot, connectPref);

        % Add right and left rod appearances to aid visualization
        robot.appearance.body{leftARodInd + 1}{2}(2, :) = (rx(-4*pi/180)*ry(-pi/2)*[0; 0; 0.34])';
        robot.appearance.body{leftBRodInd + 1}{2}(2, :) = (rx(-4*pi/180)*ry(-pi/2)*[0; 0; 0.288])';
        robot.appearance.body{rightARodInd + 1}{2}(2, :) = (rx(-4*pi/180)*ry(-pi/2)*[0; 0; 0.34])';
        robot.appearance.body{rightBRodInd + 1}{2}(2, :) = (rx(-4*pi/180)*ry(-pi/2)*[0; 0; 0.288])';
        
        % Add left knee rod and heel spring appearance to aid visualization
        robot.appearance.body{leftKneeRodInd + 1}{2}(2, :) = (rx(-4*pi/180)*ry(-pi/2)*[0; 0; 0.5])';
        leftHeelSpringInd = find(robot.jointNames == "left-heel-spring");
        robot.appearance.body{leftHeelSpringInd}{2}(2, :) = [0.114165332081761  -0.012189531621047  -0.000002087752907]; %#ok<FNDSB>
        
        % Add right knee rod and heel spring appearance to aid
        % visualization
        robot.appearance.body{rightKneeRodInd + 1}{2}(2, :) = (rx(-4*pi/180)*ry(-pi/2)*[0; 0; 0.5])';
        rightHeelSpringInd = find(robot.jointNames == "right-heel-spring");
        robot.appearance.body{rightHeelSpringInd}{2}(2, :) = [0.114165332081761  0.012189531621047  -0.000002087752907]; %#ok<FNDSB>
        
        % Left foot appearance
        % (everything needs to be transformed forward since the foot frame
        % is now the world frame due to changePinPoint
        robot.appearance.base{2} = [0 0 0; 0.01 0.01 0.01];
        leftToeInd = find(robot.jointNames == "left-toe-roll");
        center = [0 -0.0437 -0.0255];
        R = rx(-60 * pi/180);
        p1 = [0.04 0.1175 -0.0015]*R; p2 = [-0.04 -0.1175 -0.0015]*R; 
        p3 = [-0.04 0.1175 -0.0015]*R; p4 = [0.04 -0.1175 -0.0015]*R;
        robot.appearance.body{leftToeInd} = {'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01 ...
            'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01};
        robot.appearance.body{leftToeInd}{2}(2, :) = [0.0179 -0.009551 -0.054164];
        robot.appearance.body{leftToeInd}{5}(2, :) = [-0.0181 -0.009551 -0.054164];
        robot.appearance.body{leftToeInd}{8} =  [center + p1; center + p3];
        robot.appearance.body{leftToeInd}{11} = [center + p1; center + p4];
        robot.appearance.body{leftToeInd}{14} = [center + p2; center + p3];
        robot.appearance.body{leftToeInd}{17} = [center + p2; center + p4];     
        
        % Right foot appearance
        rightToeInd = find(robot.jointNames == "right-toe-roll");
        center = [0 0.0437 -0.0255];
        R = rx(60 * pi/180);
        p1 = [0.04 0.1175 -0.0015]*R; p2 = [-0.04 -0.1175 -0.0015]*R; 
        p3 = [-0.04 0.1175 -0.0015]*R; p4 = [0.04 -0.1175 -0.0015]*R;
        robot.appearance.body{rightToeInd} = {'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01 ...
            'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01, 'cyl', zeros(2, 3), 0.01};
        robot.appearance.body{rightToeInd}{2}(2, :) = [0.0179 0.009551 -0.054164];
        robot.appearance.body{rightToeInd}{5}(2, :) = [-0.0181 0.009551 -0.054164];
        robot.appearance.body{rightToeInd}{8} =  [center + p1; center + p3];
        robot.appearance.body{rightToeInd}{11} = [center + p1; center + p4];
        robot.appearance.body{rightToeInd}{14} = [center + p2; center + p3];
        robot.appearance.body{rightToeInd}{17} = [center + p2; center + p4];

        % The constrainted joints need to be in A A-rod A2 B B-rod B2 pitch
        % roll order. These permutation matrices help extract the appropriate
        % joints in the appropriate order.
        left_toe_order = ["left-toe-A" "left-toe-B" ...
                          "left-toe-A-rod" "left-A2" "left-toe-B-rod" "left-B2" ...
                          "left-toe-pitch" "left-toe-roll"];
        right_toe_order = ["right-toe-A" "right-toe-B" ...
                          "right-toe-A-rod" "right-A2" "right-toe-B-rod" "right-B2" ...
                          "right-toe-pitch" "right-toe-roll"];
        left_knee_order = ["left-shin" "left-knee" "left-heel-spring" ...
                           "left-tarsus" "left-achilles-rod" "left-ach2"];
        right_knee_order = ["right-shin" "right-knee" "right-heel-spring" ...
                           "right-tarsus" "right-achilles-rod" "right-ach2"];
        robot.leftToePerm = zeros(8, robot.NB); robot.rightToePerm = zeros(8, robot.NB);
        robot.leftKneePerm = zeros(6, robot.NB); robot.rightKneePerm = zeros(6, robot.NB);
        for i = 1:length(left_toe_order)
            robot.leftToePerm(i, find(left_toe_order(i) == robot.jointNames)) = 1;
        end
        for i = 1:length(right_toe_order)
            robot.rightToePerm(i, find(right_toe_order(i) == robot.jointNames)) = 1;
        end
        for i = 1:length(left_knee_order)
            robot.leftKneePerm(i, find(left_knee_order(i) == robot.jointNames)) = 1;
        end
        for i = 1:length(right_knee_order)
            robot.rightKneePerm(i, find(right_knee_order(i) == robot.jointNames)) = 1;
        end
        robot.leftToeLoc = robot.leftToePerm*[1:robot.NB]';
        robot.rightToeLoc = robot.rightToePerm*[1:robot.NB]';
        robot.leftKneeLoc = robot.leftKneePerm*[1:robot.NB]';
        robot.rightKneeLoc = robot.rightKneePerm*[1:robot.NB]';
        
        % Joint classifications
        robot.actNames = ["left-knee" "left-hip-pitch" "left-hip-yaw" "left-hip-roll" ...
        "right-hip-roll" "right-hip-yaw" "right-hip-pitch" "right-knee" ...
        "left-toe-A" "left-toe-B" "right-toe-A" "right-toe-B"];
        robot.unactNames = ["left-toe-roll" "left-toe-pitch" "left-tarsus" "left-shin" ...
        "right-shin" "right-tarsus" "right-toe-pitch" "right-toe-roll" ...
        "left-toe-A-rod" "left-A2" "left-toe-B-rod" "left-B2" ...
        "right-toe-A-rod" "right-A2" "right-toe-B-rod" "right-B2" ...
        "left-achilles-rod" "left-ach2" "left-heel-spring" ...
        "right-achilles-rod" "right-ach2" "right-heel-spring"];
        robot.indepNames = [robot.actNames "left-shin" "right-shin" "left-heel-spring" "right-heel-spring"];
        robot.depNames = robot.unactNames([1:3 6:18 20:21]);
        robot.agStateNames = ["left-toe-roll" "left-toe-pitch" "left-tarsus" "left-shin" ...
        "left-knee" "left-hip-pitch" "left-hip-yaw" "left-hip-roll" ...
        "right-hip-roll" "right-hip-yaw" "right-hip-pitch" "right-knee" ...
        "right-shin" "right-tarsus" "right-toe-pitch" "right-toe-roll" ...
        "left-toe-A" "left-toe-B" "right-toe-A" "right-toe-B" ...
        "left-heel-spring" "right-heel-spring"];
        robot.non_agStateNames = ["left-toe-A-rod" "left-A2" "left-toe-B-rod" "left-B2" ...
        "right-toe-A-rod" "right-A2" "right-toe-B-rod" "right-B2" ...
        "left-achilles-rod" "left-ach2" ...
        "right-achilles-rod" "right-ach2"];
    
        [~, robot.actJoints] = intersect(robot.jointNames, robot.actNames, "stable"); robot.actJoints = robot.actJoints';
        [~, robot.unactJoints] = intersect(robot.jointNames, robot.unactNames, "stable"); robot.unactJoints = robot.unactJoints';
        [~, robot.indepJoints] = intersect(robot.jointNames, robot.indepNames, "stable"); robot.indepJoints = robot.indepJoints';
        [~, robot.depJoints] = intersect(robot.jointNames, robot.depNames, "stable"); robot.depJoints = robot.depJoints';
        [~, robot.agStateJoints] = intersect(robot.jointNames, robot.agStateNames, "stable"); robot.agStateJoints = robot.agStateJoints';
        [~, robot.non_agStateJoints] = intersect(robot.jointNames, robot.non_agStateNames, "stable"); robot.non_agStateJoints = robot.non_agStateJoints';
        
%         robot.act = [4:11 15:18 29 32];
%         robot.unact = [1:3 12:14 19:28 30:31];
%         robot.nonIK_joints = [1:18 29 32];
%         robot.IK_joints = [19:28 30:31];
% 
        robot.FDcrb_constraints = @FDcrb_constraints_FloatingBaseDigit;
        robot.ID_constraints = @ID_constraints_FloatingBaseDigit;
        robot.modelMod = @modelMod_FloatingBaseDigit;
        
        robot.IK_constraints = @IK_constraints_FloatingBaseDigit;
%         robot.jointConstraints = @jointConstraints;
        robot.FK_X = @FK_X_FloatingBaseDigit;
        robot.FK = @FK_FloatingBaseDigit;
        robot.FK_Vel = @FK_Vel_FloatingBaseDigit;
        
        % Indices for all the FK joints in IK_constraints
        robot = addIKJointInds(robot);
        
        % Forward dynamics stablization gains
        robot.kP = 0;
        robot.kD = 0;
        
        robot.posConstraints = @posConstraints_FloatingBaseDigit;
%         robot.velConstraints = @velConstraints_FloatingBaseDigit;
%         robot.accelConstraints = @accelConstraints_FloatingBaseDigit;
        robot.fillUnactJoints = @fillUnactJoints_FloatingBaseDigit;
        
        % Helper functions
        robot.getDynamics = @getDynamics;
        robot.getc = @getc_FloatingBaseDigit;
        robot.getJ = @getJ_FloatingBaseDigit;
        robot.getc_d = @getc_d_FloatingBaseDigit;
        robot.getJ_d = @getJ_d_FloatingBaseDigit;

        
        % Actuator limits
        robot.limits = zeros(robot.NB, 1);
        robot.limits(robot.actJoints) = [126.682; 79.1765; 216.928; 231.317; ...
            126.682; 79.1765; 216.928; 231.317; 41.9759; 41.9759; 41.9759; 41.9759];

        robot.shinSpringStiffness = 6335.330329;
        robot.heelSpringStiffness = 6335.330329;
        % robot.heelSpringStiffness = 6335.330329;  % 8254.347872;
%         
%         % Test code
%         robot.verifyFD_ID = @verifyFD_ID_FloatingBaseDigit;
%         robot.generateMuJoCoIC = @generateMuJoCoIC_FloatingBaseDigit;
%         
%         save(path, 'robot');
%         
%         % Export to C++
%         convertRobotToCppFile(robot, "robot_description/digit/floatingBaseDigit_Full.txt");
%     else
%         load(path);
%     end
    
    % Disable friction. If you re-enable this, you need to 
    robot = rmfield(robot, {'friction'});

    robot.name = 'FloatingDigit';
end

function [q_dd, lambda] = FDcrb_constraints_FloatingBaseDigit(robot, q, q_d, tau)
    % Dynamics
    [H, C, J, J_d, stabTerm] = getDynamics(robot, q, q_d);
    
    % Zero out unactuated torques
%     tau(robot.unact) = 0;
    
    % Subtract damping forces
    if isfield(robot, 'damping')
        Bq_d = robot.damping .* q_d;
    else
        Bq_d = zeros(robot.NB,1);
    end
    
    % Subtract friction froces
    Fq_d = zeros(robot.NB, 1);
    if isfield(robot, 'friction')
        for i = 1:robot.NB
            if abs(q_d(i)) > 10^-8
                Fq_d(i) = robot.friction(i) * sign(q_d(i));
            end
        end
    end

    % Spring forces
    shins = [robot.rightKneeLoc(1) robot.leftKneeLoc(1)];
    heels = [robot.rightKneeLoc(3) robot.leftKneeLoc(3)];
    
    tau(shins) = tau(shins) - robot.shinSpringStiffness*q(shins);
    tau(heels) = tau(heels) - robot.heelSpringStiffness*q(heels);
    
    % Additional forces not associated with the original model
    addF = robot.modelMod(robot, q, q_d, tau);

    % Solve forward dynamics
    H_bar = [H -J'; J zeros(size(J, 1))];
    % disp('size')
    % disp([size(tau),size(C),size(Bq_d),size(Fq_d),size(addF),size(stabTerm),size(J_d),size(q_d)])
    C_bar = [tau - C - Bq_d - Fq_d + addF; stabTerm-J_d*q_d];
    q_ddAndLam = H_bar\C_bar;
    
    q_dd = q_ddAndLam(1:robot.NB);
%     q_dd_tmp = H\(tau-C);

    lambda = q_ddAndLam(robot.NB+1:end);
%     disp('size');
%     disp([ size(tau),size(C),size(lambda),size(q_dd_tmp)]);
end

function [addF] = modelMod_FloatingBaseDigit(robot, q, q_d, tau)
    addF = zeros(robot.NB, 1);
end

function [tau] = ID_constraints_FloatingBaseDigit(robot, q, q_d, q_dd)
    % Dynamics
    [~, ~, J, ~, ~] = getDynamics(robot, q, q_d);
    
    %Spanning tree inverse dynamics tau_span = J'lam + tau
    tau_span = ID(robot, q, q_d, q_dd);
    
    %Solve for actuator torques
    tau = zeros(robot.NB, 1);

    % the difference between the following two commands won't be large
    % since J(:, robot.depJoints) is a full rank square matrix
%     tau(robot.indepJoints) = -J(:, robot.indepJoints)'*inv(J(:, robot.depJoints)')* ...
%                     tau_span(robot.depJoints) + tau_span(robot.indepJoints);
    tau(robot.indepJoints) = -(J(:, robot.depJoints) \ J(:, robot.indepJoints))' * tau_span(robot.depJoints) ...
                    + tau_span(robot.indepJoints);
    
    shins = [robot.rightKneeLoc(1) robot.leftKneeLoc(1)];
    heels = [robot.rightKneeLoc(3) robot.leftKneeLoc(3)];
    tau(shins) = tau(shins) + robot.shinSpringStiffness*q(shins);
    tau(heels) = tau(heels) + robot.heelSpringStiffness*q(heels);
    
    % Additional forces not associated with the original model
    addF = robot.modelMod(robot, q, q_d);
    
    tau = tau - addF;
end

function [X] = FK_X_FloatingBaseDigit(robot, q, jointInd, frameInd, endX, includeFirstJoint)
% Calculate forward kinematics to the joint ind times the
    % optional endeffector transform, expressed in frameInd
    if nargin < 6
        includeFirstJoint = true;
    end
    
    if nargin < 5 
        endX = eye(6);
    elseif endX == -1 %Use appearance data
        assert(isequal(robot.appearance.body{jointInd}{1}, 'cyl'), sprintf("Attempted " + ...
            "to generate endX from appearance data for joint %d but " + ...
            "robot.appearance.body{%d}{1} is not a cylinder", jointInd, jointInd));
        if (length(robot.appearance.body{1}) ~= 3)
            warning("robot.appearance.body{%d} seems to contain multiple " + ...
                "drawing instructions. Only the first will be used.", jointInd);
        end
        endX = plux(eye(3), robot.appearance.body{jointInd}{2}(2, :)'); %Endpoint of cylinder
    end
    
    if nargin < 4
        frameInd = 0;
    end

    i = jointInd;
    ind = [];
    while i ~= frameInd
        ind = [i, ind];
        i = robot.parent(i);
    end
    
    X = eye(6);
    if isequal(class(q), 'sym')
        X = sym(X);
    end
    
    % Xtree is the coordinate transform from parent to the child
    for i = ind
        if i == jointInd && ~includeFirstJoint
            X = robot.Xtree{i} * X;
        else
            X = (  jcalc( robot.jtype{i}, q(i) ) * robot.Xtree{i}  ) * X;
        end
    end
    
    X = endX * X;
end

function [p, quat] = FK_FloatingBaseDigit(robot, q, jointInd, frameInd, endX)
    % Calculate forward kinematics to the joint ind times the
    % optional endeffector transform, expressed in frameInd

    if nargin < 5 
        endX = eye(6);
    elseif endX == -1 %Use appearance data
        assert(isequal(robot.appearance.body{jointInd}{1}, 'cyl'), sprintf("Attempted " + ...
            "to generate endX from appearance data for joint %d but " + ...
            "robot.appearance.body{%d}{1} is not a cylinder", jointInd, jointInd));
        if (length(robot.appearance.body{1}) ~= 3)
            warning("robot.appearance.body{%d} seems to contain multiple " + ...
                "drawing instructions. Only the first will be used.", jointInd);
        end
        endX = plux(eye(3), robot.appearance.body{jointInd}{2}(2, :)'); %Endpoint of cylinder
    end
    
    if nargin < 4
        frameInd = 0;
    end

    i = jointInd;
    ind = [];
    while i ~= frameInd
        ind = [i, ind];
        i = robot.parent(i);
    end
    
    X = eye(6);
    if isequal(class(q), 'sym')
        X = sym(X);
    end
    
    % Xtree is the coordinate transform from parent to the child
    for i = ind
        X = (  jcalc( robot.jtype{i}, q(i) ) * robot.Xtree{i}  ) * X;
    end
    % Plux gives the translation vector (-R*p), which is A's origin in B's
    % frame. The FK is calculated backward, we need to invert the
    % transformation. Since Plux gives the translation vector (-R*p), which
    % is A's origin in B's frame, we do not need to invert the translation
    % vector and only invert the rotational matrix, R. 
    [R, p] = plux(endX * X);
    
    if nargout >= 2
        quat = rq(R');
    end
end

function v_body = FK_Vel_FloatingBaseDigit(robot, q, q_d, jointInd, frameInd, endX)
    if nargin < 6
        endX = eye(6);
    end

    i = jointInd; 
    ind = [];
    
    while i ~= frameInd
        ind = [i, ind];
        i = robot.parent(i);
    end

    X = eye(6);
    % Xtree is the coordinate transform from parent to the child
    for i = ind
        X = (  jcalc( robot.jtype{i}, q(i) ) * robot.Xtree{i}  ) * X;
    end


    for i = ind
        [ XJ, S{i} ] = jcalc( robot.jtype{i}, q(i));
        vJ = S{i}*q_d(i);
        Xup{i} = XJ * robot.Xtree{i};
        if robot.parent(i) == 0
            v{i} = vJ;
        else
            v{i} = Xup{i}*v{robot.parent(i)} + vJ;
        end
    end
%     v_body =  inv(X*endX) * v{jointInd};
    v_body =  (X*endX) \ v{jointInd};

end

function [H, C, J, J_d, stabTerm] = getDynamics(robot, q, q_d)
    % Dynamics
    [H, C] = HandC(robot, q, q_d);
    
    %Jacobian and derivative
    J = robot.getJ(robot, q);
    J_d = robot.getJ_d(robot, q, q_d);
    
    %Constraint and derivative
    c = robot.getc(robot, q);
    c_d = robot.getc_d(robot, q, q_d);
    
    % Constraint stabilization term
    stabTerm = -robot.kP*c - robot.kD*c_d;
end

function [c] = getc_FloatingBaseDigit(robot, q)
    % Create c for right foot
    r_toe_c = floatingBaseDigit_rightToe_c(q(robot.rightToeLoc)); 
    
    % Create c for left foot
    l_toe_c = floatingBaseDigit_leftToe_c(q(robot.leftToeLoc)); 
    
    % Create c for right knee
    r_knee_c = floatingBaseDigit_rightKnee_c(q(robot.rightKneeLoc));
    
    % Create c for left knee
    l_knee_c = floatingBaseDigit_leftKnee_c(q(robot.leftKneeLoc));
    
    % Assemble full c
    c = [r_toe_c; l_toe_c; r_knee_c; l_knee_c];
end

function [J] = getJ_FloatingBaseDigit(robot, q)
    % Create Jacobian for right foot
    r_toe_J = floatingBaseDigit_rightToe_J(q(robot.rightToeLoc))*robot.rightToePerm; 
    
    % Create Jacobian for left foot
    l_toe_J = floatingBaseDigit_leftToe_J(q(robot.leftToeLoc))*robot.leftToePerm; 
    
    % Create Jacobian for right knee
    r_knee_J = floatingBaseDigit_rightKnee_J(q(robot.rightKneeLoc))*robot.rightKneePerm;
    
    % Create Jacobian for left knee
    l_knee_J = floatingBaseDigit_leftKnee_J(q(robot.leftKneeLoc))*robot.leftKneePerm;
    
    % Assemble full Jacobian
    J = [r_toe_J; l_toe_J; r_knee_J; l_knee_J];
end

function [c_d] = getc_d_FloatingBaseDigit(robot, q, q_d)
    % Create c derivative for right foot
    r_toe_c_d = floatingBaseDigit_rightToe_c_d(q(robot.rightToeLoc), q_d(robot.rightToeLoc)); 
    
    % Create c derivative for left foot
    l_toe_c_d = floatingBaseDigit_leftToe_c_d(q(robot.leftToeLoc), q_d(robot.leftToeLoc)); 
    
    % Create c derivative for right knee
    r_knee_c_d = floatingBaseDigit_rightKnee_c_d(q(robot.rightKneeLoc), q_d(robot.rightKneeLoc));
    
    % Create c derivative for left knee
    l_knee_c_d = floatingBaseDigit_leftKnee_c_d(q(robot.leftKneeLoc), q_d(robot.leftKneeLoc));
    
    % Assemble full c derivative
    c_d = [r_toe_c_d; l_toe_c_d; r_knee_c_d; l_knee_c_d];
end

function [J_d] = getJ_d_FloatingBaseDigit(robot, q, q_d)
    % Create Jacobian derivative for right foot
    r_toe_J_d = floatingBaseDigit_rightToe_J_d(q(robot.rightToeLoc), q_d(robot.rightToeLoc))*robot.rightToePerm; 
    
    % Create Jacobian derivative for left foot
    l_toe_J_d = floatingBaseDigit_leftToe_J_d(q(robot.leftToeLoc), q_d(robot.leftToeLoc))*robot.leftToePerm; 
    
    % Create Jacobian derivative for right knee
    r_knee_J_d = floatingBaseDigit_rightKnee_J_d(q(robot.rightKneeLoc), q_d(robot.rightKneeLoc))*robot.rightKneePerm;
    
    % Create Jacobian derivative for left knee
    l_knee_J_d = floatingBaseDigit_leftKnee_J_d(q(robot.leftKneeLoc), q_d(robot.leftKneeLoc))*robot.leftKneePerm;
    
    % Assemble full Jacobian derivative
    J_d = [r_toe_J_d; l_toe_J_d; r_knee_J_d; l_knee_J_d];
end

function [q, q_d, q_dd] = fillUnactJoints_FloatingBaseDigit(robot, q, q_d, q_dd)
    for i = 1:size(q, 2)
        q(robot.leftToeLoc, i) = mapFunc_floatingBaseDigit_leftToe(q(robot.leftToeLoc(1:2), i));
        q(robot.rightToeLoc, i) = mapFunc_floatingBaseDigit_rightToe(q(robot.rightToeLoc(1:2), i));
    
        q(robot.leftKneeLoc, i) = mapFunc_floatingBaseDigit_leftKnee(q(robot.leftKneeLoc(1:3), i));
        q(robot.rightKneeLoc, i) = mapFunc_floatingBaseDigit_rightKnee(q(robot.rightKneeLoc(1:3), i));

        % Check constraints
        c = robot.posConstraints(robot, q(robot.depJoints, i), q(:, i));
        if (max(abs(c)) > 1e-2)
            warning("FillUnactJoints didn't not calculate a solution for q with constraint violation less than 1e-2");
        end

        if nargout >= 2 && nargin >= 3 % Calc q_d
            J = robot.getJ(robot, q(:, i));
            q_d(robot.depJoints, i) = -J(:, robot.depJoints) \ J(:, robot.indepJoints)*...
                                    q_d(robot.indepJoints, i);  

            if nargout >= 3 && nargin >= 4 % Calc q_dd
                J_d = robot.getJ_d(robot, q(:, i), q_d(:, i));
                q_dd(robot.depJoints, i) = -J(:, robot.depJoints) \ (J(:, robot.indepJoints)*...
                                    q_dd(robot.indepJoints, i) + J_d*q_d(:, i));
            end
        end
    end
end

function [q, q_d, q_dd] = IK_constraints_FloatingBaseDigit(robot, q, q_d, q_dd)
    nin = nargin;
    non = nargout;
    if size(q, 2) < 1000 % Only use parfor for large amounts of data
        for i = 1:size(q, 2)
            % Right knee IK
            q(:, i) = rodIK(robot, q(:, i), robot.rightKneeLoc(5:6), ...
                            robot.IK_inds.rightKnee, plux(eye(3), [0.113789 0.011056 0]'));

            % Left knee IK
            q(:, i) = rodIK(robot, q(:, i), robot.leftKneeLoc(5:6), ...
                            robot.IK_inds.leftKnee, plux(eye(3), [0.113789 -0.011056 0]'));

            % Right toe IK
            q(:, i) = rodIK(robot, q(:, i), robot.rightToeLoc(3:4), ...
                            robot.IK_inds.rightToeA, plux(eye(3), [0.0179 0.009551 -0.054164]'));
            q(:, i) = rodIK(robot, q(:, i), robot.rightToeLoc(5:6),  ...
                            robot.IK_inds.rightToeB, plux(eye(3), [-0.0181 0.009551 -0.054164]'));

            % Left toe IK
            q(:, i) = rodIK(robot, q(:, i), robot.leftToeLoc(3:4), ...
                            robot.IK_inds.leftToeA, plux(eye(3), [0.0179 -0.009551 -0.054164]'));
            q(:, i) = rodIK(robot, q(:, i), robot.leftToeLoc(5:6), ...
                            robot.IK_inds.leftToeB, plux(eye(3), [-0.0181 -0.009551 -0.054164]'));

            if non >= 2 && nin >= 3 % Calc q_d
                J = robot.getJ(robot, q(:, i));
                q_d_temp(robot.depJoints) = -inv(J(:, robot.depJoints))*J(:, robot.indepJoints)*...
                                        q_d(robot.indepJoints, i);  
                q_d(robot.non_agStateJoints, i) = q_d_temp(robot.non_agStateJoints);

                if non >= 3 && nin >= 4 % Calc q_dd
                    J_d = robot.getJ_d(robot, q(:, i), q_d(:, i));
                    q_dd_temp(robot.depJoints) = -inv(J(:, robot.depJoints))*(J(:, robot.indepJoints)*...
                                        q_dd(robot.indepJoints, i) + J_d*q_d(:, i));
                    q_dd(robot.non_agStateJoints, i) = q_dd_temp(robot.non_agStateJoints);
                end
            end
        end
    else       
        if nin < 3
            q_d = q;
        end
        if nin < 4
            q_dd = q;
        end
        parfor i = 1:size(q, 2)
            % Right knee IK
            q(:, i) = rodIK(robot, q(:, i), robot.rightKneeLoc(5:6), ...
                            robot.IK_inds.rightKnee, plux(eye(3), [0.113789 0.011056 0]'));

            % Left knee IK
            q(:, i) = rodIK(robot, q(:, i), robot.leftKneeLoc(5:6), ...
                            robot.IK_inds.leftKnee, plux(eye(3), [0.113789 -0.011056 0]'));

            % Right toe IK
            q(:, i) = rodIK(robot, q(:, i), robot.rightToeLoc(3:4), ...
                            robot.IK_inds.rightToeA, plux(eye(3), [0.0179 0.009551 -0.054164]'));
            q(:, i) = rodIK(robot, q(:, i), robot.rightToeLoc(5:6),  ...
                            robot.IK_inds.rightToeB, plux(eye(3), [-0.0181 0.009551 -0.054164]'));

            % Left toe IK
            q(:, i) = rodIK(robot, q(:, i), robot.leftToeLoc(3:4), ...
                            robot.IK_inds.leftToeA, plux(eye(3), [0.0179 -0.009551 -0.054164]'));
            q(:, i) = rodIK(robot, q(:, i), robot.leftToeLoc(5:6), ...
                            robot.IK_inds.leftToeB, plux(eye(3), [-0.0181 -0.009551 -0.054164]'));
            
            if non >= 2 && nin >= 2 % Calc q_d
                local_q_d = q_d(:, i);
                q_d_temp = zeros(robot.NB, 1);
                J = robot.getJ(robot, q(:, i));
                q_d_temp(robot.depJoints) = -inv(J(:, robot.depJoints))*J(:, robot.indepJoints)*...
                                        local_q_d(robot.indepJoints);  
                local_q_d(robot.non_agStateJoints) = q_d_temp(robot.non_agStateJoints);
                q_d(:, i) = local_q_d;

                if non == 3 && nin == 3 % Calc q_dd
                    local_q_dd = q_dd(:, i);
                    q_dd_temp = zeros(robot.NB, 1);
                    J_d = robot.getJ_d(robot, q(:, i), q_d(:, i));
                    q_dd_temp(robot.depJoints) = -inv(J(:, robot.depJoints))*(J(:, robot.indepJoints)*...
                                        local_q_dd(robot.indepJoints) + J_d*q_d(:, i));
                    local_q_dd(robot.non_agStateJoints) = q_dd_temp(robot.non_agStateJoints);
                    q_dd(:, i) = local_q_dd(:, i);
                end
            end
        end
    end
end

function [robot] = addIKJointInds(robot)
    robot.IK_inds = struct();

    robot.IK_inds.rightKnee = struct();
    robot.IK_inds.rightKnee.rodTip = find(robot.jointNames == "right-ach2");
    robot.IK_inds.rightKnee.anchor = find(robot.jointNames == "right-heel-spring");
    robot.IK_inds.rightKnee.rodPivot = find(robot.jointNames == "right-achilles-rod");
    robot.IK_inds.rightKnee.conJoint = find(robot.jointNames == "right-hip-pitch");

    robot.IK_inds.leftKnee = struct();
    robot.IK_inds.leftKnee.rodTip = find(robot.jointNames == "left-ach2");
    robot.IK_inds.leftKnee.anchor = find(robot.jointNames == "left-heel-spring");
    robot.IK_inds.leftKnee.rodPivot = find(robot.jointNames == "left-achilles-rod");
    robot.IK_inds.leftKnee.conJoint = find(robot.jointNames == "left-hip-pitch");

    robot.IK_inds.rightToeA = struct();
    robot.IK_inds.rightToeA.rodTip = find(robot.jointNames == "right-A2");
    robot.IK_inds.rightToeA.anchor = find(robot.jointNames == "right-toe-roll");
    robot.IK_inds.rightToeA.rodPivot = find(robot.jointNames == "right-toe-A-rod");
    robot.IK_inds.rightToeA.conJoint = find(robot.jointNames == "right-tarsus");

    robot.IK_inds.leftToeA = struct();
    robot.IK_inds.leftToeA.rodTip = find(robot.jointNames == "left-A2");
    robot.IK_inds.leftToeA.anchor = find(robot.jointNames == "left-toe-roll");
    robot.IK_inds.leftToeA.rodPivot = find(robot.jointNames == "left-toe-A-rod");
    robot.IK_inds.leftToeA.conJoint = find(robot.jointNames == "left-tarsus");

    robot.IK_inds.rightToeB = struct();
    robot.IK_inds.rightToeB.rodTip = find(robot.jointNames == "right-B2");
    robot.IK_inds.rightToeB.anchor = find(robot.jointNames == "right-toe-roll");
    robot.IK_inds.rightToeB.rodPivot = find(robot.jointNames == "right-toe-B-rod");
    robot.IK_inds.rightToeB.conJoint = find(robot.jointNames == "right-tarsus");

    robot.IK_inds.leftToeB = struct();
    robot.IK_inds.leftToeB.rodTip = find(robot.jointNames == "left-B2");
    robot.IK_inds.leftToeB.anchor = find(robot.jointNames == "left-toe-roll");
    robot.IK_inds.leftToeB.rodPivot = find(robot.jointNames == "left-toe-B-rod");
    robot.IK_inds.leftToeB.conJoint = find(robot.jointNames == "left-tarsus");
end

function [q] = rodIK(robot, q, inds, fk_inds, anchorX)
    q(inds) = 0;
    
    anchor = robot.FK_X(robot, q, fk_inds.anchor, ... 
                      fk_inds.conJoint, anchorX);

    % Calculate rodTip with the rod angles zeroed
    rodTipZero = robot.FK_X(robot, q, fk_inds.rodTip, ...
                      fk_inds.conJoint, -1);

    % Try shifting to a different reference frame
    rodTipEq = robot.FK_X(robot, q, fk_inds.rodPivot, ...
                      fk_inds.conJoint);

    anchor = pluho(rodTipEq / anchor);
    rodTipZero = pluho(rodTipEq / rodTipZero);

    q(inds) = padenKahanSubproblem2([0; 0; 1], [0; 1; 0], rodTipZero(1:3, 4), anchor(1:3, 4), zeros(3, 1));
end

function [c, J] = posConstraints_FloatingBaseDigit(robot, x, q)
    q(robot.depJoints) = x;
    
    c = robot.getc(robot, q);
    J = robot.getJ(robot, q);
    J = J(:, robot.depJoints);
end