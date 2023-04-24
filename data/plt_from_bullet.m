clc; clear;clf;
f=figure(1);
f.Position(3:4)=[1000 500];
% robot = createLeftFootDigit();
% robot = createFloatingBaseDigit();
%% plot data from matlab
% Array=csvread('/home/chankyo/roahm/ROAHM_Robust_Control/MuJoCo/bin/data/chankyo-assignment2/digitBasePinned_test.csv',1,0);
% n_d1=max(find(Array(:,end)<=0.3));
% time = Array(1:n_d1, end);
% qd = zeros(n_d1, size(robot.actNames,2));
% q = zeros(n_d1, size(robot.actNames,2));
% for i =1:length(robot.actJoints)
%     q(:,i)=Array(1:n_d1,robot.actJoints(i)); % position
%     qd(:,i)=Array(1:n_d1,34+robot.actJoints(i)); % velocity
% end
% 
% % for i = 1:size(robot.actNames,2)
% %     subplot(2,6,i);
% %     plot(time, q(:,i));hold on;
% %     legend('Matlab Data');
% %     title(robot.actNames(i));
% % end
% % legend(robot.actNames);

%% plot data from mujoco
Array2=readtable('/Users/ckkim/Chankyo Kim/Michigan/pybullet/data/mjc-bullet/digitBasePinned_test.csv');
Array2=table2array(Array2);
% time2 = Array2(:, end);
% n_d=max(find(Array2(:,end)<=0.3));
% time2 = Array2(1:n_d, end);
qd2 = zeros(height(Array2), 28);
q2 = zeros(height(Array2), 28);


%velocity
% for i = 1:length(robot.actJoints)
%     qd2(:,i)=Array2(:,3*(robot.actJoints(i)-1)+2);cd ..
% end
% figure(2);


for i = 1:28
    q2(:,i)=Array2(:,i); %position
%     qd2(:,i)=Array2(1:height(Array2),34+robot.actJoints(i)); %velocity
end
%position
% q2(:,1)=Array2(1:n_d,5); 
% q2(:,2)=Array2(1:n_d,3); 
% q2(:,3)=Array2(1:n_d,2); 
% q2(:,4)=Array2(1:n_d,1); 
% q2(:,5)=Array2(1:n_d,19); 
% q2(:,6)=Array2(1:n_d,20); 
% q2(:,7)=Array2(1:n_d,21); 
% q2(:,8)=Array2(1:n_d,23); 
% q2(:,9)=Array2(1:n_d,9); 
% q2(:,10)=Array2(1:n_d,11); 
% q2(:,11)=Array2(1:n_d,27); 
% q2(:,12)=Array2(1:n_d,29); 

% for i = 1:size(robot.actNames,2)
%     subplot(2,6,i);
%     plot(time2, q2(:,i));hold on;legend(robot.actNames(i));
%     legend('Mujoco Data');
% end
sz = round(length(q2)/15);
actNames={'left-hip-roll';'left-hip-yaw';'left-hip-pitch';'left-knee';'right-hip-roll';'right-hip-yaw';'right-hip-pitch';'right-knee'};
for i = 1:4
    subplot(2,4,i);
%     plot(time, q(:,i));hold on; %legend('Matlab');%position
    plot(1:sz, q2(1:sz,i));hold on;% legend('Mujoco');%position
%     legend('Position: Matlab', 'Position: Mujoco'); %Position
    legend('Position: Bullet'); %Position
%     plot(time, qd(:,i));hold on; plot(time2, qd2(:,i));hold on; %velocity
%     legend('Velocity: Matlab', 'Velocity: Mujoco'); %Velocity
%     title(robot.actNames(i));
    title(actNames(i));
    xlabel('time');ylabel('joint position [rad]');
end

for i = 5:8
    subplot(2,4,i);
%     plot(time, q(:,i));hold on; %legend('Matlab');%position
    plot(1:sz, q2(1:sz,i+9));hold on;% legend('Mujoco');%position
%     legend('Position: Matlab', 'Position: Mujoco'); %Position
%     legend('Position: Bullet','Location','northwestoutside'); %Position
%     plot(time, qd(:,i));hold on; plot(time2, qd2(:,i));hold on; %velocity
%     legend('Velocity: Matlab', 'Velocity: Mujoco'); %Velocity
    legend('Position: Bullet'); %Position
    title(actNames(i));
    xlabel('time');ylabel('joint position [rad]');
end