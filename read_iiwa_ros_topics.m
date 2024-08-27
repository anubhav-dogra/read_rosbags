clear all
clc
close all
bag = rosbag("cartesian_wrench_test1.bag");
% bag = rosbag("force_calibration1.bag");
% bag = rosbag("torque_diff.bag");
bSel1 = select(bag,'Topic','/additional_outputs');
bSel2 = select(bag,'Topic','/iiwa/joint_states');
bSel3 = select(bag,'Topic','/cartesian_wrench_tool');
bSel4 = select(bag,'Topic','/tool_link_ee_pose');
msgStructs1 = readMessages(bSel1,'DataFormat','struct');
msgStructs2 = readMessages(bSel2,'DataFormat','struct');
msgStructs3 = readMessages(bSel3,'DataFormat','struct');
msgStructs4 = readMessages(bSel4,'DataFormat','struct');
fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs3);
z = cellfun(@(m) double(m.Transform.Translation.Z),msgStructs4);
tau_ext_read = cellfun(@(m) double(m.ExternalTorques.Data),msgStructs1,...
    'UniformOutput',false);
tau_cmd_read = cellfun(@(m) double(m.CommandedTorques.Data),msgStructs1,...
    'UniformOutput',false);
effort_read = cellfun(@(m) double(m.Effort),msgStructs2,'UniformOutput',false);
q_read = cellfun(@(m) double(m.Position),msgStructs2,'UniformOutput',false);
qdot_read = cellfun(@(m) double(m.Velocity),msgStructs2,'UniformOutput',false);
% dps = size(q_read,1);
dps = min([size(q_read,1),size(effort_read,1),size(tau_ext_read,1)]);
    for i = 1:dps
        tau_ext(:,i) = tau_ext_read{i,1}; % tau_read is saved in column format
        effort(:,i) = effort_read{i,1}; % tau_read is saved in column format
        q(:,i) = q_read{i,1};
        tau_cmd(:,i) = tau_cmd_read{i,1};
        qdot(:,i) = qdot_read{i,1};
    end
%%
subplot(2,1,1)
plot(fz,'LineWidth',2.0)
set(gca,'LineWidth',0.75,'FontSize',16,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');
xlabel('Time (sec)')
ylabel('Force (N)')
% grid on
% box on
subplot(2,1,2)
plot(1000*z,'LineWidth',2.0)
set(gca,'LineWidth',0.75,'FontSize',16,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');
xlabel('Time (sec)')
ylabel('EEF Position (Z (mm))')
% grid on
% box on
[b,a] = butter(2,2/(43/2));
fz_ = filter(b, a, fz);
subplot(2,1,1)
hold on

plot(fz_,'LineWidth',2.0)
%%
figure
[b,a] = butter(2, 2 / (200 / 2),'low');
effort_ = filtfilt(b, a, effort);
plot(effort(2,:))
hold on
plot(effort_(2,:))
% for i = 3:7
% subplot(5,1,i-2)
% plot(tau_ext_(i,:),'LineWidth',2.0);
% end
% subplot(2,1,1)
% hold on
% tau_ext_read = cellfun(@(m) double(m.ExternalTorques.Data),msgStructs,...
%     'UniformOutput',false);
% q_read = cellfun(@(m) double(m.Position),msgStructs1,'UniformOutput',false);
% qdot_read = cellfun(@(m) double(m.Velocity),msgStructs1,'UniformOutput',false);
% effort_read = cellfun(@(m) double(m.Effort),msgStructs1,'UniformOutput',false);
% dps = min([size(q_read,1),size(effort_read,1),size(tau_ext_read,1)]);
%     for i = 1:dps
%         tau_ext(:,i,j) = tau_ext_read{i,1}; % tau_read is saved in column format
%         effort(:,i,j) = effort_read{i,1}; % tau_read is saved in column format
%         q(:,i,j) = q_read{i,1};
%         qdot(:,i,j) = qdot_read{i,1};
%     end
%     for i = 1:7
%         tau_ext_m(j,i) = mean(tau_ext(i,:,j));
%         effort_m(j,i) = mean(effort(i,:,j));
%         q_m(j,i) = mean(q(i,:,j));
%         qdot_m(j,i) = mean(qdot(i,:,j));
%         tau_ext_v(j,i) = var(tau_ext(i,:,j));
%         % disp("joint index" + num2str(i));
%         % disp("mean:   " + num2str(mean(tau(i,:))))
%         % disp("variance:   " + num2str(var(tau(i,:))))
%     end
% for i =1:7
%     subplot(2,4,i)
%     plot(tau_ext_m(:,i),'LineWidth',2.0)
%     title("mean: joint"+num2str(i))
% end
% figure
% for i =1:7
%     subplot(2,4,i)
%     plot(tau_ext_v(:,i),'MarkerSize',10)
%     title("Variance: joint"+num2str(i))
% end

% writematrix(effort_m,"joint_effort.txt");
% writematrix(q_m,"joint_positions.txt");
% writematrix(qdot_m,"joint_velocity.txt")
%% Verify
iiwa = loadrobot("kukaIiwa14");
iiwa.DataFormat = "row";

for i = 1:size(q,2)
    J = geometricJacobian(iiwa,q(:,i)',"iiwa_link_ee_kuka");
    Tee = getTransform(iiwa,q(:,i)',"iiwa_link_ee_kuka");
    F = pinv(J)'*tau_ext(:,i);
    F_ee(:,i) =  Tee \ [F(4);F(5);F(6);0];

end
