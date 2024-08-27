clear all
clc
close all
bag = rosbag("cartesian_wrench_test1.bag");
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
%% filter data
order = 2; % Filter order
cutoff_frequency = 1.5; % Cutoff frequency in Hz
sampling_frequency = 200; % Sampling frequency in Hz
[b, a] = butter(order, cutoff_frequency / (sampling_frequency / 2), 'low');
% Apply the Butterworth filter to your data
for i = 1:7
    effort(i,:) = filtfilt(b, a, effort(i,:));
    q(i,:) = filtfilt(b, a, q(i,:));
    tau_ext(i,:) = filtfilt(b, a, tau_ext(i,:));
end
    %%
    for i=500:1300
        q_des1(:,i) = q(:,i);
        effort_des1(:,i) = effort(:,i);
        tau_ext_des1(:,i) = tau_ext(:,i);
    end
    q_des1(:,1:500) = [];
    effort_des1(:,1:500)=[];
    tau_ext_des1(:,1:500)=[];

    for i=8000:9000
        q_des2(:,i) = q(:,i);
        effort_des2(:,i) = effort(:,i);
        tau_ext_des2(:,i) = tau_ext(:,i);
    end
    q_des2(:,1:8000) = [];
    effort_des2(:,1:8000)=[];
    tau_ext_des2(:,1:8000)=[];

    for i=48000:49000
        q_des3(:,i) = q(:,i);
        effort_des3(:,i) = effort(:,i);
        tau_ext_des3(:,i) = tau_ext(:,i);
    end
    q_des3(:,1:48000) = [];
    effort_des3(:,1:48000)=[];
    tau_ext_des3(:,1:48000)=[];

    for i=51000:52000
        q_des4(:,i) = q(:,i);
        effort_des4(:,i) = effort(:,i);
        tau_ext_des4(:,i) = tau_ext(:,i);
    end
    q_des4(:,1:51000) = [];
    effort_des4(:,1:51000)=[];
    tau_ext_des4(:,1:51000)=[];

    for i=52500:53500
        q_des5(:,i) = q(:,i);
        effort_des5(:,i) = effort(:,i);
        tau_ext_des5(:,i) = tau_ext(:,i);
    end
    q_des5(:,1:52500) = [];
    effort_des5(:,1:52500)=[];
    tau_ext_des5(:,1:52500)=[];

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
%%
% writematrix(q_des1,"joint_position_static1.txt");
% writematrix(effort_des1,"joint_effort_static1.txt");
% writematrix(tau_ext_des1,"joint_torque_ext_static1.txt")
%% Verify
iiwa = loadrobot("kukaIiwa14");
iiwa.DataFormat = "row";

for i = 1:size(q_des1,2)
    % J = geometricJacobian(iiwa,q(:,i)',"iiwa_link_ee_kuka");
    Tee = getTransform(iiwa,q_des1(:,i)',"iiwa_link_ee_kuka");
    Pz1(i) = Tee(3,4); 
    % F = pinv(J)'*tau_ext(:,i);
    % F_ee(:,i) =  Tee \ [F(4);F(5);F(6);0];

end
