
bag = rosbag('file_name.bag');
bSel = select(bag,'Topic','/cartesian_wrench_tool');
msgStructs = readMessages(bSel,'DataFormat','struct');

fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs);
n = size(fz,1);
time_init = double(msgStructs{1}.Header.Stamp.Sec);
time_final = double(msgStructs{n}.Header.Stamp.Sec);
n_sec = time_final-time_init;
time=linspace(0,n_sec,n);


fz(1:682) = [];
fz(3232:end) = [];
fz = fz - fz(1);
fz = -fz;

% [b, a] = butter(order, cutoff_frequency / (sampling_frequency / 2), 'low');
[b, a] = butter(2,0.5/(44/2),'low');

filtered_force = filtfilt(b,a,fz);

hold on
plot(filtered_force, 'LineWidth',2.0)
hold on 
% plot(fz,'LineWidth',1.0)
% xlim([0 3800])
set(gca,'LineWidth',0.75,'FontSize',16,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');
xlabel('Time (sec)')
ylabel('Force (N)')
grid on
% tightfig;
%text(4,-.4,'Newtons','FontSize',16,'Color','b')
