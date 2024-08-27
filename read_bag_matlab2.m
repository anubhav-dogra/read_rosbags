bag_name = "filename.bag";
bag = rosbag(bag_name);
pose_B = select(bag, 'Topic', '/pose_topic');
wrench_B = select(bag, 'Topic', '/wrench_topic');
wrench_ts = timeseries(wrench_B,'Wrench.Force.Z');
ts_pose = timeseries(pose_B, 'Transform.Translation.Z');

subplot(2,1,1)
plot(wrench_ts,'r','LineWidth',2.0)
subplot(2,1,2)
plot(ts_pose,'LineWidth',2.0)