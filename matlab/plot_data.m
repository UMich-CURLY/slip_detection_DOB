
%%
clear
vel_in = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair/MAir_0511_trial1_to_9_no_init_bias/01/01_wheel_vel_input.txt");
bias_est = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair/MAir_0511_trial1_to_9_no_init_bias/01/01_wheel_bias_est.txt");
vel_est = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair/MAir_0511_trial1_to_9_no_init_bias/01/01_wheel_vel_est.txt");
gt_pose = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair_gt/1/trial1_rectangle_center.txt");
est_pose = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair/MAir_0511_trial1_to_9_no_init_bias/01/01_inekf_wheel_vel.txt");
disturbance = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair/MAir_0511_trial1_to_9_no_init_bias/01/01_wheel_disturbance_est.txt");
t = est_pose(:,1);
imu = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair/MAir_0511_trial1_to_9_no_init_bias/01/01_wheel_imu.txt");



%%  plot estimated pose with gt pose from zed slam
figure(1)

plot(t,est_pose(:,2),'LineWidth',2);
hold on
plot(t,est_pose(:,3),'LineWidth',2);
hold on
plot(t,est_pose(:,4),'LineWidth',2);
hold on

plot(gt_pose(:,1),gt_pose(:,2),'LineWidth',2);
hold on
plot(gt_pose(:,1),gt_pose(:,3),'LineWidth',2);
hold on
plot(gt_pose(:,1),gt_pose(:,4),'LineWidth',2);

legend("est_x", "est_y","est_z", "gt_x", "gt_y", "gt_z",'FontSize',20);

%% traj

figure(1)
hold on
plot3(gt_pose(:,2),gt_pose(:,3),gt_pose(:,4));
% plot3(gt_pose_front(:,2),gt_pose_front(:,3),gt_pose_front(:,4));
plot3(est_pose(:,2),est_pose(:,3),est_pose(:,4));
% plot3(est_pose_body(:,2),est_pose_body(:,3),est_pose_body(:,4))
% plot3(est_pose_imu0(:, 2), est_pose_imu0(:, 3), est_pose_imu0(:, 4));
xlabel('$x \; (m)$', 'fontsize', 20, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', 20, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', 20, 'Interpreter','latex')
legend('MoCap Pose', 'InEKF Pose (plain version)', 'fontsize', 20, 'Interpreter','latex');
axis equal tight 
grid on

%% disturbance
figure(2);

plot(disturbance(:,1),disturbance(1:end,2),'LineWidth',1.5);
hold on
plot(disturbance(:,1),disturbance(1:end,3),'LineWidth',1.5);
hold on
plot(disturbance(:,1),disturbance(1:end,4),'LineWidth',1.5);

% plot(imu(:,1),imu(1:end,2),'LineWidth',2);
% hold on
% plot(imu(:,1),imu(1:end,3),'LineWidth',2);
% hold on
plot(imu(:,1),imu(1:end,4)/10,'LineWidth',1.5);

legend("disturbance_x", "disturbance_y","disturbance_z","w from IMU", 'FontSize',20);



%%  plot estimated velocity with input velocity
figure(3)
plot(t,vel_est(:,2),'LineWidth',2);
hold on
plot(t,vel_est(:,3),'LineWidth',2);
hold on
plot(t,vel_est(:,4),'LineWidth',2);
hold on

plot(vel_in(:,1),vel_in(:,2),'LineWidth',2);
hold on
plot(vel_in(:,1),vel_in(:,3),'LineWidth',2);
hold on
plot(vel_in(:,1),vel_in(:,4),'LineWidth',2);

legend("vel\_est_x", "vel\_est_y","vel\_est_z", "vel\_in_x", "vel\_in_y", "vel\_in_z",'FontSize',20);

%%  plot bias
figure(4)
plot(t,bias_est(:,2),'LineWidth',2);
hold on
plot(t,bias_est(:,3),'LineWidth',2);
hold on
plot(t,bias_est(:,4),'LineWidth',2);
hold on
plot(t,bias_est(:,5),'LineWidth',2);
hold on
plot(t,bias_est(:,6),'LineWidth',2);
hold on
plot(t,bias_est(:,7),'LineWidth',2);


legend("bias_{raw}", "bias_{pitch}", "bias_{yaw}", "bias_x", "bias_y", "bias_z",'FontSize',20);

%% plot bias with estimated pose
figure(4)

plot(t,est_pose(:,2),'LineWidth',2);
hold on
plot(t,est_pose(:,3),'LineWidth',2);
hold on
plot(t,est_pose(:,4),'LineWidth',2);
hold on

plot(t,bias_est(:,2),'LineWidth',2);
hold on
plot(t,bias_est(:,3),'LineWidth',2);
hold on
plot(t,bias_est(:,4),'LineWidth',2);
hold on
plot(t,bias_est(:,5),'LineWidth',2);
hold on
plot(t,bias_est(:,6),'LineWidth',2);
hold on
plot(t,bias_est(:,7),'LineWidth',2);


legend("est_x", "est_y","est_z", "bias_{raw}", "bias_{pitch}", "bias_{yaw}", "bias_x", "bias_y", "bias_z",'FontSize',20);



%% acc
figure(5);

plot(imu(:,1),imu(1:end,5),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,6),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,7),'LineWidth',2);

legend("acc_x", "acc_y","acc_z",'FontSize',20);

%% omega
figure(6);

plot(imu(:,1),imu(1:end,2),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,3),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,4),'LineWidth',2);

legend("\omega_x", "\omega_y","\omega_z",'FontSize',20);
%%
figure(7);
th = 0.03;
fs = 80;
imu_filtered = imu;
imu_filtered(:,2) = lowpass(imu(:,2),th,fs);
imu_filtered(:,3) = lowpass(imu(:,3),th,fs);
imu_filtered(:,4) = lowpass(imu(:,4),th,fs);
imu_filtered(:,5) = lowpass(imu(:,5),th,fs);
imu_filtered(:,6) = lowpass(imu(:,6),th,fs);
imu_filtered(:,7) = lowpass(imu(:,7),th,fs);

plot(imu_filtered(:,1),imu_filtered(1:end,5),'LineWidth',2);
hold on
plot(imu_filtered(:,1),imu_filtered(1:end,6),'LineWidth',2);
hold on
plot(imu_filtered(:,1),imu_filtered(1:end,7),'LineWidth',2);

legend("acc_x", "acc_y","acc_z",'FontSize',20);

%%
figure(8);

plot(imu(:,1),imu(1:end,2),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,3),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,4),'LineWidth',2);
hold on 
plot(imu1(:,1),imu1(1:end,2),'LineWidth',2);
hold on
plot(imu1(:,1),imu1(1:end,3),'LineWidth',2);
hold on
plot(imu1(:,1),imu1(1:end,4),'LineWidth',2);


legend("\omega_x", "\omega_y","\omega_z","\omega_{x\_origin}", "\omega_{y\_origin}","\omega_{z\_origin}",'FontSize',20);


%%  plot estimated velocity with input velocity
figure(9)
plot(t,vel_est(:,2),'LineWidth',2);
% hold on
% plot(t,vel_est(:,3),'LineWidth',2);
% hold on
% plot(t,vel_est(:,4),'LineWidth',2);
hold on

plot(vel_in(:,1),vel_in(:,2),'LineWidth',2);
% hold on
% plot(vel_in(:,1),vel_in(:,3),'LineWidth',2);
hold on
% plot(vel_in(:,1),vel_in(:,4),'LineWidth',2);

plot(t,bias_est(:,5).*0.000000001,'LineWidth',2);
% hold on
% plot(t,bias_est(:,6),'LineWidth',2);
hold on
% plot(t,bias_est(:,7),'LineWidth',2);

legend("vel\_est_x",  "vel\_in_x", "bias_x", 'FontSize',20);

%% omega
figure(10);

plot(imu(:,1),imu(1:end,2),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,3),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,4),'LineWidth',2);
hold on
plot(t,bias_est(:,2),'LineWidth',2);
hold on
plot(t,bias_est(:,3),'LineWidth',2);
hold on
plot(t,bias_est(:,4),'LineWidth',2);

legend("\omega_x", "\omega_y","\omega_z","est\_bias_{roll}", "est\_bias_{pitch}", "est\_bias_{yaw}",'FontSize',20);