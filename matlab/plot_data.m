
%% Initialize
clear all;
close all;
clc

clear
vel_in = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_slip_detection/wheel_vel_input.txt");
bias_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_slip_detection/wheel_bias_est.txt");
vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_slip_detection/wheel_vel_est.txt");
gt_pose = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair_gt/1/trial1_rectangle_center.txt");
est_pose = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_slip_detection/inekf_wheel_vel.txt");
disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_slip_detection/wheel_disturbance_est.txt");
t = est_pose(:,1);
imu = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_slip_detection/wheel_imu.txt");



%%  plot estimated pose with gt pose from zed slam
figure(1)

plot(t,est_pose(:,2),'LineWidth',2);
hold on
plot(t,est_pose(:,3),'LineWidth',2);
hold on
plot(t,est_pose(:,4),'LineWidth',2);
hold on
d = designfilt('lowpassfir', 'FilterOrder', 100, 'CutoffFrequency', 11, 'SampleRate', 200);
disturbance_x = filter(d, disturbance(1:end,2));
plot(gt_pose(:,1),gt_pose(:,2),'LineWidth',2);
hold on
plot(gt_pose(:,1),gt_pose(:,3),'LineWidth',2);
hold on
plot(gt_pose(:,1),gt_pose(:,4),'LineWidth',2);

legend("est_x", "est_y","est_z", "gt_x", "gt_y", "gt_z",'FontSize',20);

%% plot pose
figure(3)

plot(t-t(1),est_pose(:,2),'LineWidth',2);
hold on
plot(t-t(1),est_pose(:,3),'LineWidth',2);
hold on
plot(t-t(1),est_pose(:,4),'LineWidth',2);
hold on

box on
grid on
% xlim([2, 65])

%% traj

figure(5)
hold on
% plot3(gt_pose(:,2),gt_pose(:,3),gt_pose(:,4));
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
figure(7);

% d = designfilt('lowpassfir', 'FilterOrder', 100, 'CutoffFrequency', 11, 'SampleRate', 200);
% disturbance_x = filter(d, disturbance(1:end,2));
% disturbance_y = filter(d, disturbance(1:end,3));
% disturbance_z = filter(d, disturbance(1:end,4));

plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance(1:end,2)),'LineWidth',1.5);
hold on
plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance(1:end,3)),'LineWidth',1.5);
hold on
plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance(1:end,4)),'LineWidth',1.5);

legend("disturbance_x", "disturbance_y","disturbance_z","w from IMU", 'FontSize',20);
% xlim([0, 65])
xlabel("time (x)", "FontSize", 12)
ylabel("m/s", "FontSize", 12)
box on
grid on

%% slip flag - slip detection
figure(8);

gt_time = linspace(0,120,12000);
gt_slip = zeros(1,12000);
gt_slip(667:1293)=1;
gt_slip(2593:3247)=1;
gt_slip(4594:5514)=1;
gt_slip(6947:7914)=1;
gt_slip(9501:10580)=1;
gt_slip(1687:2120)=1;
% gt_slip(3700:4040)=1;
gt_slip(6040:6454)=1;
gt_slip(8507:8847)=1;

downsampled_disturbance = downsample(disturbance,25);

lightBlue = [0 0.4470 0.7410];

subplot(1,2,1)
% scatter(gt_time, gt_slip);
area(gt_time, gt_slip,'FaceColor',lightBlue,'EdgeColor', 'none');

hold on
scatter(downsampled_disturbance(1:end,1)-downsampled_disturbance(1,1),downsampled_disturbance(1:end,6),8,'filled');
% scatter(downsampled_disturbance(1:end,1)-downsampled_disturbance(1,1),downsampled_disturbance(1:end,7),5);
% area(downsampled_disturbance(1:end,1)-downsampled_disturbance(1,1),downsampled_disturbance(1:end,6));

hold on

legend("$Slip$ (GT)","$Slip$ (Chi-Square Test)", 'FontSize',20, 'Interpreter','latex');
xlabel("$time(s)$", "FontSize", 12,'Interpreter','latex');
ylabel("$Slip$", "FontSize", 12,'Interpreter','latex');
ylim([-0.2,1.2]);
xlim([0,120]);
box on
grid on

writematrix(downsampled_disturbance,"disturbance.txt");


subplot(1,2,2)
lightBlue = [0 0.4470 0.7410];

downsampled_disturbance_2 = downsample(disturbance,100);

% scatter(gt_time, gt_slip);
% area(gt_time, gt_slip,'FaceColor',lightBlue,'EdgeColor', 'none');
boxchart(downsampled_disturbance_2(1:end,7));
hold on

% scatter(downsampled_disturbance(1:end,1)-downsampled_disturbance(1,1),downsampled_disturbance(1:end,7),8,[0.4940 0.1840 0.5560],'filled');
% hold on



% legend("$Slip$ (Chi-Square Test using Slip Model)", 'FontSize',20, 'Interpreter','latex');
xlabel("$Chi-Square$ $Test$ $Using$ $Slip$ $model$", "FontSize", 12,'Interpreter','latex');
ylabel("$Chi$", "FontSize", 12,'Interpreter','latex');
box on
grid on

writematrix(downsampled_disturbance,"disturbance.txt");
%% chi
figure(8);

t = disturbance(1:end,1)-disturbance(1,1);

scatter(disturbance(1:end,1)-disturbance(1,1),disturbance(1:end,8),1,"+");
hold on

legend("slip flag", 'FontSize',20);
xlabel("time (x)", "FontSize", 12)
ylabel("slip classifier", "FontSize", 12)
box on
grid on

%% friction coeff
figure(8);

t = disturbance(1:end,1)-disturbance(1,1);

scatter(disturbance(1:end,1)-disturbance(1,1),disturbance(1:end,5),1,"+");
hold on

legend("slip flag", 'FontSize',20);
xlabel("time (x)", "FontSize", 12)
ylabel("slip classifier", "FontSize", 12)
box on
grid on

%% angular velocity in z axis

figure(9);
% plot(disturbance(1:end,1),disturbance(1:end,2),'LineWidth',1.5);
% hold on
% plot(disturbance(1:end,1),disturbance(1:end,3),'LineWidth',1.5);
% hold on
% plot(disturbance(1:end,1),disturbance(1:end,4),'LineWidth',1.5);


% plot(imu(:,1),imu(1:end,2),'LineWidth',2);
% hold on
% plot(imu(:,1),imu(1:end,3),'LineWidth',2);
% hold on
plot(imu(:,1),abs(imu(1:end,4))/20,'LineWidth',1.5);

legend("w from IMU", 'FontSize',20);
%% plot disturbance and angular velocity

figure(11)

subplot(2,1,1)
d = designfilt('lowpassfir', 'FilterOrder', 100, 'CutoffFrequency', 11, 'SampleRate', 200);
disturbance_x = filter(d, disturbance(1:end,2));
disturbance_y = filter(d, disturbance(1:end,3));
disturbance_z = filter(d, disturbance(1:end,4));

plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance_x),'LineWidth',1.5);
hold on
plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance_y),'LineWidth',1.5);
hold on
plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance_z),'LineWidth',1.5);

legend("$disturbance_x$", "$disturbance_y$","$disturbance_z$", 'FontSize',12, "interpreter", "latex");
% xlim([2, 65])
xlabel("time (s)", "FontSize", 12, "interpreter", "latex" )
ylabel("m/s", "FontSize", 12, "interpreter", "latex")
box on
grid on

subplot(2,1,2)
plot(imu(:,1)-imu(1,1),abs(imu(1:end,4))/20,'LineWidth',1.5);


% xlim([2, 65])
xlabel("time (s)", "FontSize", 12, "interpreter", "latex")
ylabel("rad", "FontSize", 12, "interpreter", "latex")

box on 
grid on
legend("$w(z)$", 'FontSize',12, "interpreter", "latex");


%%  plot estimated velocity with input velocity
figure(13)
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
%% plot disturbance and body velocity

figure(15)

subplot(2,1,1)
d = designfilt('lowpassfir', 'FilterOrder', 100, 'CutoffFrequency', 11, 'SampleRate', 200);
disturbance_x = filter(d, disturbance(1:end,2));
disturbance_y = filter(d, disturbance(1:end,3));
disturbance_z = filter(d, disturbance(1:end,4));

plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance_x),'LineWidth',1.5);
hold on
plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance_y),'LineWidth',1.5);
hold on
plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance_z),'LineWidth',1.5);

legend("$disturbance_x$", "$disturbance_y$","$disturbance_z$", 'FontSize',12, "interpreter", "latex");
% xlim([12, 33])
xlabel("time (s)", "FontSize", 12, "interpreter", "latex" )
ylabel("m/s", "FontSize", 12, "interpreter", "latex")
box on
grid on

subplot(2,1,2)
plot(t-t(1),vel_est(:,5),'LineWidth',2);
hold on
plot(t-t(1),vel_est(:,6),'LineWidth',2);
hold on
plot(t-t(1),vel_est(:,7),'LineWidth',2);
hold on

% xlim([12, 33])
xlabel("time (s)", "FontSize", 12, "interpreter", "latex")
ylabel("m/s", "FontSize", 12, "interpreter", "latex")

box on 
grid on
legend("$vel\_est_x$", "$vel\_est_y$","$vel\_est_z$",'FontSize',12, "interpreter", "latex");

%% plot estimated body velocity
figure(4)
plot(t-t(1),vel_est(:,5),'LineWidth',2);
hold on
plot(t-t(1),vel_est(:,6),'LineWidth',2);
hold on
plot(t-t(1),vel_est(:,7),'LineWidth',2);
hold on

legend("vel\_est_x", "vel\_est_y","vel\_est_z",'FontSize',20);

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