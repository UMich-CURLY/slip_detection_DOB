
%% Initialize
clear all;
close all;
clc

clear
vel_in = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_vel_input.txt");
bias_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_bias_est.txt");
vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_vel_est.txt");
gt_pose = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair_gt/1/trial1_rectangle_center.txt");
est_pose = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/inekf_wheel_vel.txt");
disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_disturbance_est.txt");
t = est_pose(:,1);
imu = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_imu.txt");



%%  plot estimated pose with gt pose from zed slamup_to_hill3

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

delta = 15;

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

% downsampled_disturbance = downsample(disturbance,2);

blue = [0 0.4470 0.7410];
red = [1 0 0];


trunked_disturbance = [];
for i=1:length(disturbance)
    if disturbance(i,6)==1
        trunked_disturbance = [trunked_disturbance; disturbance(i,:)];
    end
end
        

% scatter(gt_time, gt_slip);
area(gt_time, gt_slip,'FaceColor',blue,'EdgeColor', 'none');

hold on
scatter(trunked_disturbance(1:end,1)-disturbance(1,1),trunked_disturbance(1:end,6),20,'filled');
% scatter(downsampled_disturbance(1:end,1)-downsampled_disturbance(1,1),downsampled_disturbance(1:end,7),5);
% area(downsampled_disturbance(1:end,1)-downsampled_disturbance(1,1),downsampled_disturbance(1:end,6));

% for i=2:length(downsampled_disturbance)
%    x = [downsampled_disturbance(i-1,1)-downsampled_disturbance(1,1) downsampled_disturbance(i,1)-downsampled_disturbance(1,1)];
%    if downsampled_disturbance(i-1,6)==1
%        y = [1 1];
%    elseif downsampled_disturbance(i-1,6)==0
%        y = [0 0];
%    end
%     area(x,y,'FaceColor',red,'EdgeColor', 'none')
%     hold on
% end

% x = [];
% y = [];
% 
% for i=2:length(downsampled_disturbance)
%    x = [downsampled_disturbance(i-1,1)-downsampled_disturbance(1,1) downsampled_disturbance(i,1)-downsampled_disturbance(1,1)];
%    if downsampled_disturbance(i-1,6)==1
%        y = [1 1];
%    elseif downsampled_disturbance(i-1,6)==0
%        y = [0 0];
%    end
% end
% area(x,y,'FaceColor',red,'EdgeColor', 'none')
% hold on

legend("$Slip$ (GT)","$Slip$ (Chi-Square Test)", 'FontSize',19, 'Interpreter','latex');
xlabel("$time(s)$", "FontSize", 25,'Interpreter','latex');
ylabel("$Slip$", "FontSize", 25,'Interpreter','latex');
ylim([-0.1,1.1]);
xlim([0,120]);
box on
grid on

%% Slip detection
figure(93);


blue = [0 0.4470 0.7410];
red = [1 0 0];

gt_time = linspace(5,25,2000);
gt_slip = zeros(1,2000);
gt_slip(230:680)=1;
gt_slip(1190:1710)=1;

trunked_disturbance = [];
for i=1:length(disturbance)
    if disturbance(i,6)==1
        trunked_disturbance = [trunked_disturbance; disturbance(i,:)];
    end
end

% scatter(gt_time, gt_slip);
area(gt_time, gt_slip,'FaceColor',blue,'EdgeColor', 'none');

hold on
scatter(trunked_disturbance(1:end,1)-disturbance(1,1),trunked_disturbance(1:end,6),20,'filled');

legend("$Slip$ (GT)","$Slip$ (Chi-Square Test)", 'FontSize',19, 'Interpreter','latex');
xlabel("$time(s)$", "FontSize", 25,'Interpreter','latex');
ylabel("$Slip$", "FontSize", 25,'Interpreter','latex');
ylim([-0.1,1.1]);
xlim([5,25]);
box on
grid on

%% Slip detection

time = disturbance(:,1) - disturbance(1,1);

gt_slip_sync = [];
for i=1067:5336
    t = disturbance(i,1)-disturbance(1,1);
    if (t>7.3 && t<11.8) || (t>16.9 && t<22.1)
        gt_slip_sync = [gt_slip_sync;1];
    else
        gt_slip_sync = [gt_slip_sync;0];
    end
end

% count false positive (detected slip that are actually not)
count_false_pos = 0;
count_true_neg = 0;
count_false_neg = 0;
count_true_pos = 0;
count_acc = 0;
for i=1067:5336
    if disturbance(i,6) == 1 && gt_slip_sync(i-1066) == 0
        count_false_pos = count_false_pos + 1;
    elseif disturbance(i,6) == 0 && gt_slip_sync(i-1066) == 0
        count_true_neg = count_true_neg + 1;
        count_acc = count_acc+1;
    elseif disturbance(i,6) == 0 && gt_slip_sync(i-1066) == 1
        count_false_neg = count_false_neg + 1;
    elseif disturbance(i,6) == 1 && gt_slip_sync(i-1066) == 1
        count_true_pos = count_true_pos + 1;
        count_acc = count_acc+1;
    end
end

false_pos_rate = count_false_pos/(count_false_pos+count_true_neg);
false_neg_Rate = count_false_neg/(count_false_neg+count_true_pos);
acc = count_acc/(5336-1067+1);


%% Slip detection

gt_slip_sync = [];
for i=1:length(disturbance)
    t = disturbance(i,1)-disturbance(1,1);
    if (t>6.67 && t<12.93) || (t>25.93 && t<32.47) ||(t>45.94 && t<55.14) ||(t>69.47 && t<79.14) ||(t>95.01 && t<105.8) ||(t>16.87 && t<21.20) ||(t>60.40 && t<64.54) ||(t>85.07 && t<88.47)
        gt_slip_sync = [gt_slip_sync;1];
    else
        gt_slip_sync = [gt_slip_sync;0];
    end
end

% count false positive (detected slip that are actually not)
count_false_pos = 0;
count_true_neg = 0;
count_false_neg = 0;
count_true_pos = 0;
count_acc = 0;
for i=1:length(disturbance)
    if disturbance(i,6) == 1 && gt_slip_sync(i) == 0
        count_false_pos = count_false_pos + 1;
    elseif disturbance(i,6) == 0 && gt_slip_sync(i) == 0
        count_true_neg = count_true_neg + 1;
        count_acc = count_acc+1;
    elseif disturbance(i,6) == 0 && gt_slip_sync(i) == 1
        count_false_neg = count_false_neg + 1;
    elseif disturbance(i,6) == 1 && gt_slip_sync(i) == 1
        count_true_pos = count_true_pos + 1;
        count_acc = count_acc+1;
    end
end

false_pos_rate = count_false_pos/(count_false_pos+count_true_neg);
false_neg_Rate = count_false_neg/(count_false_neg+count_true_pos);
acc = count_acc/length(disturbance);


%%
figure(9);

subplot(1,2,1)

downsampled_disturbance_2 = downsample(disturbance,200);
% scatter(gt_time, gt_slip);
% area(gt_time, gt_slip,'FaceColor',lightBlue,'EdgeColor', 'none');
boxchart(downsampled_disturbance_2(1:end,8),'orientation','horizontal');
hold on

xlabel("$Value$", "FontSize", 12,'Interpreter','latex');
ylabel("$\chi^2$", "FontSize", 12,'Interpreter','latex');
box on
grid on

subplot(1,2,2)

% scatter(gt_time, gt_slip);
% area(gt_time, gt_slip,'FaceColor',lightBlue,'EdgeColor', 'none');
boxchart(downsampled_disturbance_2(1:end,7),'orientation','horizontal');
hold on

% scatter(downsampled_disturbance(1:end,1)-downsampled_disturbance(1,1),downsampled_disturbance(1:end,7),8,[0.4940 0.1840 0.5560],'filled');
% hold on

xlabel("$Value$", "FontSize", 12,'Interpreter','latex');
ylabel("$\chi^2$ $(Slip$ $Model)$", "FontSize", 12,'Interpreter','latex');
box on
grid on

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