%% Initialize
clear all;
close all;
clc

% Parameters
fsize = 12;
vel_in = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_vel_input.txt");
bias_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_bias_est.txt");
vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_vel_est.txt");
gt_pose = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair_gt/1/trial1_rectangle_center.txt");
est_pose = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/inekf_wheel_vel.txt");
disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_disturbance_est.txt");
t = est_pose(:,1);
imu = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_imu.txt");

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
gt_slip(3700:4040)=1;
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

legend("$Slip$ (GT)","$Slip$ (Chi-Square Test)", 'FontSize',19, 'Interpreter','latex');
xlabel("$time(s)$", "FontSize", 25,'Interpreter','latex');
ylabel("$Slip$", "FontSize", 25,'Interpreter','latex');
ylim([-0.1,1.1]);
xlim([0,120]);
box on
grid on

%% plot vel and ori

% Read Bag into mat file
bagFile = rosbag('/media/xihang/761C73981C7351DB/husky_rosbag/rosbag_data/slip_detect_name/slip_detect7.bag');
% Read from Orb-SLAM3
orbslam3 = select(bagFile,'Topic','/ZED_ORBSLAM3/camera');
orbslam3_data = readMessages(orbslam3,'DataFormat','struct');

orbslam3_px = cellfun(@(m) double(m.Pose.Position.X),orbslam3_data);
orbslam3_py = cellfun(@(m) double(m.Pose.Position.Y),orbslam3_data);
orbslam3_pz = cellfun(@(m) double(m.Pose.Position.Z),orbslam3_data);

orbslam3_ox = cellfun(@(m) double(m.Pose.Orientation.X),orbslam3_data);
orbslam3_oy = cellfun(@(m) double(m.Pose.Orientation.Y),orbslam3_data);
orbslam3_oz = cellfun(@(m) double(m.Pose.Orientation.Z),orbslam3_data);
orbslam3_ow = cellfun(@(m) double(m.Pose.Orientation.W),orbslam3_data);

NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),orbslam3_data,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),orbslam3_data,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
orbslam3_t = Sec_tot - Sec_tot(1,1);

orbslam3_Mat = [orbslam3_t orbslam3_px orbslam3_py orbslam3_pz orbslam3_ow orbslam3_ox orbslam3_oy orbslam3_oz];


% Calculating estimated vel from pose
vel_mat_orbslam3 = [];

for i=2:length(orbslam3_ow)
    prev_T = [quat2rotm(orbslam3_Mat(i-1,5:8)) [orbslam3_Mat(i-1,2);orbslam3_Mat(i-1,3);orbslam3_Mat(i-1,4)]; 0 0 0 1];
    current_T = [quat2rotm(orbslam3_Mat(i,5:8)) [orbslam3_Mat(i,2);orbslam3_Mat(i,3);orbslam3_Mat(i,4)]; 0 0 0 1];
    del_t = orbslam3_Mat(i,1) - orbslam3_Mat(i-1,1);
    vel_wedge = logm(inv(prev_T)*current_T)/del_t; % Left Invariant
    vel_vec = undwedge_se3(vel_wedge)'; % 1x6
    vel_mat_orbslam3 = [vel_mat_orbslam3;orbslam3_Mat(i,1) vel_vec];
end

figure(13)

% plot pose from Orb-SLAM3

est_pose = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/inekf_wheel_vel.txt");
est_pose_baseline = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_no_disturbance/inekf_wheel_vel.txt");


orbslam3_Euler = quat2eul(orbslam3_Mat(:,5:8));
orbslam3_Euler = [-orbslam3_Euler(:,1) -orbslam3_Euler(:,2) orbslam3_Euler(:,3)];

est_pose_baseline_x = est_pose_baseline(:,5);
est_pose_baseline_y = est_pose_baseline(:,6);
est_pose_baseline_z = est_pose_baseline(:,7);
est_pose_baseline_w = est_pose_baseline(:,8);
est_pose_baseline_Quat = [est_pose_baseline_w est_pose_baseline_x est_pose_baseline_y est_pose_baseline_z];
est_pose_baseline_Euler =  quat2eul(est_pose_baseline_Quat);

est_pose_x = est_pose(:,5);
est_pose_y = est_pose(:,6);
est_pose_z = est_pose(:,7);
est_pose_w = est_pose(:,8);
est_pose_Quat = [est_pose_w est_pose_x est_pose_y est_pose_z];
est_pose_Euler =  quat2eul(est_pose_Quat);

subplot(3,2,1)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1,orbslam3_Euler(:,1),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline_Euler(:,1),'b-.','LineWidth',1.3);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose_Euler(:,1),'r-.','LineWidth',1.3);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$yaw \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')
xlim([44,57]);
box on
grid on


subplot(3,2,3)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1,orbslam3_Euler(:,2),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline_Euler(:,2),'b-.','LineWidth',1.3);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose_Euler(:,2),'r-.','LineWidth',1.3);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$pitch \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')
xlim([44,57]);
box on
grid on

subplot(3,2,5)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1,orbslam3_Euler(:,3),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline_Euler(:,3),'b-.','LineWidth',1.3);
hold on
plot(est_pose(:,1)-est_pose(1,1), est_pose_Euler(:,3),'r-.','LineWidth',1.3);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$roll \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')
xlim([44,57]);
box on
grid on


R_Orb2Body = [    0.9984    0.0219    0.0521;
   -0.0220    0.9998    0.0012;
   -0.0521   -0.0024    0.9986];

Dis_Body2Orb = [-0.0041   -0.1011   -0.0177];

vel_x = movmean(vel_mat_orbslam3(1:end,2), 3);
vel_y = movmean(vel_mat_orbslam3(1:end,3), 3);
vel_z = movmean(vel_mat_orbslam3(1:end,4), 3);


vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_vel_est.txt");
vel_est_no_disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_no_disturbance/wheel_vel_est.txt");

subplot(3,2,2)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,vel_x,'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,5),'b-.','LineWidth',1.3);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,5),'r-.','LineWidth',1.3);

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
box on 
grid on
xlim([44,57]);
subplot(3,2,4)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,-vel_y,'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,6),'b-.','LineWidth',0.9);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,6),'r-.','LineWidth',0.9);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
box on 
grid on
xlim([44,57]);
subplot(3,2,6)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,-vel_z,'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,7),'b-.','LineWidth',0.9);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,7),'r-.','LineWidth',0.9);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('$State-OrbSLAM3$','$Estimated-State-Baseline$','$Estimated-State-DOB$', 'fontsize', fsize, 'Interpreter','latex');
box on 
grid on
xlim([44,57]);

%% Functions
function twist_vec = undwedge_se3(twist_mat)
    % v_x v_y v_z w_x w_y w_z
%     twist_vec = [twist_mat(:,end); twist_mat(2,3); twist_mat(3,1); twist_mat(1,2)]; % notes
    twist_vec = [constrain_lin_vel(twist_mat(1,end)); constrain_lin_vel(twist_mat(2,end)); constrain_lin_vel(twist_mat(3,end)); constrain_ang_vel(twist_mat(3,2)); constrain_ang_vel(twist_mat(1,3)); constrain_ang_vel(twist_mat(2,1))]; % web and me
end

function vel_lin_max = constrain_lin_vel(vel)
    max_lin_vel = 1; % max 1 m/s
    if vel > max_lin_vel
        vel_lin_max = max_lin_vel;
    elseif vel < -max_lin_vel
        vel_lin_max = - max_lin_vel;
    else
        vel_lin_max = vel;
    end
end

function vel_ang_max = constrain_ang_vel(vel)
    max_ang_vel = 2.5; % max 2.5 rad/s
    if vel > max_ang_vel
        vel_ang_max = max_ang_vel;
    elseif vel < -max_ang_vel
        vel_ang_max = - max_ang_vel;
    else
        vel_ang_max = vel; 
    end
end

function xs = skew(x)
xs = [0, -x(3), x(2);...
      x(3), 0, -x(1);...
      -x(2), x(1), 0];
end

function R = Rx(t)

R = [1, 0, 0;...
     0, cos(t), -sin(t);...
     0, sin(t), cos(t)];
end

function R = Rz(t)

R = [cos(t), -sin(t), 0;...
     sin(t), cos(t), 0;...
     0, 0, 1];
end

function R = Ry(t)

R = [cos(t), 0, sin(t);...
     0, 1, 0;...
     -sin(t), 0, cos(t)];
end
