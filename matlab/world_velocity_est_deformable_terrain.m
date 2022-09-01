%% Initialize
clear all;
close all;
clc
%% Parameters
fsize = 12;
husky_width = (0.42+0.67)/2;
husky_length = 0.544;
husky_diag = sqrt(husky_width^2+husky_length^2);
r_wheel = [0; 0; -0.33/2]; % right hand rule at the contact point
robot_base = [husky_diag/2*cos(pi/4); husky_diag/2*cos(pi/4); 0]; % Need to check angle and direction for each wheel
slip_threshold = 0.01;
%% Read Bag into mat file
bagFile = rosbag('/media/xihang/761C73981C7351DB/husky_rosbag/rosbag_data/slip_detect_mars/up_to_hill3.bag');
%% Read from Orb-SLAM3
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


%% plot pose from Orb-SLAM3

est_pose = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3/inekf_wheel_vel.txt");
est_pose_baseline = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_no_disturbance/inekf_wheel_vel.txt");


%% Plotting traj from Orb-SLAM3
figure(10)
hold on
plot3(orbslam3_Mat(:,2),-orbslam3_Mat(:,3),-orbslam3_Mat(:,4));
% plot3(InEKF_Mat(:,2),InEKF_Mat(:,3),InEKF_Mat(:,4));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('Orb-SLAM3 Pose', 'fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on


plot3(est_pose_baseline(:,2),est_pose_baseline(:,3),est_pose_baseline(:,4));
% plot3(InEKF_Mat(:,2),InEKF_Mat(:,3),InEKF_Mat(:,4));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('Orb-SLAM3 Pose', 'fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on

plot3(est_pose(:,2),est_pose(:,3),est_pose(:,4));
% plot3(InEKF_Mat(:,2),InEKF_Mat(:,3),InEKF_Mat(:,4));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('Orb-SLAM3 Pose', 'fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on


figure(11)

subplot(3,1,1)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1),orbslam3_Mat(:,2),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline(:,2),'b-.','LineWidth',1.5);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose(:,2),'r-.','LineWidth',1.5);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

box on
grid on

subplot(3,1,2)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1),-orbslam3_Mat(:,3),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline(:,3),'b-.','LineWidth',1.5);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose(:,3),'r-.','LineWidth',1.5);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')

box on
grid on


subplot(3,1,3)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1),-orbslam3_Mat(:,4),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline(:,4),'b-.','LineWidth',1.5);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose(:,4),'r-.','LineWidth',1.5);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('$Pose-OrbSLAM3$','$Estimated-Pose-Baseline$','$Estimated-Pose-DOB$', 'fontsize', fsize, 'Interpreter','latex');
box on
grid on
%% plot ori from Orb-SLAM3, baseline and DOB

figure(12)

orbslam3_Euler = quat2eul(orbslam3_Mat(:,5:8));

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

subplot(3,1,1)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1,-orbslam3_Euler(:,1),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline_Euler(:,1),'b-.','LineWidth',1.3);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose_Euler(:,1),'r-.','LineWidth',1.3);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$yaw \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')
legend('$Pose-OrbSLAM3$','$Estimated-Pose-Baseline$','$Estimated-Pose-DOB$', 'fontsize', fsize, 'Interpreter','latex');
box on
grid on


subplot(3,1,2)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1,-orbslam3_Euler(:,2),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline_Euler(:,2),'b-.','LineWidth',1.3);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose_Euler(:,2),'r-.','LineWidth',1.3);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$pitch \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')
legend('$Pose-OrbSLAM3$','$Estimated-Pose-Baseline$','$Estimated-Pose-DOB$', 'fontsize', fsize, 'Interpreter','latex');
box on
grid on

subplot(3,1,3)
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1,orbslam3_Euler(:,3),'k','LineWidth',1.5);
hold on
plot(est_pose_baseline(:,1)-est_pose_baseline(1,1),est_pose_baseline_Euler(:,3),'b-.','LineWidth',1.3);
hold on
plot(est_pose(:,1)-est_pose(1,1),est_pose_Euler(:,3),'r-.','LineWidth',1.3);
hold on

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$roll \; (rad)$', 'fontsize', fsize, 'Interpreter','latex')
legend('$Pose-OrbSLAM3$','$Estimated-Pose-Baseline$','$Estimated-Pose-DOB$', 'fontsize', fsize, 'Interpreter','latex');
box on
grid on



%% Calculating estimated vel from pose
vel_mat_orbslam3 = [];

for i=2:length(orbslam3_ow)
    prev_T = [quat2rotm(orbslam3_Mat(i-1,5:8)) [orbslam3_Mat(i-1,2);orbslam3_Mat(i-1,3);orbslam3_Mat(i-1,4)]; 0 0 0 1];
    current_T = [quat2rotm(orbslam3_Mat(i,5:8)) [orbslam3_Mat(i,2);orbslam3_Mat(i,3);orbslam3_Mat(i,4)]; 0 0 0 1];
    del_t = orbslam3_Mat(i,1) - orbslam3_Mat(i-1,1);
    vel_wedge = logm(inv(prev_T)*current_T)/del_t; % Left Invariant
    vel_vec = undwedge_se3(vel_wedge)'; % 1x6
    vel_mat_orbslam3 = [vel_mat_orbslam3;orbslam3_Mat(i,1) vel_vec];
end

%% plot vel in world frame


figure(100)
% d = designfilt('lowpassfir', 'FilterOrder', 1, 'CutoffFrequency', 11, 'SampleRate', 200);
% vel_x = filter(d, vel_mat_orbslam3(1:end,2));
% vel_y = filter(d, vel_mat_orbslam3(1:end,3));
% vel_z = filter(d, vel_mat_orbslam3(1:end,4));

% vel_x = vel_mat_orbslam3(1:end,2);
% vel_y = vel_mat_orbslam3(1:end,3);
% vel_z = vel_mat_orbslam3(1:end,4);


vel_x = movmean(vel_mat_orbslam3(1:end,2), 3);
vel_y = movmean(vel_mat_orbslam3(1:end,3), 3);
vel_z = movmean(vel_mat_orbslam3(1:end,4), 3);

vel_body = [vel_x vel_y vel_z];
vel_world = [];
for i=1:length(vel_mat_orbslam3)
    vel_world = [vel_world ; transpose(quat2rotm(orbslam3_Mat(i,5:8)) * transpose(vel_body(i,1:3)))];

end

vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3/wheel_vel_est.txt");
vel_est_no_disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_no_disturbance/wheel_vel_est.txt");

subplot(3,1,1)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,vel_world(:,1),'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,2),'b-.','LineWidth',1.3);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,2),'r-.','LineWidth',1.3);
xlim([5,25]);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
box on 
grid on

subplot(3,1,2)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,-vel_world(:,2),'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,3),'b-.','LineWidth',0.9);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,3),'r-.','LineWidth',0.9);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
xlim([5,25]);
box on 
grid on

subplot(3,1,3)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,-vel_world(:,3),'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,4),'b-.','LineWidth',0.9);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,4),'r-.','LineWidth',0.9);
xlim([5,25]);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('$Velocity-OrbSLAM3$','$Estimated-Velocity-Baseline$','$Estimated-Velocity-DOB$', 'fontsize', fsize, 'Interpreter','latex');
box on 
grid on


%% RMSE

% form array - Orb-SLAM3
orb_vx = [vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1 vel_world(:,1)];
orb_vy = [vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1 -vel_world(:,2)];
orb_vz = [vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1 -vel_world(:,3)];


% form array - baseline
baseline_vx = [vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1) vel_est_no_disturbance(:,2)];
baseline_vy = [vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1) vel_est_no_disturbance(:,3)];
baseline_vz = [vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1) vel_est_no_disturbance(:,4)];


% form array - DOB
DOB_vx = [vel_est(1:end,1)-vel_est(1,1) vel_est(:,2)];
DOB_vy = [vel_est(1:end,1)-vel_est(1,1) vel_est(:,3)];
DOB_vz = [vel_est(1:end,1)-vel_est(1,1) vel_est(:,4)];


% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vx_extracted = [];
DOB_vx_extracted = [];
for i=1:length(orb_vx)
    
    [closest_time,index] = min(abs(baseline_vx(:,1)-orb_vx(i,1)));
    baseline_vx_extracted = [baseline_vx_extracted;baseline_vx(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vx(:,1)-orb_vx(i,1)));
    DOB_vx_extracted = [DOB_vx_extracted;DOB_vx(index, 2)];
   
end

rmse_baseline_vx = sqrt(immse(baseline_vx_extracted, orb_vx(:,2)));
rmse_DOB_vx = sqrt(immse(DOB_vx_extracted, orb_vx(:,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vy_extracted = [];
DOB_vy_extracted = [];
for i=1:length(orb_vy)
    
    [closest_time,index] = min(abs(baseline_vy(:,1)-orb_vy(i,1)));
    baseline_vy_extracted = [baseline_vy_extracted;baseline_vy(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vy(:,1)-orb_vy(i,1)));
    DOB_vy_extracted = [DOB_vy_extracted;DOB_vy(index, 2)];
   
end

rmse_baseline_vy = sqrt(immse(baseline_vy_extracted, orb_vy(:,2)));
rmse_DOB_vy = sqrt(immse(DOB_vy_extracted, orb_vy(:,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vz_extracted = [];
DOB_vz_extracted = [];
for i=1:length(orb_vz)
    
    [closest_time,index] = min(abs(baseline_vz(:,1)-orb_vz(i,1)));
    baseline_vz_extracted = [baseline_vz_extracted;baseline_vz(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vz(:,1)-orb_vz(i,1)));
    DOB_vz_extracted = [DOB_vz_extracted;DOB_vz(index, 2)];
   
end

rmse_baseline_vz = sqrt(immse(baseline_vz_extracted, orb_vz(:,2)));
rmse_DOB_vz = sqrt(immse(DOB_vz_extracted, orb_vz(:,2)));


%% plot vel


figure(1)
% d = designfilt('lowpassfir', 'FilterOrder', 1, 'CutoffFrequency', 11, 'SampleRate', 200);
% vel_x = filter(d, vel_mat_orbslam3(1:end,2));
% vel_y = filter(d, vel_mat_orbslam3(1:end,3));
% vel_z = filter(d, vel_mat_orbslam3(1:end,4));

% vel_x = vel_mat_orbslam3(1:end,2);
% vel_y = vel_mat_orbslam3(1:end,3);
% vel_z = vel_mat_orbslam3(1:end,4);


vel_x = movmean(vel_mat_orbslam3(1:end,2), 3);
vel_y = movmean(vel_mat_orbslam3(1:end,3), 3);
vel_z = movmean(vel_mat_orbslam3(1:end,4), 3);

vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3/wheel_vel_est.txt");
vel_est_no_disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_no_disturbance/wheel_vel_est.txt");

subplot(3,1,1)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,vel_x,'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,5),'b-.','LineWidth',1.3);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,5),'r-.','LineWidth',1.3);

xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
box on 
grid on

subplot(3,1,2)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,-vel_y,'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,6),'b-.','LineWidth',0.9);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,6),'r-.','LineWidth',0.9);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
box on 
grid on

subplot(3,1,3)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,-vel_z,'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,7),'b-.','LineWidth',0.9);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,7),'r-.','LineWidth',0.9);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('$Velocity-OrbSLAM3$','$Estimated-Velocity-Baseline$','$Estimated-Velocity-DOB$', 'fontsize', fsize, 'Interpreter','latex');
box on 
grid on


%% RMSE

% form array - Orb-SLAM3
orb_vx = [vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1 vel_x];
orb_vy = [vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1 vel_y];
orb_vz = [vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1 vel_z];

orb_yaw = [orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1 -orbslam3_Euler(:,1)];
orb_pitch = [orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1 -orbslam3_Euler(:,2)];
orb_roll = [orbslam3_Mat(:,1)-orbslam3_Mat(1,1)-0.1 orbslam3_Euler(:,3)];

% form array - baseline
baseline_vx = [vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1) vel_est_no_disturbance(:,5)];
baseline_vy = [vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1) vel_est_no_disturbance(:,6)];
baseline_vz = [vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1) vel_est_no_disturbance(:,7)];

baseline_yaw = [est_pose_baseline(:,1)-est_pose_baseline(1,1) est_pose_baseline_Euler(:,1)];
baseline_pitch = [est_pose_baseline(:,1)-est_pose_baseline(1,1) est_pose_baseline_Euler(:,2)];
baseline_roll = [est_pose_baseline(:,1)-est_pose_baseline(1,1) est_pose_baseline_Euler(:,3)];

% form array - DOB
DOB_vx = [vel_est(1:end,1)-vel_est(1,1) vel_est(:,5)];
DOB_vy = [vel_est(1:end,1)-vel_est(1,1) vel_est(:,6)];
DOB_vz = [vel_est(1:end,1)-vel_est(1,1) vel_est(:,7)];


DOB_yaw = [est_pose(:,1)-est_pose(1,1) est_pose_Euler(:,1)];
DOB_pitch = [est_pose(:,1)-est_pose(1,1) est_pose_Euler(:,2)];
DOB_roll = [est_pose(:,1)-est_pose(1,1) est_pose_Euler(:,3)];


% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vx_extracted = [];
DOB_vx_extracted = [];
for i=1427:1621
    
    [closest_time,index] = min(abs(baseline_vx(:,1)-orb_vx(i,1)));
    baseline_vx_extracted = [baseline_vx_extracted;baseline_vx(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vx(:,1)-orb_vx(i,1)));
    DOB_vx_extracted = [DOB_vx_extracted;DOB_vx(index, 2)];
   
end

rmse_baseline_vx = sqrt(immse(baseline_vx_extracted, orb_vx(1427:1621,2)));
rmse_DOB_vx = sqrt(immse(DOB_vx_extracted, orb_vx(1427:1621,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vy_extracted = [];
DOB_vy_extracted = [];
for i=1427:1621
    
    [closest_time,index] = min(abs(baseline_vy(:,1)-orb_vy(i,1)));
    baseline_vy_extracted = [baseline_vy_extracted;baseline_vy(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vy(:,1)-orb_vy(i,1)));
    DOB_vy_extracted = [DOB_vy_extracted;DOB_vy(index, 2)];
   
end

rmse_baseline_vy = sqrt(immse(baseline_vy_extracted, orb_vy(1427:1621,2)));
rmse_DOB_vy = sqrt(immse(DOB_vy_extracted, orb_vy(1427:1621,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vz_extracted = [];
DOB_vz_extracted = [];
for i=1427:1621
    
    [closest_time,index] = min(abs(baseline_vz(:,1)-orb_vz(i,1)));
    baseline_vz_extracted = [baseline_vz_extracted;baseline_vz(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vz(:,1)-orb_vz(i,1)));
    DOB_vz_extracted = [DOB_vz_extracted;DOB_vz(index, 2)];
   
end

rmse_baseline_vz = sqrt(immse(baseline_vz_extracted, orb_vz(1427:1621,2)));
rmse_DOB_vz = sqrt(immse(DOB_vz_extracted, orb_vz(1427:1621,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_yaw_extracted = [];
DOB_yaw_extracted = [];
for i=1427:1621
    
    [closest_time,index] = min(abs(baseline_yaw(:,1)-orb_yaw(i,1)));
    baseline_yaw_extracted = [baseline_yaw_extracted;baseline_yaw(index, 2)];
    
    [closest_time,index] = min(abs(DOB_yaw(:,1)-orb_yaw(i,1)));
    DOB_yaw_extracted = [DOB_yaw_extracted;DOB_yaw(index, 2)];
   
end

rmse_baseline_yaw = sqrt(immse(baseline_yaw_extracted, orb_yaw(1427:1621,2)));
rmse_DOB_yaw = sqrt(immse(DOB_yaw_extracted, orb_yaw(1427:1621,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_pitch_extracted = [];
DOB_pitch_extracted = [];
for i=1427:1621
    
    [closest_time,index] = min(abs(baseline_pitch(:,1)-orb_pitch(i,1)));
    baseline_pitch_extracted = [baseline_pitch_extracted;baseline_pitch(index, 2)];
    
    [closest_time,index] = min(abs(DOB_pitch(:,1)-orb_pitch(i,1)));
    DOB_pitch_extracted = [DOB_pitch_extracted;DOB_pitch(index, 2)];
   
end

rmse_baseline_pitch = sqrt(immse(baseline_pitch_extracted, orb_pitch(1427:1621,2)));
rmse_DOB_ptich = sqrt(immse(DOB_pitch_extracted, orb_pitch(1427:1621,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_roll_extracted = [];
DOB_roll_extracted = [];
for i=1427:1621
    
    [closest_time,index] = min(abs(baseline_roll(:,1)-orb_roll(i,1)));
    baseline_roll_extracted = [baseline_roll_extracted;baseline_roll(index, 2)];
    
    [closest_time,index] = min(abs(DOB_roll(:,1)-orb_roll(i,1)));
    DOB_roll_extracted = [DOB_roll_extracted;DOB_roll(index, 2)];
   
end

rmse_baseline_roll = sqrt(immse(baseline_roll_extracted, orb_roll(1427:1621,2)));
rmse_DOB_roll = sqrt(immse(DOB_roll_extracted, orb_roll(1427:1621,2)));
%%

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vx_extracted = [];
DOB_vx_extracted = [];
for i=662:858
    
    [closest_time,index] = min(abs(baseline_vx(:,1)-orb_vx(i,1)));
    baseline_vx_extracted = [baseline_vx_extracted;baseline_vx(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vx(:,1)-orb_vx(i,1)));
    DOB_vx_extracted = [DOB_vx_extracted;DOB_vx(index, 2)];
   
end

rmse_baseline_vx_2 = sqrt(immse(baseline_vx_extracted, orb_vx(662:858,2)));
rmse_DOB_vx_2 = sqrt(immse(DOB_vx_extracted, orb_vx(662:858,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vy_extracted = [];
DOB_vy_extracted = [];
for i=662:858
    
    [closest_time,index] = min(abs(baseline_vy(:,1)-orb_vy(i,1)));
    baseline_vy_extracted = [baseline_vy_extracted;baseline_vy(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vy(:,1)-orb_vy(i,1)));
    DOB_vy_extracted = [DOB_vy_extracted;DOB_vy(index, 2)];
   
end

rmse_baseline_vy_2 = sqrt(immse(baseline_vy_extracted, orb_vy(662:858,2)));
rmse_DOB_vy_2 = sqrt(immse(DOB_vy_extracted, orb_vy(662:858,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_vz_extracted = [];
DOB_vz_extracted = [];
for i=662:858
    
    [closest_time,index] = min(abs(baseline_vz(:,1)-orb_vz(i,1)));
    baseline_vz_extracted = [baseline_vz_extracted;baseline_vz(index, 2)];
    
    [closest_time,index] = min(abs(DOB_vz(:,1)-orb_vz(i,1)));
    DOB_vz_extracted = [DOB_vz_extracted;DOB_vz(index, 2)];
   
end

rmse_baseline_vz_2 = sqrt(immse(baseline_vz_extracted, orb_vz(662:858,2)));
rmse_DOB_vz_2 = sqrt(immse(DOB_vz_extracted, orb_vz(662:858,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_yaw_extracted = [];
DOB_yaw_extracted = [];
for i=662:858
    
    [closest_time,index] = min(abs(baseline_yaw(:,1)-orb_yaw(i,1)));
    baseline_yaw_extracted = [baseline_yaw_extracted;baseline_yaw(index, 2)];
    
    [closest_time,index] = min(abs(DOB_yaw(:,1)-orb_yaw(i,1)));
    DOB_yaw_extracted = [DOB_yaw_extracted;DOB_yaw(index, 2)];
   
end

rmse_baseline_yaw_2 = sqrt(immse(baseline_yaw_extracted, orb_yaw(662:858,2)));
rmse_DOB_yaw_2 = sqrt(immse(DOB_yaw_extracted, orb_yaw(662:858,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_pitch_extracted = [];
DOB_pitch_extracted = [];
for i=662:858
    
    [closest_time,index] = min(abs(baseline_pitch(:,1)-orb_pitch(i,1)));
    baseline_pitch_extracted = [baseline_pitch_extracted;baseline_pitch(index, 2)];
    
    [closest_time,index] = min(abs(DOB_pitch(:,1)-orb_pitch(i,1)));
    DOB_pitch_extracted = [DOB_pitch_extracted;DOB_pitch(index, 2)];
   
end

rmse_baseline_pitch_2 = sqrt(immse(baseline_pitch_extracted, orb_pitch(662:858,2)));
rmse_DOB_ptich_2 = sqrt(immse(DOB_pitch_extracted, orb_pitch(662:858,2)));

% find the closest point in baseline and DOB compared with Orb-SLAM3
baseline_roll_extracted = [];
DOB_roll_extracted = [];
for i=662:858
    
    [closest_time,index] = min(abs(baseline_roll(:,1)-orb_roll(i,1)));
    baseline_roll_extracted = [baseline_roll_extracted;baseline_roll(index, 2)];
    
    [closest_time,index] = min(abs(DOB_roll(:,1)-orb_roll(i,1)));
    DOB_roll_extracted = [DOB_roll_extracted;DOB_roll(index, 2)];
   
end

rmse_baseline_roll_2 = sqrt(immse(baseline_roll_extracted, orb_roll(662:858,2)));
rmse_DOB_roll_2 = sqrt(immse(DOB_roll_extracted, orb_roll(662:858,2)));
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


