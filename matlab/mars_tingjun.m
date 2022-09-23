%% Initialize
clear all;
close all;
clc


%% Parameters
fsize = 12;

vel_in = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_vel_input.txt");
bias_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_bias_est.txt");
vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_vel_est.txt");
gt_pose = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair_gt/1/trial1_rectangle_center.txt");
est_pose = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/inekf_wheel_vel.txt");
disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_disturbance_est.txt");
t = est_pose(:,1);
imu = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_imu.txt");

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

%% plot vel in world frame

% Read Bag into mat file
bagFile = rosbag('/media/xihang/761C73981C7351DB/husky_rosbag/rosbag_data/slip_detect_mars/up_to_hill3.bag');
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
    vel_vec = undwedged_se3(vel_wedge)'; % 1x6
    vel_mat_orbslam3 = [vel_mat_orbslam3;orbslam3_Mat(i,1) vel_vec];
end

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

vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_vel_est.txt");
vel_est_no_disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_no_disturbance/wheel_vel_est.txt");

subplot(3,2,1)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,vel_world(:,1),'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,2),'b-.','LineWidth',1.3);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,2),'r-.','LineWidth',1.3);
xlim([5,25]);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
box on 
grid on

subplot(3,2,3)
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1)-0.1,-vel_world(:,2),'k','LineWidth',1.5);
plot(vel_est_no_disturbance(1:end,1)-vel_est_no_disturbance(1,1),vel_est_no_disturbance(:,3),'b-.','LineWidth',0.9);
plot(vel_est(1:end,1)-vel_est(1,1),vel_est(:,3),'r-.','LineWidth',0.9);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
xlim([5,25]);
box on 
grid on

subplot(3,2,5)
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


disturbance = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/up_to_hill3_slip_detection/wheel_disturbance_est.txt");

subplot(3,2,2)
line1 = plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance(1:end,2)),'Color','r','LineWidth',1.5);
hold on
box on 
grid on
xlim([5,25]);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$u_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(3,2,4)
line2 = plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance(1:end,3)),'Color', 'r','LineWidth',1.5);
hold on
box on 
grid on
xlim([5,25]);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$u_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')

subplot(3,2,6)
line3 = plot(disturbance(1:end,1)-disturbance(1,1),abs(disturbance(1:end,4)),'Color', 'r','LineWidth',1.5);
hold on
box on 
grid on
xlim([5,25]);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$u_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
hold on

%% Functions
function twist_vec = undwedged_se3(twist_mat)
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

