%% Initialize
clear all;
close all;
clc
%% Parameters
fsize = 20;
husky_width = (0.42+0.67)/2;
husky_length = 0.544;
husky_diag = sqrt(husky_width^2+husky_length^2);
r_wheel = [0; 0; -0.33/2]; % right hand rule at the contact point
robot_base = [husky_diag/2*cos(pi/4); husky_diag/2*cos(pi/4); 0]; % Need to check angle and direction for each wheel
slip_threshold = 0.01;

%% Read Bag into mat file
bagFile = rosbag('/media/xihang/761C73981C7351DB/husky_rosbag/rosbag_data/slip_detect_name/slip_detect7.bag');
%% Read from t265
StartTime = bagFile.StartTime;
EndTime = bagFile.EndTime;
TotalTime = EndTime - StartTime;

% T265
t265_Sel = select(bagFile,'Topic','/t265/odom/sample ');
t265_Data = readMessages(t265_Sel,'DataFormat','struct');

t265_px = cellfun(@(m) double(m.Pose.Pose.Position.X),t265_Data);
t265_py = cellfun(@(m) double(m.Pose.Pose.Position.Y),t265_Data);
t265_pz = cellfun(@(m) double(m.Pose.Pose.Position.Z),t265_Data);

t265_ox = cellfun(@(m) double(m.Pose.Pose.Orientation.X),t265_Data);
t265_oy = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),t265_Data);
t265_oz = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),t265_Data);
t265_ow = cellfun(@(m) double(m.Pose.Pose.Orientation.W),t265_Data);

t265_vx = cellfun(@(m) double(m.Twist.Twist.Linear.X),t265_Data);
t265_vy = cellfun(@(m) double(m.Twist.Twist.Linear.Y),t265_Data);
t265_vz = cellfun(@(m) double(m.Twist.Twist.Linear.Z),t265_Data);

t265_wx = cellfun(@(m) double(m.Twist.Twist.Angular.X),t265_Data);
t265_wy = cellfun(@(m) double(m.Twist.Twist.Angular.Y),t265_Data);
t265_wz = cellfun(@(m) double(m.Twist.Twist.Angular.Z),t265_Data);

NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),t265_Data,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),t265_Data,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
t265_t = Sec_tot - Sec_tot(1,1);

t265_Mat = [t265_t t265_px t265_py t265_pz t265_ow t265_ox t265_oy t265_oz t265_vx t265_vy t265_vz t265_wx t265_wy t265_wz];

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

%% Read from gps
gps = select(bagFile,'Topic','/gps/fix');
gps_data = readMessages(gps,'DataFormat','struct');

gps_lat = cellfun(@(m) double(m.Latitude),gps_data);
gps_lon = cellfun(@(m) double(m.Longitude),gps_data);


[gps_x, gps_y] = grn2eqa(gps_lat, gps_lon);

NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),gps_data,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),gps_data,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
gps_t = Sec_tot - Sec_tot(1,1);

gps_Mat = [gps_t gps_x gps_y];

%% Plotting traj from t265
figure(1)
hold on
plot3(t265_Mat(:,2),t265_Mat(:,3),t265_Mat(:,4));
% plot3(InEKF_Mat(:,2),InEKF_Mat(:,3),InEKF_Mat(:,4));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Pose','InEKF Pose', 'fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on

%% Plotting traj from Orb-SLAM3
figure(1)
hold on
plot3(orbslam3_Mat(:,2),orbslam3_Mat(:,3),orbslam3_Mat(:,4));
% plot3(InEKF_Mat(:,2),InEKF_Mat(:,3),InEKF_Mat(:,4));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('Orb-SLAM3 Pose', 'fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on

%% plot pose from Orb-SLAM3
figure(3)

plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1),orbslam3_Mat(:,2),'LineWidth',2);
hold on
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1),orbslam3_Mat(:,3),'LineWidth',2);
hold on
plot(orbslam3_Mat(:,1)-orbslam3_Mat(1,1),orbslam3_Mat(:,4),'LineWidth',2);
hold on

box on
grid on

%% Plotting traj from GPS
figure(1)
hold on
plot(gps_Mat(:,2),gps_Mat(:,3));
% plot3(InEKF_Mat(:,2),InEKF_Mat(:,3),InEKF_Mat(:,4));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('GPS Pose', 'fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on
%% Body Velocity from t265
figure(2)

hold on
plot(t265_Mat(:,1),t265_Mat(:,9));
plot(t265_Mat(:,1),t265_Mat(:,10));
plot(t265_Mat(:,1),t265_Mat(:,11));
% plot(vel_mat_orbslam3(:,1),vel_mat_orbslam3(:,2));
% plot(vel_inekf(:,1),vel_inekf(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Velocity x','T265 Velocity y', 'T265 Velocity z', 'fontsize', fsize, 'Interpreter','latex');
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

% plot vel
figure(2)

d = designfilt('lowpassfir', 'FilterOrder', 1, 'CutoffFrequency', 11, 'SampleRate', 200);
vel_x = filter(d, vel_mat_orbslam3(1:end,2));
vel_y = filter(d, vel_mat_orbslam3(1:end,3));
vel_z = filter(d, vel_mat_orbslam3(1:end,4));

subplot(3,1,1)
hold on
plot(t265_Mat(:,1),t265_Mat(:,9));
plot(vel_mat_orbslam3(:,1),vel_x);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Velocity','Lie Group Velocity From Orb-SLAM3', 'fontsize', fsize, 'Interpreter','latex');
grid on

subplot(3,1,2)
hold on
plot(t265_Mat(:,1),t265_Mat(:,10));
plot(vel_mat_orbslam3(:,1),vel_y);
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Velocity','Lie Group Velocity From Orb-SLAM3', 'fontsize', fsize, 'Interpreter','latex');
grid on

subplot(3,1,3)
hold on
plot(t265_Mat(:,1),t265_Mat(:,11));
plot(vel_mat_orbslam3(:,1),vel_z)
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Velocity','Lie Group Velocity From Orb-SLAM3', 'fontsize', fsize, 'Interpreter','latex');
grid on
%% vel est

vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7/wheel_vel_est.txt");

t = vel_est(1:end,1)-vel_est(1,1);


figure(15)

% d = designfilt('lowpassfir', 'FilterOrder', 5, 'CutoffFrequency', 11, 'SampleRate', 200);
% vel_x = filter(d, vel_mat_orbslam3(1:end,2));
% vel_y = filter(d, vel_mat_orbslam3(1:end,3));
% vel_z = filter(d, vel_mat_orbslam3(1:end,4));

plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1),vel_x,'LineWidth',1.5);
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1),vel_y,'LineWidth',1.5);
hold on
plot(vel_mat_orbslam3(:,1)-vel_mat_orbslam3(1,1),vel_z,'LineWidth',1.5);
hold on

legend("$disturbance_x$", "$disturbance_y$","$disturbance_z$", 'FontSize',12, "interpreter", "latex");

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
legend("$vel-x-Orb-SLAM3$", "$vel-y-Orb-SLAM3$","$vel-z-Orb-SLAM3$","$vel-x-DOB$", "$vel-y-DOB$","$vel-z-DOB$",'FontSize',12, "interpreter", "latex");



%% Motion Capture System as ground truth velocity
gt_pose = load("/home/xihang/Code/husky_inekf_plain/catkin_ws/src/husky_inekf/data/2022-05-11_mair_gt/1/trial1_rectangle_center.txt");
%% Calculating estimated vel from motion capture system
vel_mat_orbslam3 = [];

for i=2:length(orbslam3_ow)
    prev_T = [quat2rotm(t265_Mat(i-1,5:8)) [gt_pose(i-1,2);gt_pose(i-1,3);gt_pose(i-1,4)]; 0 0 0 1];
    current_T = [quat2rotm(orbslam3_Mat(i,5:8)) [gt_pose(i,2);gt_pose(i,3);gt_pose(i,4)]; 0 0 0 1];
    del_t = orbslam3_Mat(i,1) - orbslam3_Mat(i-1,1);
    vel_wedge = logm(inv(prev_T)*current_T)/del_t; % Left Invariant
    vel_vec = undwedge_se3(vel_wedge)'; % 1x6
    vel_mat_orbslam3 = [vel_mat_orbslam3;orbslam3_Mat(i,1) vel_vec];
end

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

