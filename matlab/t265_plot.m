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

% InEKF
InEKF_Sel = select(bagFile,'Topic','/husky/inekf_estimation/pose');
InEKF_Data = readMessages(InEKF_Sel,'DataFormat','struct');

InEKF_px = cellfun(@(m) double(m.Pose.Pose.Position.X),InEKF_Data);
InEKF_py = cellfun(@(m) double(m.Pose.Pose.Position.Y),InEKF_Data);
InEKF_pz = cellfun(@(m) double(m.Pose.Pose.Position.Z),InEKF_Data);

InEKF_ox = cellfun(@(m) double(m.Pose.Pose.Orientation.X),InEKF_Data);
InEKF_oy = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),InEKF_Data);
InEKF_oz = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),InEKF_Data);
InEKF_ow = cellfun(@(m) double(m.Pose.Pose.Orientation.W),InEKF_Data);

NSec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Nsec),InEKF_Data,'UniformOutput',false));
Sec = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec),InEKF_Data,'UniformOutput',false));
Sec_tot = Sec + NSec*10^(-9);
InEKF_t = Sec_tot - Sec_tot(1,1);

InEKF_Mat = [InEKF_t InEKF_px InEKF_py InEKF_pz InEKF_ow InEKF_ox InEKF_oy InEKF_oz];


%% Calculating estimated vel from pose
vel_mat = [];
vel_inekf = [];

for i=2:length(t265_ow)
    prev_T = [quat2rotm(t265_Mat(i-1,5:8)) [t265_Mat(i-1,2);t265_Mat(i-1,3);t265_Mat(i-1,4)]; 0 0 0 1];
    current_T = [quat2rotm(t265_Mat(i,5:8)) [t265_Mat(i,2);t265_Mat(i,3);t265_Mat(i,4)]; 0 0 0 1];
    del_t = t265_Mat(i,1) - t265_Mat(i-1,1);
    vel_wedge = logm(prev_T\current_T)/del_t; % Left Invariant
    vel_vec = undwedge_se3(vel_wedge)'; % 1x6
    vel_mat = [vel_mat;t265_Mat(i,1) vel_vec];
end

for i=2:length(InEKF_ow)
    prev_T = [quat2rotm(InEKF_Mat(i-1,5:8)) [InEKF_Mat(i-1,2);InEKF_Mat(i-1,3);InEKF_Mat(i-1,4)]; 0 0 0 1];
    current_T = [quat2rotm(InEKF_Mat(i,5:8)) [InEKF_Mat(i,2);InEKF_Mat(i,3);InEKF_Mat(i,4)]; 0 0 0 1];
    del_t = InEKF_Mat(i,1) - InEKF_Mat(i-1,1);
    vel_wedge = logm(prev_T\current_T)/del_t; % Left Invariant
    vel_vec = undwedge_se3(vel_wedge)'; % 1x6
    vel_inekf = [vel_inekf;InEKF_Mat(i,1) vel_vec];
end
%% Plotting
figure(1)
hold on
plot3(t265_Mat(:,2),t265_Mat(:,3),t265_Mat(:,4));
plot3(InEKF_Mat(:,2),InEKF_Mat(:,3),InEKF_Mat(:,4));
xlabel('$x \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$y \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
zlabel('$z \; (m)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Pose','InEKF Pose', 'fontsize', fsize, 'Interpreter','latex');
axis equal tight 
grid on

figure(2)
subplot(3,1,1)
hold on
plot(t265_Mat(:,1),t265_Mat(:,9));
plot(vel_mat(:,1),vel_mat(:,2));
plot(vel_inekf(:,1),vel_inekf(:,2));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_x \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Velocity','Lie Group Velocity From T265', 'Lie Group Velocity From InEKF', 'fontsize', fsize, 'Interpreter','latex');
grid on

subplot(3,1,2)
hold on
plot(t265_Mat(:,1),t265_Mat(:,10));
plot(vel_mat(:,1),vel_mat(:,3));
plot(vel_inekf(:,1),vel_inekf(:,3));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_y \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Velocity','Lie Group Velocity From T265', 'Lie Group Velocity From InEKF', 'fontsize', fsize, 'Interpreter','latex');
grid on

subplot(3,1,3)
hold on
plot(t265_Mat(:,1),t265_Mat(:,11));
plot(vel_mat(:,1),vel_mat(:,4))
plot(vel_inekf(:,1),vel_inekf(:,4));
xlabel('$t \; (s)$', 'fontsize', fsize, 'Interpreter','latex')
ylabel('$v_z \; (m/s)$', 'fontsize', fsize, 'Interpreter','latex')
legend('T265 Velocity','Lie Group Velocity From T265', 'Lie Group Velocity From InEKF', 'fontsize', fsize, 'Interpreter','latex');
grid on

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