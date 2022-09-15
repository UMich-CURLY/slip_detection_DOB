


%% Read Bag into mat file
bagFile = rosbag('/media/xihang/761C73981C7351DB/husky_rosbag/rosbag_data/slip_detect_name/slip_detect7.bag');

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

%% plot vel in body frame
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
%%

vel_est = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_vel_est.txt");
imu = load("/home/xihang/Code/slip_detector_test(world_frame)/catkin_ws/src/slip_detector/data/2022-slip_detection/slip_detect7_vis_paper/wheel_imu.txt");


param.vcam = [vel_x -vel_y -vel_z];
param.tcam = vel_mat_orbslam3(1:end,1)-vel_mat_orbslam3(1,1);
param.vcar = [vel_est(:,5) vel_est(:,6) vel_est(:,7)];
param.tcar = vel_est(:, 1)-vel_est(1, 1);
param.wcar = [imu(:,2) imu(:,3) imu(:,4)];
param.twcar = imu(:,1)-imu(1,1);

lb = [-1, -1, -1, -1, -1, -1] * 3;
ub = [1, 1, 1, 1, 1, 1] * 3;

options = optimoptions('fmincon', 'Display', 'iter', 'StepTolerance', 1e-16);

[sol, fval] = fmincon(@(x)cost(x, param), randn(6, 1) * 0.01, [], [], [], [], lb, ub, [], options);

sol'
Rx(sol(1)) * Ry(sol(2)) * Rz(sol(3))
%%
function J = cost(x, param)
vcam = param.vcam;
tvcam = param.tcam;

vcar = param.vcar;
tvcar = param.tcar;

wcar = param.wcar;
twcar = param.twcar;

tvcam_ = interp1(tvcam, tvcam, tvcar);
vcam_ = interp1(tvcam, vcam, tvcar);

twcar_ = interp1(twcar, twcar, tvcar);
wcar_ = interp1(twcar, wcar, tvcar);

nan_list = ~isnan(tvcam_ + twcar_);

wcar = wcar_(nan_list, :);
vcam = vcam_(nan_list, :);
vcar = vcar(nan_list, :);

R = Rx(x(1)) * Ry(x(2)) * Rz(x(3));

err = (vcar' - skew(x(4:6)) * wcar') - R*vcam';
J = norm(err);
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