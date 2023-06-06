#include "system/husky_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/timer/timer.hpp>

#include <vector>
#include <numeric>

HuskySystem::HuskySystem(ros::NodeHandle *nh, husky_inekf::husky_data_t *husky_data_buffer) : nh_(nh), husky_data_buffer_(husky_data_buffer), pose_publisher_node_(nh), new_pose_ready_(false)
{

    // initialize velocity noise
    // TODO: integrate them into noiseParams
    double std;
    // wheel velocity covariance
    nh_->param<double>("/noise/wheel_vel_std", std, 0.05);
    wheel_vel_cov_ = std * std * Eigen::Matrix<double, 3, 3>::Identity();
    // wheel_vel_cov_.block<2,2>(1,1) = 0.02 * std * std * Eigen::Matrix<double,2,2>::Identity();

    // camera velocity covariance
    nh_->param<double>("/noise/camera_vel_std", std, 0.05);
    camera_vel_cov_ = std * std * Eigen::Matrix<double, 3, 3>::Identity();

    // gps velocity covariance
    nh_->param<double>("/noise/gps_vel_std", std, 0.05);
    gps_vel_cov_ = std * std * Eigen::Matrix<double, 3, 3>::Identity();

    // set velocity update methods
    nh_->param<bool>("/settings/enable_wheel_velocity_update", enable_wheel_vel_, true);
    nh_->param<bool>("/settings/enable_camera_velocity_update", enable_camera_vel_, false);
    nh_->param<bool>("/settings/enable_gps_velocity_update", enable_gps_vel_, false);

    // Initialize pose publishing if requested
    nh_->param<bool>("/settings/enable_pose_publisher", enable_pose_publisher_, false);
    nh_->param<bool>("/settings/enable_pose_logger", enable_pose_logger_, false);
    nh_->param<bool>("/settings/enable_debug_logger", enable_debug_logger_, false);
    nh_->param<int>("/settings/log_pose_skip", log_pose_skip_, 100);

    // Initialize inekf pose file printouts
    nh_->param<std::string>("/settings/inekf_pose_filename", file_name_,
                            "husky_inekf_pose.txt");
    nh_->param<std::string>("/settings/inekf_vel_est_file_name", vel_est_file_name_,
                            "vel_est.txt");
    nh_->param<std::string>("/settings/inekf_bias_est_file_name", bias_est_file_name_,
                            "bias_est.txt");
    nh_->param<std::string>("/settings/inekf_disturbance_file_name", disturbance_est_file_name_,
                            "bias_est.txt");
    nh_->param<std::string>("/settings/inekf_vel_input_file_name", vel_input_file_name_,
                            "vel_input.txt");
    nh_->param<std::string>("/settings/inekf_imu_file_name", imu_file_name_,
                            "imu.txt");

    nh_->param<bool>("/settings/enable_friction_estimator", enable_friction_estimator, false);

    nh_->param<bool>("/settings/enable_body_vel_est", BodyVelEstModeOn_, false);

    outfile_.open(file_name_, std::ofstream::out);
    vel_est_outfile_.open(vel_est_file_name_, std::ofstream::out);
    bias_est_outfile_.open(bias_est_file_name_, std::ofstream::out);
    disturbance_est_outfile_.open(disturbance_est_file_name_, std::ofstream::out);
    outfile_.precision(20);
    vel_est_outfile_.precision(20);
    bias_est_outfile_.precision(20);
    disturbance_est_outfile_.precision(20);

    if (enable_debug_logger_)
    {
        vel_input_outfile_.open(vel_input_file_name_, std::ofstream::out);
        imu_outfile_.open(imu_file_name_, std::ofstream::out);
        vel_input_outfile_.precision(20);
        imu_outfile_.precision(20);
    }

    last_imu_time_ = 0;
    skip_count_ = 0;
}

HuskySystem::~HuskySystem()
{
    std::cout << "Ready to close Husky system" << std::endl;
    outfile_.close();
    vel_est_outfile_.close();
    bias_est_outfile_.close();
    vel_input_outfile_.close();
    imu_outfile_.close();
    disturbance_est_outfile_.close();
}

void HuskySystem::step()
{

    // if the estimator is initialized
    if (estimator_.enabled())
    {

        // if IMU measurement exists we do prediction
        if (updateNextIMU())
        {
            estimator_.propagateIMU(*(imu_packet_.get()), state_);
            if (enable_debug_logger_)
            {
                imu_outfile_ << imu_packet_->getTime() << " "
                             << imu_packet_->angular_velocity.x << " "
                             << imu_packet_->angular_velocity.y << " "
                             << imu_packet_->angular_velocity.z << " "
                             << imu_packet_->linear_acceleration.x << " "
                             << imu_packet_->linear_acceleration.y << " "
                             << imu_packet_->linear_acceleration.z << std::endl
                             << std::flush;
            }
            new_pose_ready_ = true;
        }

        // update using body velocity from wheel encoders
        if (enable_wheel_vel_ && updateNextWheelVelocity())
        {
            if (BodyVelEstModeOn_)
            {
                auto disturbance_est = state_.getDisturbance();
                double dist_x = disturbance_est(0);
                double dist_y = disturbance_est(1);
                double dist_z = disturbance_est(2);
                double disturbance_abs = sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
                wheel_vel_cov_adapt_ = wheel_vel_cov_ * exp(disturbance_abs);
            }

            estimator_.correctVelocity(*(wheel_velocity_packet_.get()), state_, wheel_vel_cov_adapt_);
            new_pose_ready_ = true;

            // record
            auto v_in = wheel_velocity_packet_->getLinearVelocity();
            if (enable_debug_logger_)
            {
                vel_input_outfile_ << wheel_velocity_packet_->getTime() << " " << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl
                                   << std::flush;
            }
        }

        // update using camera velocity
        if (enable_camera_vel_ && updateNextCameraVelocity())
        {
            estimator_.correctVelocity(*(camera_velocity_packet_.get()), state_, camera_vel_cov_);
            new_pose_ready_ = true;

            // record
            auto v_in = camera_velocity_packet_->getLinearVelocity();
            if (enable_debug_logger_)
            {
                vel_input_outfile_ << camera_velocity_packet_->getTime() << " " << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl
                                   << std::flush;
            }
        }

        // update using gps velocity
        if (enable_gps_vel_ && updateNextGPSVelocity())
        {
            estimator_.correctVelocity(*(gps_velocity_packet_.get()), state_, gps_vel_cov_);
            new_pose_ready_ = true;

            // record
            auto v_in = gps_velocity_packet_->getLinearVelocity();
            if (enable_debug_logger_)
            {
                vel_input_outfile_ << gps_velocity_packet_->getTime() << " " << v_in(0) << " " << v_in(1) << " " << v_in(2) << std::endl
                                   << std::flush;
            }
        }

        if (enable_pose_publisher_ && new_pose_ready_)
        {
            pose_publisher_node_.posePublish(state_);
            pose_publisher_node_.velPublish(state_);
            pose_publisher_node_.slipPublish(state_);
            pose_publisher_node_.slipFlagPublish(state_);
        }

        if (new_pose_ready_)
        {
            slipEstimator(state_);
            slipEstimator_SlipModel(state_);
        }

        if (enable_friction_estimator && new_pose_ready_)
        {
            frictionEstimator(state_);
        }
        if (enable_pose_logger_ && new_pose_ready_)
        {
            logPoseTxt(state_);
        }

        new_pose_ready_ = false;
    }
    // initialization
    else
    {
        if (estimator_.biasInitialized())
        {
            // wait until we receive imu msg
            while (!updateNextIMU())
            {
            };

            if (enable_wheel_vel_)
            {
                while (!updateNextWheelVelocity())
                {
                }
                estimator_.initState(*(imu_packet_.get()), *(wheel_velocity_packet_.get()), state_);
            }
            else if (enable_camera_vel_)
            {
                while (!updateNextCameraVelocity())
                {
                }
                estimator_.initState(*(imu_packet_.get()), *(camera_velocity_packet_.get()), state_);
            }
            else if (enable_gps_vel_)
            {
                while (!updateNextGPSVelocity())
                {
                }
                estimator_.initState(*(imu_packet_.get()), *(gps_velocity_packet_.get()), state_);
            }

            estimator_.enableFilter();
            husky_data_buffer_->wheel_velocity_q = {};
            husky_data_buffer_->camera_velocity_q = {};
            husky_data_buffer_->gps_velocity_q = {};

            std::cout << "State initialized." << std::endl;
        }
        else
        {
            while (!updateNextIMU())
            {
            };
            estimator_.initBias(*(imu_packet_.get()));
        }
    }
}

void HuskySystem::frictionEstimator(const husky_inekf::HuskyState &state)
{

    // Extract out current IMU data [w;a]
    Eigen::Matrix<double, 6, 1> imu;
    imu << imu_packet_->angular_velocity.x,
        imu_packet_->angular_velocity.y,
        imu_packet_->angular_velocity.z,
        imu_packet_->linear_acceleration.x,
        imu_packet_->linear_acceleration.y,
        imu_packet_->linear_acceleration.z;
    double t = imu_packet_->getTime();

    // Bias corrected IMU measurements
    Eigen::Vector3d a = imu.tail(3) - state_.getImuBias().tail(3); // Linear Acceleration
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d g;
    g << 0, 0, -9.81;
    a = (R.transpose() * (R * a + g)).eval();

    auto disturbance_est = state_.getDisturbance();
    double dist_x = disturbance_est(0);
    double dist_y = disturbance_est(1);
    double dist_z = disturbance_est(2);
    double disturbance_abs = sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);

    double a_abs = sqrt(a(0) * a(0) + a(1) * a(1) + a(2) * a(2));

    // std::cout << "time: " << t << " disturbance_abs: " << disturbance_abs << std::endl;

    if (slip_flag_1)
    {
        mu = a_abs / 9.81;
    }
}

void HuskySystem::slipEstimator(husky_inekf::HuskyState &state)
{

    // // Extract out current IMU data [w;a]
    // Eigen::Matrix<double,6,1> imu;
    // imu << imu_packet_->angular_velocity.x,
    //        imu_packet_->angular_velocity.y,
    //        imu_packet_->angular_velocity.z,
    //        imu_packet_->linear_acceleration.x,
    //        imu_packet_->linear_acceleration.y ,
    //        imu_packet_->linear_acceleration.z;
    // double t = imu_packet_->getTime();

    // // Bias corrected IMU measurements
    // Eigen::Vector3d a = imu.tail(3) - state_.getImuBias().tail(3); // Linear Acceleration
    // Eigen::Matrix3d R = state_.getRotation();
    // Eigen::Vector3d v = state_.getWorldVelocity();
    // Eigen::MatrixXd P = state_.getP();

    // Eigen::Vector3d measured_velocity = (*(wheel_velocity_packet_.get())).getLinearVelocity();

    // Eigen::MatrixXd G;
    // G.conservativeResize(3, 3);
    // G.block(0,0,3,3) = R.transpose();

    // Eigen::Matrix3d Sigma = G* P.block(9,9,3,3)* G.transpose();

    Eigen::MatrixXd P = state_.getP();
    Eigen::Matrix3d R = state_.getRotation();
    auto disturbance_est = state_.getDisturbance();

    Eigen::MatrixXd G;
    G.conservativeResize(3, 6);
    G.block(0, 0, 3, 3) = skew(disturbance_est);
    G.block(0, 3, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);

    Eigen::MatrixXd Cov;
    Cov.conservativeResize(6, 6);
    Cov.block(0, 0, 3, 3) = P.block(0, 0, 3, 3);
    Cov.block(0, 3, 3, 3) = P.block(0, 9, 3, 3);
    Cov.block(3, 0, 3, 3) = P.block(9, 0, 3, 3);
    Cov.block(3, 3, 3, 3) = P.block(9, 9, 3, 3);

    Eigen::Matrix3d Sigma = G * Cov * G.transpose();

    // std::cout << "Sigma: " << Sigma << std::endl;

    // std::cout << "disturbance_est: " << disturbance_est << std::endl;

    // std::cout << "P.block(9,9,3,3): " << P.block(9, 9, 3, 3) << std::endl;

    Eigen::Matrix3d Sigma1 = 0.001 * Eigen::MatrixXd::Identity(3, 3);

    chi = (disturbance_est).transpose() * (Sigma1).inverse() * (disturbance_est);

    // std::cout << "chi: " << chi << std::endl;
    if (chi > 4.642)
    {
        slip_flag_1 = 1;
    }
    else
    {
        slip_flag_1 = 0;
    }

    state.slip_flag = slip_flag_1;

    // // Threshold:
    // auto disturbance_est = state_.getDisturbance();
    // double dist_x = disturbance_est(0);
    // double dist_y = disturbance_est(1);
    // double dist_z = disturbance_est(2);
    // double disturbance_abs = sqrt(dist_x*dist_x+dist_y*dist_y+dist_z*dist_z);
    // std::cout << "disturbance_abs: " << disturbance_abs << std::endl;
    // if (disturbance_abs > 0.1){
    //     slip_flag_1 = 1;
    // }
    // else{
    //     slip_flag_1 = 0;
    // }
    // std::cout << "slip_flag_1: " << slip_flag_1 << std::endl;
    // state.slip_flag = slip_flag_1;
}

void HuskySystem::slipEstimator_SlipModel(const husky_inekf::HuskyState &state)
{

    // Extract out current IMU data [w;a]
    Eigen::Matrix<double, 6, 1> imu;
    imu << imu_packet_->angular_velocity.x,
        imu_packet_->angular_velocity.y,
        imu_packet_->angular_velocity.z,
        imu_packet_->linear_acceleration.x,
        imu_packet_->linear_acceleration.y,
        imu_packet_->linear_acceleration.z;
    double t = imu_packet_->getTime();

    // Bias corrected IMU measurements
    Eigen::Vector3d a = imu.tail(3) - state_.getImuBias().tail(3); // Linear Acceleration
    Eigen::Matrix3d R = state_.getRotation();
    Eigen::Vector3d v = state_.getWorldVelocity();
    Eigen::MatrixXd P = state_.getP();
    Eigen::Vector3d disturbance = state_.getDisturbance();

    Eigen::Vector3d measured_velocity = (*(wheel_velocity_packet_.get())).getLinearVelocity();

    Eigen::MatrixXd G;
    G.conservativeResize(3, 6);
    G.block(0, 0, 3, 3) = R.transpose();
    G.block(0, 3, 3, 3) = R.transpose();

    Eigen::MatrixXd Cov;
    Cov.conservativeResize(6, 6);
    Cov.block(0, 0, 3, 3) = P.block(3, 3, 3, 3);
    Cov.block(0, 3, 3, 3) = P.block(0, 9, 3, 3);
    Cov.block(3, 0, 3, 3) = P.block(9, 0, 3, 3);
    Cov.block(3, 3, 3, 3) = P.block(9, 9, 3, 3);

    Eigen::Matrix3d Sigma = G * Cov * G.transpose();

    chi_2 = (measured_velocity - R.transpose() * v - R.transpose() * disturbance).transpose() * (Sigma + 0.0001 * Eigen::MatrixXd::Identity(3, 3)).inverse() * (measured_velocity - R.transpose() * v - R.transpose() * disturbance);

    if (chi_2 > 4.642)
    {
        slip_flag_2 = 1;
    }
    else
    {
        slip_flag_2 = 0;
    }
}

void HuskySystem::logPoseTxt(const husky_inekf::HuskyState &state_)
{
    if (skip_count_ == 0)
    {
        // ROS_INFO_STREAM("write new pose\n");
        double t = state_.getTime();

        // log pose tum style
        outfile_ << t << " " << state_.x() << " " << state_.y() << " " << state_.z() << " " << state_.getQuaternion().x()
                 << " " << state_.getQuaternion().y() << " " << state_.getQuaternion().z() << " " << state_.getQuaternion().w() << std::endl
                 << std::flush;

        // log estimated velocity
        auto vel_est = state_.getWorldVelocity();
        auto vel_est_body = state_.getBodyVelocity();
        vel_est_outfile_ << t << " " << vel_est(0) << " " << vel_est(1) << " " << vel_est(2) << " " << vel_est_body(0) << " " << vel_est_body(1) << " " << vel_est_body(2) << std::endl
                         << std::flush;

        // log estimated bias
        auto bias_est = state_.getImuBias();
        bias_est_outfile_ << t << " " << bias_est(0) << " " << bias_est(1) << " " << bias_est(2) << " " << bias_est(3)
                          << " " << bias_est(4) << " " << bias_est(5) << std::endl
                          << std::flush;

        // std::cout << "slip_flag_1: " << slip_flag_1 << std::endl;

        // log estimated disturbance
        auto disturbance_est = state_.getDisturbance();
        disturbance_est_outfile_ << t << " " << disturbance_est(0) << " " << disturbance_est(1) << " " << disturbance_est(2) << " " << mu << " " << slip_flag_1 << " " << chi_2 << " " << chi << std::endl
                                 << std::flush;

        skip_count_ = log_pose_skip_;
    }
    else
    {
        skip_count_--;
        // std::cout<<"skipping--"<<std::endl;
    }
}

// Private Functions
bool HuskySystem::updateNextIMU()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->imu_mutex);
    if (!husky_data_buffer_->imu_q.empty())
    {

        if (husky_data_buffer_->imu_q.size() > 1)
        {
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("IMU queue size: " << husky_data_buffer_->imu_q.size());
        }
        imu_packet_ = husky_data_buffer_->imu_q.front();
        husky_data_buffer_->imu_q.pop();
        // Update Husky State
        state_.setImu(imu_packet_);

        return true;
    }
    // ROS_INFO_STREAM("!!!imu not received!!!");
    return false;
}

bool HuskySystem::updateNextJointState()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->joint_state_mutex);
    if (!husky_data_buffer_->joint_state_q.empty())
    {

        if (husky_data_buffer_->joint_state_q.size() > 1)
        {
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Joint state queue size: " << husky_data_buffer_->joint_state_q.size());
        }

        joint_state_packet_ = husky_data_buffer_->joint_state_q.front();
        husky_data_buffer_->joint_state_q.pop();

        // Update Husky State
        state_.setJointState(joint_state_packet_);

        return true;
    }
    return false;
}

bool HuskySystem::updateNextWheelVelocity()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->wheel_vel_mutex);

    if (!husky_data_buffer_->wheel_velocity_q.empty())
    {

        if (husky_data_buffer_->wheel_velocity_q.size() > 1)
        {
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Velocity queue size: " << husky_data_buffer_->wheel_velocity_q.size());
        }

        wheel_velocity_packet_ = husky_data_buffer_->wheel_velocity_q.front();
        husky_data_buffer_->wheel_velocity_q.pop();

        return true;
    }
    // std::cout<<"velocity q empty... "<<std::endl;
    return false;
}

bool HuskySystem::updateNextCameraVelocity()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->cam_vel_mutex);

    if (!husky_data_buffer_->camera_velocity_q.empty())
    {

        if (husky_data_buffer_->camera_velocity_q.size() > 1)
        {
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Velocity queue size: " << husky_data_buffer_->camera_velocity_q.size());
        }

        camera_velocity_packet_ = husky_data_buffer_->camera_velocity_q.front();
        husky_data_buffer_->camera_velocity_q.pop();

        return true;
    }
    // std::cout<<"velocity q empty... "<<std::endl;
    return false;
}

bool HuskySystem::updateNextGPSVelocity()
{
    std::lock_guard<std::mutex> lock(husky_data_buffer_->gps_vel_mutex);

    if (!husky_data_buffer_->gps_velocity_q.empty())
    {

        if (husky_data_buffer_->gps_velocity_q.size() > 1)
        {
            ROS_INFO_STREAM("Filter not running in real-time!");
            ROS_INFO_STREAM("Velocity queue size: " << husky_data_buffer_->gps_velocity_q.size());
        }

        gps_velocity_packet_ = husky_data_buffer_->gps_velocity_q.front();
        husky_data_buffer_->gps_velocity_q.pop();

        return true;
    }
    // std::cout<<"velocity q empty... "<<std::endl;
    return false;
}