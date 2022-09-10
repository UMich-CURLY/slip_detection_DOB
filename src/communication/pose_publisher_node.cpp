#include "communication/pose_publisher_node.hpp"

PosePublisherNode::PosePublisherNode(ros::NodeHandle* n) : n_(n) {
    // Create private node handle
    ros::NodeHandle nh("~");
    // std::string pose_csv_file, init_rot_file;
    std::string pose_topic, pose_frame, slip_topic, vel_topic, slip_flag_topic;

    nh.param<std::string>("/settings/pose_topic", pose_topic, "/husky/inekf_estimation/pose");
    nh.param<std::string>("/settings/slip_topic", slip_topic, "/husky/inekf_estimation/slip");
    nh.param<std::string>("/settings/vel_topic", vel_topic, "/husky/inekf_estimation/body_velocity");
    nh.param<std::string>("/settings/vel_topic", slip_flag_topic, "/husky/inekf_estimation/slip_flag");


    nh.param<std::string>("/settings/map_frame_id", pose_frame, "/odom");
    nh.param<double>("/settings/publish_rate", publish_rate_, 1000); 
    nh.param<int>("/settings/pose_skip", pose_skip_, 0); 
    first_pose_ = {0, 0, 0};
    // first_pose_ = pose_from_csv_.front();
    // std::cout<<"first pose is: "<<first_pose_[0]<<", "<<first_pose_[1]<<", "<<first_pose_[2]<<std::endl;
    pose_frame_ = pose_frame;
    
    pose_pub_ = n_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    slip_pub_ = n_->advertise<geometry_msgs::Vector3Stamped>(slip_topic, 1000);
    slip_flag_pub_ = n_->advertise<geometry_msgs::Vector3Stamped>(slip_flag_topic, 1000);
    vel_pub_ = n_->advertise<geometry_msgs::Vector3Stamped>(vel_topic, 1000);
    // this->pose_publishing_thread_ = std::thread([this]{this->posePublishingThread();});
}

PosePublisherNode::~PosePublisherNode() {}

// Publishes pose
void PosePublisherNode::posePublish(const husky_inekf::HuskyState& state_) {
    // std::array<float,3> cur_pose = pose_from_csv_.front();
    // pose_from_csv_.pop();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.seq = seq_;
    pose_msg.header.stamp = ros::Time::now();

    ros::Time stamp = ros::Time::now();


    pose_msg.header.frame_id = pose_frame_;
    pose_msg.pose.pose.position.x = state_.x() - first_pose_[0];
    pose_msg.pose.pose.position.y = state_.y() - first_pose_[1];
    pose_msg.pose.pose.position.z = state_.z() - first_pose_[2];
    pose_msg.pose.pose.orientation.w = state_.getQuaternion().w();
    pose_msg.pose.pose.orientation.x = state_.getQuaternion().x();
    pose_msg.pose.pose.orientation.y = state_.getQuaternion().y();  
    pose_msg.pose.pose.orientation.z = state_.getQuaternion().z();
    // std::cout<<"publishing: "<<pose_msg.pose.pose.position.x<<", "<<pose_msg.pose.pose.position.y<<", "<<pose_msg.pose.pose.position.z<<std::endl;
    pose_pub_.publish(pose_msg);
    seq_++;
}

void PosePublisherNode::velPublish(const husky_inekf::HuskyState& state_) {
    // std::array<float,3> cur_pose = pose_from_csv_.front();
    // pose_from_csv_.pop();

    geometry_msgs::Vector3Stamped vel_msg;
    // vel_msg.header.seq = seq_;
    vel_msg.header.stamp = ros::Time::now();
    vel_msg.header.frame_id = pose_frame_;
    auto vb = state_.getBodyVelocity();
    vel_msg.vector.x = vb(0);
    vel_msg.vector.y = vb(1);
    vel_msg.vector.z = vb(2);
    // std::cout<<"publishing: "<<pose_msg.pose.pose.position.x<<", "<<pose_msg.pose.pose.position.y<<", "<<pose_msg.pose.pose.position.z<<std::endl;
    vel_pub_.publish(vel_msg);
    // seq_++;
}

void PosePublisherNode::slipPublish(const husky_inekf::HuskyState& state_) {
    // std::array<float,3> cur_pose = pose_from_csv_.front();
    // pose_from_csv_.pop();

    geometry_msgs::Vector3Stamped slip_msg;
    // slip_msg.header.seq = seq_;
    slip_msg.header.stamp = ros::Time::now();
    slip_msg.header.frame_id = pose_frame_;
    auto dist = state_.getDisturbance();
    slip_msg.vector.x = dist(0);
    slip_msg.vector.y = dist(1);
    slip_msg.vector.z = dist(2);
    // std::cout<<"publishing: "<<pose_msg.pose.pose.position.x<<", "<<pose_msg.pose.pose.position.y<<", "<<pose_msg.pose.pose.position.z<<std::endl;
    slip_pub_.publish(slip_msg);
    // seq_++;
}

void PosePublisherNode::slipFlagPublish(const husky_inekf::HuskyState& state_) {
    // std::array<float,3> cur_pose = pose_from_csv_.front();
    // pose_from_csv_.pop();
        
    geometry_msgs::Vector3Stamped slip_flag_msg;
    // slip_msg.header.seq = seq_;
    slip_flag_msg.header.stamp = ros::Time::now();
    slip_flag_msg.header.frame_id = pose_frame_;
    int slip_flag = state_.slip_flag;
    slip_flag_msg.vector.x = (double)slip_flag;
    slip_flag_msg.vector.y = (double)slip_flag;
    slip_flag_msg.vector.z = (double)slip_flag;
    // std::cout<<"publishing: "<<pose_msg.pose.pose.position.x<<", "<<pose_msg.pose.pose.position.y<<", "<<pose_msg.pose.pose.position.z<<std::endl;
    slip_flag_pub_.publish(slip_flag_msg);
    // seq_++;
}


// Pose message callback
// void PosePublisherNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
//     if ((int)msg->header.seq%pose_skip_!=0) { return; }
//     geometry_msgs::PoseStamped pose;
//     pose.header = msg->header;
//     pose.pose = msg->pose.pose;
//     std::lock_guard<std::mutex> lock(poses_mutex_);
//     poses_.push_back(pose);
// }


// Path publishing thread
// void PosePublisherNode::posePublishingThread(){
//     // Loop and publish data
//     ros::Rate loop_rate(publish_rate_);
//     while(ros::ok()){
//         posePublish();
//         loop_rate.sleep();
//     }
// }

// int main(int argc, char **argv) {
//     // Initialize ROS node
//     ros::init(argc, argv, "pose_publisher");  
//     ros::NodeHandle n;
//     PosePublisherNode pose_publisher_node(n);
//     ros::spin();
//     return 0;
// }
