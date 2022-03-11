#include "holonomic_controller.h"

using namespace std;

RobotState::RobotState(double x, double y, double theta){
    x_ = x;
    y_ = y;
    theta_ = theta;
}

Eigen::Vector3d RobotState::getVector(){
    Eigen::Vector3d vec;
    vec << x_, y_, theta_;
    return vec;
}

void PurePursuit::initialize(){
    // load parameter
    nh_.param<double>("/holonomic_controller/control_frequency", control_frequency_, 100);
    nh_.param<double>("/holonomic_controller/lookahead_distance", lookahead_d_, 0.4);

    nh_.param<double>("/holonomic_controller/linear_kp", linear_kp_, 0.8);
    nh_.param<double>("/holonomic_controller/linear_max_velocity", linear_max_vel_, 0.5);
    nh_.param<double>("/holonomic_controller/linear_acceleration", linear_acceleration_, 0.3);
    nh_.param<double>("/holonomic_controller/linear_brake_distance", linear_brake_distance_, 0.3);
    nh_.param<double>("/holonomic_controller/xy_tolerance", xy_tolerance_, 0.02);

    nh_.param<double>("/holonomic_controller/angular_kp", angular_kp_, 0.5);
    nh_.param<double>("/holonomic_controller/angular_max_velocity", angular_max_vel_, 1);
    nh_.param<double>("/holonomic_controller/angular_acceleration", angular_acceleration_, 0.5);
    nh_.param<double>("/holonomic_controller/angular_brake_distance", angular_brake_distance_, 0.35);
    nh_.param<double>("/holonomic_controller/theta_tolerance", theta_tolerance_, 0.03);
    
    poseSub_ = nh_.subscribe("/ekf_pose", 50, &PurePursuit::poseCallback, this);
    goalSub_ = nh_.subscribe("/nav_goal", 50, &PurePursuit::goalCallback, this);
    velPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void PurePursuit::plannerClient(RobotState cur_pos, RobotState goal_pos){
    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = "map";
    cur.pose.position.x = cur_pos.x_;
    cur.pose.position.y = cur_pos.y_;
    cur.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, cur_pos.theta_);
    cur.pose.orientation.x = q.x();
    cur.pose.orientation.y = q.y();
    cur.pose.orientation.z = q.z();
    cur.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_pos.x_;
    goal.pose.position.y = goal_pos.y_;
    goal.pose.position.z = 0;

    // tf2::Quaternion q;
    q.setRPY(0, 0, goal_pos.theta_);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    std::vector<geometry_msgs::PoseStamped> path_msg;

    if (client.call(srv)){
        ROS_INFO("Path Acheived from global planner !");
        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;
        global_path_.clear();
        
        for(const auto & point: path_msg.poses){
            RobotState pose;
            pose.x_ = point.pose.position.x;
            pose.y_ = point.pose.position.y;
            tf2::Quaternion q;
            tf2::fromMsg(point.pose.orientation, q);
            tf2::Matrix3x3 qt(q);
            double _, yaw;
            qt.getRPY(_, _, yaw);
            pose.theta_ = yaw;
            global_path_.push_back(pose);
        }
        
        // print global path
        // for(const auto & point: global_path_){
        //     ROS_INFO("(%f, %f, %f)",point.x_, point.y_, point.theta_);
        // }
    }
    else{
        ROS_ERROR("Failed to call service make_plan");
        // return 1;
    }
}

void PurePursuit::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
    cur_pose_.x_ = pose_msg->pose.pose.position.x;
    cur_pose_.y_ = pose_msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    cur_pose_.theta_ = yaw;
}

void PurePursuit::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
    goal_pose_.x_ = pose_msg->pose.position.x;
    goal_pose_.y_ = pose_msg->pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    goal_pose_.theta_ = yaw;
    ROS_INFO("Goal received ! (%f, %f, %f)", goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);
    plannerClient(cur_pose_, goal_pose_);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "holonomic_controller");
    ros::NodeHandle nh;
    PurePursuit purepursuit(nh);
    purepursuit.initialize();

    while (ros::ok()){
        ros::spin();
    }
}
