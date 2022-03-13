#include "pathTracker.h"

using namespace std;

RobotState::RobotState(double x, double y, double theta){
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos){
    return sqrt(pow(x_-pos.x_, 2) + pow(y_-pos.y_, 2));
}

Eigen::Vector3d RobotState::getVector(){
    Eigen::Vector3d vec;
    vec << x_, y_, theta_;
    return vec;
}

pathTracker::pathTracker(ros::NodeHandle& nh){
    nh_ = nh;
    initialize();
}

void pathTracker::timerCallback(const ros::TimerEvent& e){
    if (if_globalpath_received){
        rollingWindow(cur_pose_, global_path_, lookahead_d_);
    }
}

void pathTracker::initialize(){
    // load parameter
    nh_.param<double>("/pathTracker/control_frequency", control_frequency_, 200);
    nh_.param<double>("/pathTracker/lookahead_distance", lookahead_d_, 0.4);

    nh_.param<double>("/pathTracker/linear_kp", linear_kp_, 0.8);
    nh_.param<double>("/pathTracker/linear_max_velocity", linear_max_vel_, 0.5);
    nh_.param<double>("/pathTracker/linear_acceleration", linear_acceleration_, 0.3);
    nh_.param<double>("/pathTracker/linear_brake_distance", linear_brake_distance_, 0.3);
    nh_.param<double>("/pathTracker/xy_tolerance", xy_tolerance_, 0.02);

    nh_.param<double>("/pathTracker/angular_kp", angular_kp_, 0.5);
    nh_.param<double>("/pathTracker/angular_max_velocity", angular_max_vel_, 1);
    nh_.param<double>("/pathTracker/angular_acceleration", angular_acceleration_, 0.5);
    nh_.param<double>("/pathTracker/angular_brake_distance", angular_brake_distance_, 0.35);
    nh_.param<double>("/pathTracker/theta_tolerance", theta_tolerance_, 0.03);
    
    poseSub_ = nh_.subscribe("/ekf_pose", 50, &pathTracker::poseCallback, this);
    goalSub_ = nh_.subscribe("/nav_goal", 50, &pathTracker::goalCallback, this);
    velPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    localgoalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
    posearrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("/orientation", 10);

    if_globalpath_received = false;
    if_localgoal_final_reached = false;

    timer_ = nh_.createTimer(ros::Duration(1.0/control_frequency_), &pathTracker::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1 / control_frequency_), false);
    timer_.start();
    ROS_INFO("Initialized !");
}

void pathTracker::plannerClient(RobotState cur_pos, RobotState goal_pos){
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
        ROS_INFO("Path received from global planner !");
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
        global_path_ = orientationFilter(global_path_);
        if_globalpath_received = true;
        
        // print global path
        for(const auto & point: global_path_){
            ROS_INFO("(%f, %f, %f)",point.x_, point.y_, point.theta_);
        }
    }
    else{
        ROS_ERROR("Failed to call service make_plan");
        // return 1;
    }
}

void pathTracker::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
    cur_pose_.x_ = pose_msg->pose.pose.position.x;
    cur_pose_.y_ = pose_msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    cur_pose_.theta_ = yaw;
}

void pathTracker::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
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

RobotState pathTracker::rollingWindow(RobotState cur_pos, std::vector<RobotState> path, double L_d){
    int k = 1;
    int last_k = 0;
    int d_k = 0;
    RobotState a;
    RobotState b;
    RobotState local_goal;
    int b_idx = 0;
    int a_idx = 0;
    bool if_b_asigned = false;
    double r = L_d;

    for (int i = 0; i < path.size(); i++){
        last_k = k;
        if (cur_pos.distanceTo(path.at(i)) > r)
            k = 1;
        else
            k = 0;
            
        d_k = k - last_k;

        if (d_k == 1){
            b = path.at(i);
            if_b_asigned = true;
            b_idx = i;
            a_idx = i-1;
            break;
        }
    }
    if (!if_b_asigned and !if_localgoal_final_reached){
        double min = 1000000000;
        for (int i = 0; i < path.size(); i++){
            if (cur_pos.distanceTo(path.at(i)) < min){
                min = cur_pos.distanceTo(path.at(i));
                b_idx = i;
                a_idx = i-1;
                b = path.at(i);
            }
        }
    }
    else if (if_localgoal_final_reached){
        local_goal = path.back();
    }
    else {
        if (a_idx = -1){
            local_goal = path.at(b_idx);
        }
        else{
            a = path.at(a_idx);
            double d_ca = cur_pos.distanceTo(a);
            double d_cb = cur_pos.distanceTo(b);
            local_goal.x_ = a.x_ + (b.x_-a.x_) * (r - d_ca)/(d_cb-d_ca);
            local_goal.y_ = a.y_ + (b.y_-a.y_) * (r - d_ca)/(d_cb-d_ca);
            local_goal.theta_ = a.theta_;
        }
    }

    if (local_goal.distanceTo(path.back()) < 0.05){
        if_localgoal_final_reached = true;
    }

    // for rviz visualization
    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header.frame_id = "map";
    pos_msg.header.stamp = ros::Time::now();
    pos_msg.pose.position.x = local_goal.x_;
    pos_msg.pose.position.y = local_goal.y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, local_goal.theta_);
    pos_msg.pose.orientation.x = q.x();
    pos_msg.pose.orientation.y = q.y();
    pos_msg.pose.orientation.z = q.z();
    pos_msg.pose.orientation.w = q.w();
    localgoalPub_.publish(pos_msg);
    return local_goal;
}

std::vector<RobotState> pathTracker::orientationFilter(std::vector<RobotState> origin_path){
    std::vector<RobotState> path;
    double init_theta = cur_pose_.theta_;
    double goal_theta = goal_pose_.theta_;
    double theta_err = 0;
    double d_theta = 0;
    int rotate_direction = 0;
    Eigen::Vector3d init;
    Eigen::Vector3d goal;
    // calculate rotate direction
    init << cos(init_theta), sin(init_theta), 0;
    goal << cos(goal_theta), sin(goal_theta), 0;

    if (init.cross(goal)(2) >= 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;

    // theta_err = acos(init(0)*goal(0)+init(1)*goal(1));
    theta_err = fabs(angleLimitChecking(goal_theta - init_theta));
    d_theta = rotate_direction * theta_err / (origin_path.size()-1);

    RobotState point(origin_path.at(0).x_, origin_path.at(0).y_, init_theta);
    path.push_back(point);
    
    for (int i = 0; i < origin_path.size(); i ++){
        if (i != 0){
            double theta;
            theta = angleLimitChecking(path.at(i-1).theta_ + d_theta);
            cout << "theta = " << theta << endl;
            RobotState point(origin_path.at(i).x_, origin_path.at(i).y_, theta);
            path.push_back(point);
        }
    }

    // Rviz visualize processed path
    geometry_msgs::PoseArray arr_msg;
    arr_msg.header.frame_id = "map";
    arr_msg.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Pose> poses;
    
    for (int i = 0; i < path.size(); i ++){
        geometry_msgs::Pose pose;
        pose.position.x = path.at(i).x_;        
        pose.position.y = path.at(i).y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, path.at(i).theta_);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        poses.push_back(pose);
    }
    arr_msg.poses = poses;
    posearrayPub_.publish(arr_msg);

    return path;
}

double pathTracker::angleLimitChecking(double theta){
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    return theta;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pathTracker");
    ros::NodeHandle nh;
    pathTracker pathTracker_inst(nh);

    while (ros::ok()){
        ros::spin();
    }
}
