#include "pathTracker.h"

using namespace std;

RobotState::RobotState(double x, double y, double theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos)
{
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}

Eigen::Vector3d RobotState::getVector()
{
    Eigen::Vector3d vec;
    vec << x_, y_, theta_;
    return vec;
}

pathTracker::pathTracker(ros::NodeHandle& nh)
{
    nh_ = nh;
    initialize();
}

void pathTracker::timerCallback(const ros::TimerEvent& e)
{
    switch (workingMode_)
    {
        case Mode::GLOBALPATH_RECEIVED: {
            if (workingMode_past_ == Mode::IDLE)
            {
                workingMode_past_ = workingMode_;
                workingMode_ = Mode::TRACKING;
                break;
            }
            else if (workingMode_past_ == Mode::TRACKING)
            {
                // Slow down first then start tracking new path
                workingMode_past_ = workingMode_;
                workingMode_ = Mode::SLOW_DOWN;
                break;
            }
        }
        break;

        case Mode::TRACKING: {
            if (xy_goal_reached(cur_pose_, goal_pose_) && theta_goal_reached(cur_pose_, goal_pose_))
            {
                workingMode_past_ = workingMode_;
                workingMode_ = Mode::IDLE;

                velocity_state_.x_ = 0;
                velocity_state_.y_ = 0;
                velocity_state_.theta_ = 0;
                velocityPublish();
                break;
            }

            if (robot_type_ == RobotType::OmniDrive)
            {
                RobotState local_goal;
                local_goal = pathTracker::rollingWindow(cur_pose_, global_path_, lookahead_d_);
                pathTracker::omniController(local_goal, cur_pose_);
            }
            else if (robot_type_ == RobotType::DiffDrive)
            {
                RobotState local_goal;
                local_goal = pathTracker::rollingWindow(cur_pose_, global_path_, lookahead_d_);
                pathTracker::diffController(local_goal, cur_pose_);
            }
        }
        break;

        case Mode::IDLE: {
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
            velocity_state_.theta_ = 0;
            velocityPublish();
        }
        break;

        case Mode::SLOW_DOWN: {
        }
        break;
    }
}

void pathTracker::initialize()
{
    // load parameter
    robot_type_ = RobotType::OmniDrive;
    nh_.param<double>("/pathTracker/control_frequency", control_frequency_, 200);
    nh_.param<double>("/pathTracker/lookahead_distance", lookahead_d_, 0.2);

    nh_.param<double>("/pathTracker/linear_kp", linear_kp_, 0.8);
    nh_.param<double>("/pathTracker/linear_max_velocity", linear_max_vel_, 0.5);
    nh_.param<double>("/pathTracker/linear_acceleration", linear_acceleration_, 0.3);
    nh_.param<double>("/pathTracker/linear_brake_distance", linear_brake_distance_, 0.3);
    nh_.param<double>("/pathTracker/xy_tolerance", xy_tolerance_, 0.02);

    nh_.param<double>("/pathTracker/angular_kp", angular_kp_, 0.9);
    nh_.param<double>("/pathTracker/angular_max_velocity", angular_max_vel_, 1);
    nh_.param<double>("/pathTracker/angular_acceleration", angular_acceleration_, 0.5);
    nh_.param<double>("/pathTracker/angular_brake_distance", angular_brake_distance_, 0.35);
    nh_.param<double>("/pathTracker/theta_tolerance", theta_tolerance_, 0.03);

    poseSub_ = nh_.subscribe("/ekf_pose", 50, &pathTracker::poseCallback, this);
    goalSub_ = nh_.subscribe("/nav_goal", 50, &pathTracker::goalCallback, this);
    velPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    localgoalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
    posearrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("/orientation", 10);

    if_localgoal_final_reached = false;

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &pathTracker::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1 / control_frequency_), false);
    timer_.start();
    workingMode_ = Mode::IDLE;
    workingMode_past_ = Mode::IDLE;

    ROS_INFO("Initialized !");
}

void pathTracker::plannerClient(RobotState cur_pos, RobotState goal_pos)
{
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

    if (client.call(srv))
    {
        ROS_INFO("Path received from global planner !");
        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;
        global_path_.clear();

        for (const auto& point : path_msg.poses)
        {
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
        workingMode_ = Mode::GLOBALPATH_RECEIVED;

        ROS_INFO("Path received from global planner !");

        // print global path
        ROS_INFO("--- global path ---");
        for (const auto& point : global_path_)
        {
            ROS_INFO("(%f, %f, %f)", point.x_, point.y_, point.theta_);
        }
        ROS_INFO("--- ---");
    }
    else
    {
        ROS_ERROR("Failed to call service make_plan");
        // return 1;
    }
}

void pathTracker::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    cur_pose_.x_ = pose_msg->pose.pose.position.x;
    cur_pose_.y_ = pose_msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    cur_pose_.theta_ = yaw;
}

void pathTracker::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
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

RobotState pathTracker::rollingWindow(RobotState cur_pos, std::vector<RobotState> path, double L_d)
{
    int k = 1;
    int last_k = 0;
    int d_k = 0;

    RobotState a;
    int a_idx = 0;

    RobotState b;
    int b_idx = 0;

    RobotState local_goal;
    bool if_b_asigned = false;
    double r = L_d;

    for (int i = 0; i < path.size(); i++)
    {
        last_k = k;
        if (cur_pos.distanceTo(path.at(i)) > r)
            k = 1;
        else
            k = 0;

        d_k = k - last_k;

        if (d_k == 1)
        {
            b = path.at(i);
            if_b_asigned = true;
            b_idx = i;
            a_idx = i - 1;
            break;
        }
    }

    if (!if_b_asigned)
    {
        double min = 1000000000;
        for (int i = 0; i < path.size(); i++)
        {
            if (cur_pos.distanceTo(path.at(i)) < min)
            {
                min = cur_pos.distanceTo(path.at(i));
                b_idx = i;
                a_idx = i - 1;
                b = path.at(i);
            }
        }
    }

    if (a_idx == -1)
    {
        local_goal = path.at(b_idx);
    }
    else
    {
        cout << " aaaaaaa " << endl;
        cout << "a_idx" << a_idx << endl;
        cout << "b_idx" << b_idx << endl;
        a = path.at(a_idx);
        double d_ca = cur_pos.distanceTo(a);
        double d_cb = cur_pos.distanceTo(b);
        local_goal.x_ = a.x_ + (b.x_ - a.x_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.y_ = a.y_ + (b.y_ - a.y_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.theta_ = a.theta_;
    }

    if (if_localgoal_final_reached)
    {
        cout << " bbbbbbbbb " << endl;
        local_goal = path.back();
    }

    if (cur_pos.distanceTo(path.back()) < lookahead_d_ - 0.2)
    {
        cout << " ccccccccc " << endl;
        local_goal = path.back();
    }

    if (local_goal.distanceTo(path.back()) < lookahead_d_ - 0.2)
    {
        cout << " ddddddddd " << endl;
        local_goal = path.back();
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

std::vector<RobotState> pathTracker::orientationFilter(std::vector<RobotState> origin_path)
{
    std::vector<RobotState> path;
    double init_theta = cur_pose_.theta_;
    double goal_theta = goal_pose_.theta_;
    double theta_err = 0;
    double d_theta = 0;
    Eigen::Vector3d init;
    Eigen::Vector3d goal;
    // calculate rotate direction
    init << cos(init_theta), sin(init_theta), 0;
    goal << cos(goal_theta), sin(goal_theta), 0;

    if (init.cross(goal)(2) >= 0)
        rotate_direction_ = 1;
    else
        rotate_direction_ = -1;

    // theta_err = acos(init(0)*goal(0)+init(1)*goal(1));
    theta_err = fabs(angleLimitChecking(goal_theta - init_theta));
    d_theta = rotate_direction_ * theta_err / (origin_path.size() - 1);

    RobotState point(origin_path.at(0).x_, origin_path.at(0).y_, init_theta);
    path.push_back(point);

    for (int i = 0; i < origin_path.size(); i++)
    {
        if (i != 0)
        {
            double theta;
            theta = angleLimitChecking(path.at(i - 1).theta_ + d_theta);
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

    for (int i = 0; i < path.size(); i++)
    {
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

double pathTracker::angleLimitChecking(double theta)
{
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    return theta;
}

// Path tracker for differential drive robot
void pathTracker::diffController(RobotState local_goal, RobotState cur_pos)
{
}

// Path tracker for omni drive robot
void pathTracker::omniController(RobotState local_goal, RobotState cur_pos)
{
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    int rotate_direction = 0;
    Eigen::Vector3d goal_vec(goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);
    Eigen::Vector3d cur_vec(cur_pos.x_, cur_pos.y_, cur_pos.theta_);

    if (cur_vec.cross(goal_vec)(2) >= 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;

    // transform local_goal to base_footprint frame
    Eigen::Vector2d goal_base_vec;
    Eigen::Vector2d localgoal_bf;
    Eigen::Matrix2d rot;
    goal_base_vec << (local_goal.x_ - cur_pos.x_), (local_goal.y_ - cur_pos.y_);
    rot << cos(-cur_pos.theta_), -sin(-cur_pos.theta_), sin(-cur_pos.theta_), cos(-cur_pos.theta_);
    localgoal_bf = rot * goal_base_vec;

    if (xy_goal_reached(cur_pose_, goal_pose_))
    {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
    }
    else
    {
        linear_velocity =
            velocityProfile(Velocity::linear, cur_pose_, goal_pose_, velocity_state_, linear_acceleration_);
        double direction = atan2(localgoal_bf(1), localgoal_bf(0));
        velocity_state_.x_ = linear_velocity * cos(direction);
        velocity_state_.y_ = linear_velocity * sin(direction);
    }

    if (theta_goal_reached(cur_pose_, goal_pose_))
    {
        velocity_state_.theta_ = 0;
    }
    else
    {
        angular_velocity = velocityProfile(Velocity::angular, cur_pose_, local_goal, velocity_state_,
                                           rotate_direction_ * angular_acceleration_);
        velocity_state_.theta_ = angular_velocity;
    }
    velocityPublish();
}

double pathTracker::velocityProfile(Velocity vel_type, RobotState cur_pos, RobotState goal_pos,
                                    RobotState vel_state_past, double acceleration)
{
    double output_vel = 0;

    if (vel_type == Velocity::linear)
    {
        double d_vel = acceleration / control_frequency_;
        RobotState _(0, 0, 0);
        double last_vel = vel_state_past.distanceTo(_);
        output_vel = last_vel + d_vel;

        if (cur_pos.distanceTo(goal_pos) < linear_brake_distance_)
            output_vel = cur_pos.distanceTo(goal_pos) * linear_kp_;

        if (output_vel > linear_max_vel_)
            output_vel = linear_max_vel_;
    }

    if (vel_type == Velocity::angular)
    {
        double theta_err;
        theta_err = fabs(angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));

        output_vel = theta_err * angular_kp_;
        if (signbit(acceleration))
        {
            output_vel *= -1;
        }
    }
    return output_vel;
}

bool pathTracker::xy_goal_reached(RobotState cur_pos, RobotState goal_pos)
{
    if (cur_pos.distanceTo(goal_pos) < xy_tolerance_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool pathTracker::theta_goal_reached(RobotState cur_pos, RobotState goal_pos)
{
    double theta_err = 0;
    Eigen::Vector2d cur_vec;
    Eigen::Vector2d goal_vec;
    cur_vec << cos(cur_pos.theta_), sin(cur_pos.theta_);
    goal_vec << cos(goal_pos.theta_), sin(goal_pos.theta_);
    theta_err = cur_vec.dot(goal_vec);

    theta_err = fabs(angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));
    if (fabs(theta_err) < theta_tolerance_)
    {
        return true;
    }
    else
        return false;
}

void pathTracker::velocityPublish()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = velocity_state_.x_;
    vel_msg.linear.y = velocity_state_.y_;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = velocity_state_.theta_;
    velPub_.publish(vel_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathTracker");
    ros::NodeHandle nh;
    pathTracker pathTracker_inst(nh);

    while (ros::ok())
    {
        ros::spin();
    }
}