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

rollingWindow::rollingWindow(ros::NodeHandle& nh, std::vector<RobotState> path, double R, std::string topic)
{
    nh_ = nh;
    window_radius_ = R;
    rviz_topic = topic;

    if_pathfinal_reached = false;
    posePub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic, 10);
}

RobotState rollingWindow::findLocalgoal(RobotState cur_pos)
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
    double r = window_radius_;

    for (int i = 0; i < rolling_path_.size(); i++)
    {
        if (i == 1)
            last_k = 0;
        last_k = k;
        if (cur_pos.distanceTo(rolling_path_.at(i)) >= r)
            k = 1;
        else
            k = 0;

        d_k = k - last_k;

        if (d_k == 1)
        {
            b = rolling_path_.at(i);
            if_b_asigned = true;
            b_idx = i;
            a_idx = i - 1;
            break;
        }
    }

    if (!if_b_asigned)
    {
        double min = 1000000;
        for (int i = 0; i < rolling_path_.size(); i++)
        {
            if (cur_pos.distanceTo(rolling_path_.at(i)) < min)
            {
                min = cur_pos.distanceTo(rolling_path_.at(i));
                b_idx = i;
                a_idx = i - 1;
                b = rolling_path_.at(i);
            }
        }
    }

    if (a_idx == -1)
    {
        local_goal = rolling_path_.at(b_idx);
    }
    else
    {
        a = rolling_path_.at(a_idx);
        double d_ca = cur_pos.distanceTo(a);
        double d_cb = cur_pos.distanceTo(b);
        local_goal.x_ = a.x_ + (b.x_ - a.x_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.y_ = a.y_ + (b.y_ - a.y_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.theta_ = a.theta_;
    }

    if (if_pathfinal_reached)
    {
        // cout << "local goal set to path.back()" << endl;
        local_goal = rolling_path_.back();
    }

    if (cur_pos.distanceTo(rolling_path_.back()) < r + 0.01)
        local_goal = rolling_path_.back();

    if (local_goal.distanceTo(rolling_path_.back()) < 0.005)
    {
        local_goal = rolling_path_.back();
        if_pathfinal_reached = true;
    }

    // for rviz visualization
    // geometry_msgs::PoseStamped pos_msg;
    // pos_msg.header.frame_id = "map";
    // pos_msg.header.stamp = ros::Time::now();
    // pos_msg.pose.position.x = local_goal.x_;
    // pos_msg.pose.position.y = local_goal.y_;
    // tf2::Quaternion q;
    // q.setRPY(0, 0, local_goal.theta_);
    // pos_msg.pose.orientation.x = q.x();
    // pos_msg.pose.orientation.y = q.y();
    // pos_msg.pose.orientation.z = q.z();
    // pos_msg.pose.orientation.w = q.w();
    // posePub_.publish(pos_msg);

    return local_goal;
}

void rollingWindow::updatePath(std::vector<RobotState> update_path)
{
    rolling_path_ = update_path;
    if_pathfinal_reached = false;
}

pathTracker::pathTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &pathTracker::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

pathTracker::~pathTracker()
{
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("lookahead_distance");
    nh_local_.deleteParam("linear_kp");
    nh_local_.deleteParam("linear_max_velocity");
    nh_local_.deleteParam("linear_acceleration");
    nh_local_.deleteParam("linear_brake_distance");
    nh_local_.deleteParam("xy_tolerance");
    nh_local_.deleteParam("linear_transition_vel_");
    nh_local_.deleteParam("linear_transition_acc_");
    nh_local_.deleteParam("linear_acceleration_profile");
    nh_local_.deleteParam("linear_deceleration_profile");

    nh_local_.deleteParam("angular_kp");
    nh_local_.deleteParam("angular_max_velocity");
    nh_local_.deleteParam("angular_acceleration");
    nh_local_.deleteParam("angular_brake_distance");
    nh_local_.deleteParam("theta_tolerance");
    nh_local_.deleteParam("angular_transition_vel_");
    nh_local_.deleteParam("angular_transition_acc_");
    nh_local_.deleteParam("angular_acceleration_profile");
    nh_local_.deleteParam("angular_deceleration_profile");
}

void pathTracker::initialize()
{
    // setup rolling window instance
    apf_global_path_rw.nh_ = nh_;
    apf_global_path_rw.rviz_topic = "APF_local_goal";
    apf_global_path_rw.window_radius_ = far_lookahead_d_;

    local_path_rw.nh_ = nh_;
    local_path_rw.rviz_topic = "near_obs_local_goal";
    local_path_rw.window_radius_ = lookahead_d_;

    global_path_rw.nh_ = nh_;
    global_path_rw.rviz_topic = "near_local_goal";
    global_path_rw.window_radius_ = lookahead_d_;

    // if_globalpath_rw_finished = false;
    // if_localpath_rw_finished = false;

    if_globalpath_switched = false;
    if_obstacle_approached = false;

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &pathTracker::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();
    workingMode_ = Mode::IDLE;
    workingMode_past_ = Mode::IDLE;

    ROS_INFO("Initialized !");
}

bool pathTracker::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    // load parameter
    bool get_param_ok = true;
    bool prev_active = p_active_;

    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    get_param_ok = nh_local_.param<string>("robot_type", robot_type_, "omni");
    get_param_ok = nh_local_.param<double>("control_frequency", control_frequency_, 50);
    get_param_ok = nh_local_.param<double>("lookahead_distance", lookahead_d_, 0.2);
    get_param_ok = nh_local_.param<double>("local_planner_lookahead_distance", far_lookahead_d_, 0.8);

    // linear parameter
    // acceleration
    get_param_ok = nh_local_.param<double>("linear_max_velocity", linear_max_vel_, 0.5);
    get_param_ok = nh_local_.param<double>("linear_acceleration", linear_acceleration_, 0.3);
    get_param_ok = nh_local_.param<string>("linear_acceleration_profile", linear_acceleration_profile_, "linear");
    // transition
    get_param_ok = nh_local_.param<double>("linear_transition_velocity", linear_transition_vel_, 0.15);
    get_param_ok = nh_local_.param<double>("linear_transition_acceleration", linear_transition_acc_, 0.6);
    // deceleration
    get_param_ok = nh_local_.param<double>("linear_kp", linear_kp_, 0.8);
    get_param_ok = nh_local_.param<double>("linear_brake_distance_ratio", linear_brake_distance_ratio_, 0.3);
    get_param_ok = nh_local_.param<string>("linear_deceleration_profile", linear_deceleration_profile_, "linear");

    // angular parameter
    get_param_ok = nh_local_.param<double>("angular_max_velocity", angular_max_vel_, 3);
    get_param_ok = nh_local_.param<double>("angular_acceleration", angular_acceleration_, 0.5);
    get_param_ok = nh_local_.param<double>("angular_brake_distance", angular_brake_distance_, 0.35);
    get_param_ok = nh_local_.param<double>("angular_transition_velocity", angular_transition_vel_, 0.15);
    get_param_ok = nh_local_.param<double>("angular_transition_acceleration", angular_transition_acc_, 0.6);
    get_param_ok = nh_local_.param<double>("angular_kp", angular_kp_, 1.5);
    get_param_ok = nh_local_.param<string>("angular_acceleration_profile", angular_acceleration_profile_, "linear");
    get_param_ok = nh_local_.param<string>("angular_deceleration_profile", angular_deceleration_profile_, "linear");

    get_param_ok = nh_local_.param<double>("xy_tolerance", xy_tolerance_, 0.01);
    get_param_ok = nh_local_.param<double>("theta_tolerance", theta_tolerance_, 0.03);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            poseSub_ = nh_.subscribe("/ekf_pose", 50, &pathTracker::poseCallback, this);
            goalSub_ = nh_.subscribe("/nav_goal", 50, &pathTracker::goalCallback, this);
            obsSub_ = nh_.subscribe("/have_obstacles", 50, &pathTracker::obsCallback, this);
            velPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            localgoalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 10);
            posearrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("/orientation", 10);
        }
        else
        {
            poseSub_.shutdown();
            goalSub_.shutdown();
            obsSub_.shutdown();
            velPub_.shutdown();
            localgoalPub_.shutdown();
            posearrayPub_.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Path Tracker]: "
                        << "set param ok");
    }
    else
    {
        ROS_WARN_STREAM("[Path Tracker]: "
                        << "set param failed");
    }
    cout << "param updated !" << endl;
    return true;
}

void pathTracker::timerCallback(const ros::TimerEvent& e)
{
    switch (workingMode_)
    {
        case Mode::GLOBALPATH_RECEIVED: {
            if (workingMode_past_ == Mode::IDLE)
            {
                switchMode(Mode::TRACKING);
                break;
            }
            else if (workingMode_past_ == Mode::TRACKING)
            {
                // Slow down first then start tracking new path
                switchMode(Mode::TRANSITION);
                break;
            }
            else if (workingMode_past_ == Mode::TRANSITION)
            {
                switchMode(Mode::TRACKING);
                break;
            }
        }
        break;

        case Mode::TRACKING: {
            if (xy_goal_reached(cur_pose_, goal_pose_) && theta_goal_reached(cur_pose_, goal_pose_))
            {
                ROS_INFO("Working Mode : GOAL REACHED !");
                switchMode(Mode::IDLE);
                velocity_state_.x_ = 0;
                velocity_state_.y_ = 0;
                velocity_state_.theta_ = 0;
                velocityPublish();
                break;
            }

            if (workingMode_past_ == Mode::TRANSITION)
            {
                if (if_globalpath_switched == false)
                {
                    plannerClient(cur_pose_, goal_pose_);
                    global_path_rw.updatePath(global_path_);

                    linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
                    if_globalpath_switched = true;
                }
            }

            // ROS_INFO("Working Mode : TRACKING");
            if (robot_type_ == "omni")
            {
                RobotState local_goal;
                RobotState far_local_goal;
                std::vector<RobotState> tracking_path;
                // cout << "if_obstacle_approached = " << if_obstacle_approached << endl;
                if (if_obstacle_approached == true)
                {
                    far_local_goal = apf_global_path_rw.findLocalgoal(cur_pose_);
                    localPlannerClient(cur_pose_, far_local_goal);
                    local_goal = local_path_rw.findLocalgoal(cur_pose_);
                    omniController(local_goal, cur_pose_);

                    if_obstacle_approached = false;
                }
                else
                {
                    local_goal = global_path_rw.findLocalgoal(cur_pose_);
                    omniController(local_goal, cur_pose_);
                }
            }
            else if (robot_type_ == "diff")
            {
            }
        }
        break;

        case Mode::IDLE: {
            // ROS_INFO("Working Mode : IDLE");
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
            velocity_state_.theta_ = 0;
            velocityPublish();
        }
        break;

        case Mode::TRANSITION: {
            // ROS_INFO("Working Mode : TRANSITION");
            double linear_vel = sqrt(pow(velocity_state_.x_, 2) + pow(velocity_state_.y_, 2));
            double angular_vel = velocity_state_.theta_;

            if (linear_vel <= linear_transition_vel_ && angular_vel <= angular_transition_vel_)
            {
                switchMode(Mode::TRACKING);
                break;
            }

            if (robot_type_ == "omni")
            {
                RobotState local_goal;
                global_path_rw.rolling_path_ = global_path_past_;
                local_goal = global_path_rw.findLocalgoal(cur_pose_);
                // local_goal = rollingWindow(cur_pose_, global_path_past_, lookahead_d_, "global");
                omniController(local_goal, cur_pose_);
            }
            else if (robot_type_ == "diff")
            {
            }
        }
        break;
    }
}

void pathTracker::switchMode(Mode next_mode)
{
    workingMode_past_ = workingMode_;
    workingMode_ = next_mode;
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

void pathTracker::localPlannerClient(RobotState cur_pos, RobotState goal_pos)
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

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/apf_localplanner");
    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    std::vector<geometry_msgs::PoseStamped> path_msg;

    if (client.call(srv))
    {
        ROS_INFO("Path received from global planner !");
        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;
        local_path_.clear();
        local_path_.push_back(cur_pos);

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
            local_path_.push_back(pose);
        }
        local_path_.push_back(goal_pos);
        ROS_INFO("--- local path ---");
        for (const auto& point : local_path_)
        {
            ROS_INFO("(%f, %f, %f)", point.x_, point.y_, point.theta_);
        }
        ROS_INFO("--- ---");
        local_path_ = orientationFilter(local_path_);
        local_path_rw.updatePath(local_path_);
    }
    else
    {
        ROS_ERROR("Failed to call service apf_localplanner");
        // return 1;
    }
}

void pathTracker::obsCallback(const std_msgs::Bool::ConstPtr& obs_msg)
{
    if_obstacle_approached = obs_msg->data;
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

    global_path_past_ = global_path_;
    if (workingMode_ == Mode::IDLE)
    {
        plannerClient(cur_pose_, goal_pose_);
        linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
    }

    // if_globalpath_rw_finished = false;
    global_path_rw.updatePath(global_path_);
    apf_global_path_rw.updatePath(global_path_);
    if_globalpath_switched = false;
    switchMode(Mode::GLOBALPATH_RECEIVED);
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
    cout << "path size" << origin_path.size()<< endl;

    RobotState point(origin_path.at(0).x_, origin_path.at(0).y_, init_theta);
    path.push_back(point);
    // cout << "path size" << origin_path.size()<< endl;

    for (int i = 0; i < origin_path.size(); i++)
    {
        if (i != 0)
        {
            double theta;
            theta = angleLimitChecking(path.at(i - 1).theta_ + d_theta);
            // cout << "theta = " << theta << endl;
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

double pathTracker::velocityProfile(Velocity vel_type, RobotState cur_pos, RobotState goal_pos, RobotState vel_state_,
                                    double acceleration)
{
    double output_vel = 0;
    if (workingMode_ == Mode::TRACKING)
    {
        if (vel_type == Velocity::linear)
        {
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            // acceleration
            if (linear_acceleration_profile_ == "linear")
            {
                double d_vel = acceleration / control_frequency_;
                output_vel = last_vel + d_vel;
            }
            else if (linear_acceleration_profile_ == "smooth_step")
            {
            }

            // deceleration
            if (cur_pose_.distanceTo(goal_pose_) < linear_brake_distance_)
            {
                if (linear_deceleration_profile_ == "linear")
                {
                    double acc = pow(linear_max_vel_, 2) / 2 / linear_brake_distance_;
                    output_vel = sqrt(2 * acc * cur_pose_.distanceTo(goal_pose_));
                }
                else if (linear_deceleration_profile_ == "p_control")
                {
                    output_vel = cur_pos.distanceTo(goal_pos) * linear_kp_;
                }
                else if (linear_deceleration_profile_ == "smooth_step")
                {
                }
            }

            // Saturation
            if (output_vel > linear_max_vel_)
                output_vel = linear_max_vel_;
            ROS_INFO("linear vel %f", output_vel);
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
            // Saturation
            if (output_vel > angular_max_vel_)
                output_vel = angular_max_vel_;
            if (output_vel < -angular_max_vel_)
                output_vel = -angular_max_vel_;
        }
    }

    else if (workingMode_ == Mode::TRANSITION)
    {
        if (vel_type == Velocity::linear)
        {
            double d_vel = linear_transition_acc_ / control_frequency_;
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            output_vel = last_vel - d_vel;
            if (output_vel < linear_transition_vel_)
                output_vel = linear_transition_vel_;
        }

        if (vel_type == Velocity::angular)
        {
            double d_vel = angular_transition_acc_ / control_frequency_;
            if (output_vel > 0)
            {
                output_vel = vel_state_.theta_ - d_vel;
                if (output_vel < angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            }
            else
            {
                output_vel = vel_state_.theta_ + d_vel;
                if (output_vel > angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            }
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
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    pathTracker pathTracker_inst(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
}
