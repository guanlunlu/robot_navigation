#ifndef __PATHTRACKER_H__
#define __PATHTRACKER_H__

#include <Eigen/Dense>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
// message
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <std_msgs/Bool.h>

enum class Mode
{
    IDLE,
    TRACKING,
    TRANSITION,
    GLOBALPATH_RECEIVED
};

enum class Velocity
{
    linear,
    angular
};

class RobotState
{
  public:
    RobotState(double x, double y, double theta);
    RobotState()
    {
    }
    double x_;
    double y_;
    double theta_;
    Eigen::Vector3d getVector();
    double distanceTo(RobotState);
};

class rollingWindow
{
  public:
    rollingWindow(ros::NodeHandle& nh, std::vector<RobotState> path, double R, std::string rviz_topic);
    rollingWindow()
    {
    }
    
    ros::NodeHandle nh_;
    ros::Publisher posePub_;

    std::vector<RobotState> rolling_path_;
    std::string rviz_topic;
    double window_radius_;
    double if_pathfinal_reached;

    RobotState findLocalgoal(RobotState cur_pose);
    void updatePath(std::vector<RobotState>);
};

class pathTracker
{
  public:
    pathTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~pathTracker();
    bool initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void initialize();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    // subscriber
    ros::Subscriber poseSub_;
    ros::Subscriber goalSub_;
    ros::Subscriber obsSub_;
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void obsCallback(const std_msgs::Bool::ConstPtr& obs_msg);

    // Publisher
    ros::Publisher velPub_;
    void velocityPublish();
    ros::Publisher localgoalPub_;
    ros::Publisher posearrayPub_;
    // Client
    void plannerClient(RobotState, RobotState);
    void localPlannerClient(RobotState, RobotState);

    RobotState goal_pose_;
    RobotState cur_pose_;
    RobotState velocity_state_;

    bool if_obstacle_approached;
    bool if_globalpath_rw_finished;
    bool if_localpath_rw_finished;
    bool if_reached_target_range;

    bool xy_goal_reached(RobotState cur_pose_, RobotState goal_pose_);
    bool theta_goal_reached(RobotState cur_pose_, RobotState goal_pose_);

    // Goal rquest from Main and Path received from global planner
    std::vector<RobotState> global_path_;
    std::vector<RobotState> global_path_past_;
    std::vector<RobotState> local_path_;

    // timer setup
    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent& e);
    Mode workingMode_;
    Mode workingMode_past_;
    void switchMode(Mode next_mode);
    bool if_globalpath_switched;

    // controller parameter
    std::string robot_type_;
    bool p_active_;
    double control_frequency_;
    double lookahead_d_;
    double far_lookahead_d_;

    double linear_kp_;
    double linear_max_vel_;
    double linear_acceleration_;
    double linear_brake_distance_;
    double linear_transition_vel_;
    double linear_transition_acc_;
    // double linear_tracking_vel_;
    double xy_tolerance_;
    double linear_brake_distance_ratio_;
    // velocity profile type : linear, smooth_step
    std::string linear_acceleration_profile_;
    std::string linear_deceleration_profile_;

    double angular_kp_;
    double angular_max_vel_;
    double angular_acceleration_;
    double angular_brake_distance_;
    double angular_transition_vel_;
    double angular_transition_acc_;
    double theta_tolerance_;
    double theta_err;
    // velocity profile type : p_control, linear, smooth_step
    std::string angular_acceleration_profile_;
    std::string angular_deceleration_profile_;

    double angleLimitChecking(double theta);
    double velocityProfile(Velocity, RobotState cur_pos, RobotState goal_pos, RobotState velocity_state,
                           double acceleration_sign);
    // Path post process
    std::vector<RobotState> orientationFilter(std::vector<RobotState>);
    int rotate_direction_;

    // rollingWindow method
    rollingWindow apf_global_path_rw;
    rollingWindow local_path_rw;
    rollingWindow global_path_rw;

    void diffController(RobotState local_goal, RobotState cur_pos);
    void omniController(RobotState local_goal, RobotState cur_pos);
};

#endif
