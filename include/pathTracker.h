#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// message
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

enum class Mode{
    IDLE,
    TRACKING
};

class RobotState{
    public:
        RobotState(double x, double y, double theta);
        RobotState(){}
        double x_;
        double y_;
        double theta_;
        Eigen::Vector3d getVector();
        double distanceTo(RobotState);
};

class pathTracker{
    public:
        pathTracker(ros::NodeHandle& nh);
        void initialize();

    private:
        ros::NodeHandle nh_;
        // subscriber
        ros::Subscriber poseSub_;
        ros::Subscriber goalSub_;
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        // Publisher
        ros::Publisher velPub_;
        ros::Publisher localgoalPub_;
        ros::Publisher posearrayPub_;
        // Client
        void plannerClient(RobotState, RobotState);

        RobotState goal_pose_;
        RobotState cur_pose_;
        RobotState local_goal_pose_;
        bool if_localgoal_final_reached;
        // Goal rquest from Main and Path received from global planner
        std::vector<RobotState> global_path_;

        // timer setup
        ros::Timer timer_;
        void timerCallback(const ros::TimerEvent& e);
        
        // controller parameter
        double control_frequency_;
        double lookahead_d_;

        double linear_kp_;
        double linear_max_vel_;
        double linear_acceleration_;
        double linear_brake_distance_;
        double xy_tolerance_;

        double angular_kp_;
        double angular_max_vel_;
        double angular_acceleration_;
        double angular_brake_distance_;
        double theta_tolerance_;

        bool if_globalpath_received;
        double angleLimitChecking(double theta);
        RobotState rollingWindow(RobotState cur_pos, std::vector<RobotState> global_path, double R);
        // Path post process
        std::vector<RobotState> orientationFilter(std::vector<RobotState>);
        RobotState diffController(RobotState local_goal, RobotState cur_pos);
        RobotState omniController(RobotState local_goal, RobotState cur_pos);
};