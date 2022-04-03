#ifndef __ROLLINGWINDOW_H__
#define __ROLLINGWINDOW_H__

#include <pathTracker.h>

class rollingWindow
{
  public:
    rollingWindow(ros::NodeHandle& nh, std::vector<RobotState> path, double R, std::string rviz_topic);

    ros::NodeHandle nh_;
    ros::Publisher posePub_;

    std::vector<RobotState> rolling_path_;
    std::string rviz_topic;
    double window_radius_;
    double if_pathfinal_reached;

    RobotState findLocalgoal(RobotState cur_pose);
    void updatePath(std::vector<RobotState>);
};

#endif
