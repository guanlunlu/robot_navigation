#include "rollingWindow.h"

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
    posePub_.publish(pos_msg);

    return local_goal;
}

void rollingWindow::updatePath(std::vector<RobotState> update_path)
{
    rolling_path_ = update_path;
    if_pathfinal_reached = false;
}
