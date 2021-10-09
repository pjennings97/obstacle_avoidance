#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#ifndef GOAL_PUBLISHER_CPP
#define GOAL_PUBLISHER_CPP

class GoalPublisher
{
    public:

    GoalPublisher();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void statusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& map);
    void publishRandomGoal();
    ros::Publisher pub;
    ros::Subscriber map_sub;
    ros::Subscriber status_sub;
    int padding;

    private:

    int count;
    bool got_map;
    nav_msgs::OccupancyGrid map;
};

#endif