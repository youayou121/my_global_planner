#include <nav_core/base_global_planner.h>
#include <my_global_planner/my_global_planner.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)

// Default Constructor
namespace my_global_planner
{

    MyGlobalPlanner::MyGlobalPlanner()
    {
    }

    MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        frame_id_ = costmap_ros->getGlobalFrameID();
        costmap_ = costmap_ros->getCostmap();
    }

    bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        Dijkstra dj = Dijkstra(costmap_, start, goal, plan);
        ROS_INFO("Find Goal!!!");
        // plan.push_back(start);
        // plan.push_back(goal);
        publishPlan(plan);
        return true;
    }

    void MyGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        nav_msgs::Path global_path;
        global_path.poses.resize(plan.size());
        global_path.header.frame_id = frame_id_;
        global_path.header.stamp = ros::Time::now();

        for (int i = 0; i < plan.size(); i++)
        {
            global_path.poses[i] = plan[i];
        }
        plan_pub_.publish(global_path);
    }

};