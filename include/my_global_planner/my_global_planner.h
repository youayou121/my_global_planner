#ifndef MY_GLOBAL_PLANNER_H_
#define MY_GLOBAL_PLANNER_H_
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <my_global_planner/dijkstra.h>
namespace my_global_planner
{
    class MyGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        MyGlobalPlanner();
        MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);
        void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);

    private:
        ros::Publisher plan_pub_;
        costmap_2d::Costmap2D *costmap_;
        std::string frame_id_;
    };
};

#endif