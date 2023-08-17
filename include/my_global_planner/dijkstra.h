#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_
#include <my_global_planner/node.h>
#include <geometry_msgs/PoseStamped.h>
namespace my_global_planner
{
    class Dijkstra

    {
    private:
        costmap_2d::Costmap2D *costmap_;
        unsigned int *costarr_;
        int nx, ny, ns;
        double pre_min_cost;
        double resolution_;
        Node *open_list;
        Node *closed_list;
        double motion[8][3] = {
            {0, 1, 1},
            {0, -1, 1},
            {-1, 0, 1},
            {1, 0, 1},
            {-1, 1, sqrt(2)},
            {-1, -1, sqrt(2)},
            {1, 1, sqrt(2)},
            {1, -1, sqrt(2)}

        };

    public:
        ros::Publisher searched_map_pub;
        Dijkstra(costmap_2d::Costmap2D *costmap,
                 const geometry_msgs::PoseStamped &start,
                 const geometry_msgs::PoseStamped &goal,
                 std::vector<geometry_msgs::PoseStamped> &plan);
        void planning(unsigned int *costarr_,
                      double resolution,
                      int start_x, int start_y,
                      int goal_x, int goal_y,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        void setNavArr(costmap_2d::Costmap2D *costmap_);
        void setCostmap(costmap_2d::Costmap2D *costmap_, bool allow_unknown);
        int calcMapIndex(Node node);
        void getPath(Node *closed_list, std::vector<geometry_msgs::PoseStamped> &plan);
        bool nodeInMap(Node node);
        int find_min_by_cost();
        void publishSearchedMap();
        ~Dijkstra();
    };

    Dijkstra::Dijkstra(costmap_2d::Costmap2D *costmap,
                       const geometry_msgs::PoseStamped &start,
                       const geometry_msgs::PoseStamped &goal,
                       std::vector<geometry_msgs::PoseStamped> &plan)
    {
        ros::NodeHandle nh("~/Dijkstra");
        searched_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("searched_map", 1);
        costmap_ = costmap;
        pre_min_cost = 0;
        setNavArr(costmap_);
        setCostmap(costmap_, true);
        resolution_ = costmap_->getResolution();
        double wx = start.pose.position.x;
        double wy = start.pose.position.y;

        unsigned int start_mx, start_my, goal_mx, goal_my;
        if (!costmap_->worldToMap(wx, wy, start_mx, start_my))
        {
            ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        }
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;
        if (!costmap_->worldToMap(wx, wy, goal_mx, goal_my))
        {
            ROS_WARN_THROTTLE(1.0, "The robot's goal position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        }

        ROS_INFO("Map start(%d,%d),goal(%d,%d)", start_mx, start_my, goal_mx, goal_my);
        planning(costarr_, resolution_, start_mx, start_my, goal_mx, goal_my, plan);
    }

    void Dijkstra::planning(unsigned int *costarr_,
                            double resolution,
                            int start_x, int start_y,
                            int goal_x, int goal_y,
                            std::vector<geometry_msgs::PoseStamped> &plan)
    {
        Node start_node = Node(start_x, start_y, 0, -1);
        Node goal_node = Node(goal_x, goal_y, -1, -1);
        open_list[calcMapIndex(start_node)] = start_node;
        while (1)
        {
            int current_id;
            current_id = find_min_by_cost();
            Node current_node = open_list[current_id];
            if (current_node.x_ == goal_x && current_node.y_ == goal_y)
            {
                ROS_INFO("Find goal!!!");
                goal_node.parent_index_ = current_node.parent_index_;
                goal_node.cost_ = current_node.cost_;
                break;
            }
            closed_list[current_id] = current_node;

            // ROS_INFO("curren_id:%d,current node:x:%d,y:%d,cost:%.2f,parent_index:%d,%d", current_id, current_node.x_, current_node.y_, current_node.cost_, current_node.parent_index_, current_node.searched);
            for (int i = 0; i < 8; i++)
            {
                Node node = Node(current_node.x_ + (int)motion[i][0],
                                 current_node.y_ + (int)motion[i][1],
                                 current_node.cost_ + motion[i][2],
                                 current_id);
                if (!nodeInMap(node))
                {
                    continue;
                }
                int next_id = calcMapIndex(node);
                if (closed_list[next_id].cost_ > 0)
                {
                    continue;
                }
                if (costarr_[next_id] > 0)
                {
                    continue;
                }
                if (open_list[next_id].searched == false)
                {
                    open_list[next_id] = node;
                }
                else
                {
                    if (open_list[next_id].cost_ >= node.cost_ && node.cost_ > 0)
                    {
                        open_list[next_id] = node;
                    }
                }
            }
            open_list[current_id].searched = true;
            publishSearchedMap();
        }
        int pre_index = goal_node.parent_index_;
        int path_size = 0;
        while (pre_index != -1)
        {
            // ROS_INFO("node_x:%d", closed_list[pre_index].x_);
            pre_index = closed_list[pre_index].parent_index_;
            path_size++;
        }

        int path[path_size];
        pre_index = goal_node.parent_index_;
        int i = 0;
        while (pre_index != -1)
        {
            // ROS_INFO("node_x:%d", closed_list[pre_index].x_);
            pre_index = closed_list[pre_index].parent_index_;
            path[i] = pre_index;
            i++;
        }
        for (int i = path_size - 2; i >= 0; i--)
        {
            geometry_msgs::PoseStamped pose;
            int mx = open_list[path[i]].x_;
            int my = open_list[path[i]].y_;
            double wx;
            double wy;
            ROS_INFO("%d",mx);
            costmap_->mapToWorld(mx,my,wx,wy);
            pose.header.frame_id ="map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            plan.push_back(pose);
        }
    }

    void Dijkstra::setNavArr(costmap_2d::Costmap2D *costmap_)
    {
        ROS_INFO("Set naviagation array");
        nx = costmap_->getSizeInCellsX();
        ny = costmap_->getSizeInCellsY();
        ns = nx * ny;
        costarr_ = new unsigned int[ns];
        open_list = new Node[ns];
        closed_list = new Node[ns];
        ROS_INFO("map size x:%d,y:%d", nx, ny);
    }

    void Dijkstra::setCostmap(costmap_2d::Costmap2D *costmap_, bool allow_unknown)
    {
        ROS_INFO("Set costmap array.");
        unsigned int *cm = costarr_;
        unsigned char *cmap = costmap_->getCharMap();
        for (int i = 0; i < ny; i++)
        {
            for (int j = 0; j < nx; j++)
            {
                *cm = 0;
                int v = *cmap;
                if (v > 252 && v < 255)
                {
                    *cm = 1;
                }
                else if (v == 255 && !allow_unknown)
                {
                    *cm = 1;
                }
                cm++;
                cmap++;
            }
        }
    }

    int Dijkstra::calcMapIndex(Node node)
    {
        int index = node.x_ + node.y_ * nx;
        if (node.x_ > nx || node.y_ > ny)
        {
            return -1;
        }
        else
        {
            return index;
        }
    }

    void Dijkstra::getPath(Node *closed_list, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        geometry_msgs::PoseStamped start;
    }

    int Dijkstra::find_min_by_cost()
    {
        double min_cost = 10000;
        // double min_cost = pre_min_cost;
        int index = 0;
        for (int i = 0; i < ns; i++)
        {
            double cost = open_list[i].cost_;
            bool searched = open_list[i].searched;
            // if (!searched)
            // {
            //     if (cost == pre_min_cost + 1)
            //     {
            //         index = i;
            //         pre_min_cost = cost;
            //         return index;
            //     }
            //     else if (cost == pre_min_cost + sqrt(2))
            //     {
            //         index = i;
            //         pre_min_cost = cost;
            //         return index;
            //     }
            // }

            if (cost < min_cost && cost >= 0 && !searched)
            {
                index = i;
                min_cost = cost;
            }
        }
        return index;
    }

    bool Dijkstra::nodeInMap(Node node)
    {
        if (node.x_ < 0 || node.y_ < 0 || node.x_ >= nx || node.y_ >= ny)
        {
            return false;
        }
        return true;
    }

    void Dijkstra::publishSearchedMap()
    {
        nav_msgs::OccupancyGrid map;
        map.header.frame_id = "map";
        map.header.stamp = ros::Time::now();
        map.info.height = ny;
        map.info.width = nx;
        map.info.origin.position.x = costmap_->getOriginX();
        map.info.origin.position.y = costmap_->getOriginY();
        map.info.resolution = resolution_;
        map.data.resize(ns);
        for (int i = 0; i < ns; i++)
        {
            if (open_list[i].searched)
            {
                map.data[i] = 100;
            }
            else
            {
                map.data[i] = 0;
            }
        }
        searched_map_pub.publish(map);
    }

    Dijkstra::~Dijkstra()
    {
    }
}
#endif
