#ifndef _NODE_H_
#define _NODE_H_

namespace my_global_planner
{
    class Node
    {
    public:
        int x_;
        int y_;
        double cost_;
        int parent_index_;
        bool searched;
        Node();
        Node(int x, int y, double cost, int parent_index);
        ~Node(){};
    };
    Node::Node()
    {
        x_ = -1;
        y_ = -1;
        cost_ = -1;
        parent_index_ = -1;
        searched = false;
    }
    Node::Node(int x, int y, double cost, int parent_index)
    {
        x_ = x;
        y_ = y;
        cost_ = cost;
        parent_index_ = parent_index;
        searched = false;
    }
}
#endif