#ifndef RRT_H
#define RRT_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include <cstdlib>

namespace RRT {
// Forward declaration of Node struct
struct Node {
    geometry_msgs::Point position;
    int parent;  // Index of the parent node
    double gain;    
    double cost; // mostly distance?


    // Constructor with default position (0, 0, 0)
    Node(const geometry_msgs::Point& pos = geometry_msgs::Point(), int parent = -1)
        : position(pos), parent(parent), gain(0.0), cost(0.0) {}
};

// Tree class definition
class Tree {
public:
    Tree();
    // Constructor
    Tree(float max_step_size, nav_msgs::OccupancyGrid& grid, ros::Publisher& marker_pub, geometry_msgs::Point& start, geometry_msgs::Point& goal);
    // Main function to run the RRT algorithm
    void init();

    geometry_msgs::Point randomSample(float x_min, float x_max, float y_min, float y_max);

    // Set the occupancy grid
    void setGrid(const nav_msgs::OccupancyGrid& msg);

    void setPoints(const float step, const geometry_msgs::Point& start, const geometry_msgs::Point& goal);

    void setPub(const ros::Publisher& pub);

    void printTree();

    void rewire();

    void addNode(Node& new_node);

    std::vector<Node> returnTree();
    
    // Find the nearest node to a given point
    int nearestNode(const geometry_msgs::Point& point);
    
    // Extend the tree towards a random point
    Node extendTree(int nearest, const geometry_msgs::Point& random_point, float gain);
    
    // Check if there is no collision between two nodes
    bool isCollisionFree(const Node& node1, const Node& node2);
    
    // Visualize the tree using ROS markers
    void visualizeTree();

    void addGain(int node_idx, float gain);

private:

    std::vector<Node> tree_;  // Vector to store the nodes of the RRT tree
    double max_step_size_;  // Maximum step size for extending the tree
    double grid_resolution_;  // Resolution of the occupancy grid
    nav_msgs::OccupancyGrid grid_;  // Occupancy grid for collision detection
    geometry_msgs::Point start_;  // Starting point of the RRT
    geometry_msgs::Point goal_;  // Goal point of the RRT
    ros::Publisher marker_pub_;
};

} // namespace RRT

#endif // RRT_H
