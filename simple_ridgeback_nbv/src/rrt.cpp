#include "rrt.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

RRT::Tree::Tree(){}


RRT::Tree::Tree(float max_step_size, nav_msgs::OccupancyGrid& grid, ros::Publisher& marker_pub, geometry_msgs::Point& start, geometry_msgs::Point& goal)
    : max_step_size_(max_step_size),
      grid_(grid),
      grid_resolution_(grid.info.resolution),
      marker_pub_(marker_pub),
      start_(start),
      goal_(goal) {


}

std::vector<RRT::Node> RRT::Tree::returnTree() {
    return tree_;
}

void RRT::Tree::init() {
    srand(time(nullptr));  // Seed for random sampling
    tree_.clear();
    RRT::Node start_node = {start_, -1}; 
    ROS_INFO("RRT at the node %f, %f", start_.x, start_.y);
    tree_.push_back(start_node);

    /**  
    int max_iterations = 20;
    for (int i = 0; i < max_iterations; ++i) {
        geometry_msgs::Point random_point = randomSample(radius, debug_markers);
        int nearest = nearestNode(random_point);
        RRT::Node new_node = extendTree(nearest, random_point);
        if (isCollisionFree(tree_[nearest], new_node)) {
            tree_.push_back(new_node);

            if (std::sqrt(std::pow(new_node.position.x - goal_.x, 2) +
                        std::pow(new_node.position.y - goal_.y, 2)) < max_step_size_) {
                ROS_INFO("Goal reached!");
                break;
            }
        }
        marker_pub_.publish(debug_markers);
    }
    
    visualizeTree(); */  
}

void RRT::Tree::rewire() {

}

void RRT::Tree::printTree() {
    for (size_t i = 0; i < tree_.size(); ++i) {
        const auto& node = tree_[i];
        if (node.parent != -1) {
            const auto& parent = tree_[node.parent];
            ROS_INFO("Node %li and its parent %i", i, node.parent);
        } else {
            ROS_INFO("Root node at %f, %f, %f", node.position.x, node.position.y, node.position.z);
        }
    }
}

RRT::Node RRT::Tree::extendTree(int nearest, const geometry_msgs::Point& random_point, float gain) {
    RRT::Node new_node;
    float max_step_size = 2;
    new_node.position.x = tree_[nearest].position.x + max_step_size_ * (random_point.x - tree_[nearest].position.x) / std::sqrt(std::pow(random_point.x - tree_[nearest].position.x, 2) + std::pow(random_point.y - tree_[nearest].position.y, 2));
    new_node.position.y = tree_[nearest].position.y + max_step_size_ * (random_point.y - tree_[nearest].position.y) / std::sqrt(std::pow(random_point.x - tree_[nearest].position.x, 2) + std::pow(random_point.y - tree_[nearest].position.y, 2));
    new_node.position.z = 0;
    new_node.parent = nearest;
    new_node.gain = gain + tree_[nearest].gain;
    return new_node;
}

void RRT::Tree::addGain(int node_idx, float gain) {
    auto& node = tree_[node_idx];
    node.gain += gain;
}

/**
RRT::Node RRT::Tree::extendTree(int nearest, const geometry_msgs::Point& random_point) {
    RRT::Node new_node;
    new_node.position.x = tree_[nearest].position.x + max_step_size_ * (random_point.x - tree_[nearest].position.x) / std::sqrt(std::pow(random_point.x - tree_[nearest].position.x, 2) + std::pow(random_point.y - tree_[nearest].position.y, 2));
    new_node.position.y = tree_[nearest].position.y + max_step_size_ * (random_point.y - tree_[nearest].position.y) / std::sqrt(std::pow(random_point.x - tree_[nearest].position.x, 2) + std::pow(random_point.y - tree_[nearest].position.y, 2));
    new_node.position.z = 0;
    new_node.parent = nearest;
    return new_node;
} */

int RRT::Tree::nearestNode(const geometry_msgs::Point& point) {
    int nearest_index = 0;
    double min_dist = std::sqrt(std::pow(point.x - tree_[0].position.x, 2) +
                                std::pow(point.y - tree_[0].position.y, 2));
                                
    for (size_t i = 1; i < tree_.size(); ++i) {
        double dist = std::sqrt(std::pow(point.x - tree_[i].position.x, 2) +
                                std::pow(point.y - tree_[i].position.y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            nearest_index = i;
        }
    }
    ROS_INFO("Nearest node index: %d, position: %f %f", nearest_index, tree_[nearest_index].position.x, tree_[nearest_index].position.y);
    return nearest_index;
}

geometry_msgs::Point RRT::Tree::randomSample(float x_min, float x_max, float y_min, float y_max) {

    float x =  x_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (x_max - x_min)));
    float y =  y_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (y_max - y_min)));

    geometry_msgs::Point p; 
    p.x = x;
    p.y = y;
    ROS_INFO("sampled at %f, %f", p.x, p.y);

    return p;
}

/** 
geometry_msgs::Point RRT::Tree::randomSample(float radius, visualization_msgs::MarkerArray debug_marker) {
    float rand_float = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    geometry_msgs::Point p; 
    float r = radius * std::sqrt(rand_float);
    float rand_theta = (rand_float) * 2 * M_PI;
    p.x = start_.x + r * std::cos(rand_theta);
    p.y = start_.y + r * std::sin(rand_theta);
    ROS_INFO("sampled at %f, %f", p.x, p.y);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = static_cast<int>(rand_theta);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    debug_marker.markers.push_back(marker);
    
    return p;
}*/

void RRT::Tree::addNode(Node& new_node) {
    tree_.push_back(new_node);
}
    
void RRT::Tree::setGrid(const nav_msgs::OccupancyGrid& msg) {
    grid_ = msg;
}

void RRT::Tree::setPub(const ros::Publisher& pub) {
    marker_pub_ = pub;
}

void RRT::Tree::setPoints(const float step, const geometry_msgs::Point& start, const geometry_msgs::Point& goal) {
    max_step_size_ = step;
    start_ = start;
    goal_ = goal;
}

bool RRT::Tree::isCollisionFree(const Node& node1, const Node& node2) {
    // Implement collision detection with the occupancy grid here
    // For simplicity, assuming no collision here
    return true;
}

void RRT::Tree::visualizeTree() {
    visualization_msgs::MarkerArray markers;
    int id = 0;
    
    for (size_t i = 0; i < tree_.size(); ++i) {
        const auto& node = tree_[i];
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt_tree";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = node.position;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        markers.markers.push_back(marker);
        
        if (node.parent != -1) {
            const auto& parent = tree_[node.parent];
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "rrt_tree";
            line_marker.id = id++;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.pose.orientation.w = 1.0;
            line_marker.scale.x = 0.05;
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;
            
            geometry_msgs::Point p1 = node.position;
            geometry_msgs::Point p2 = parent.position;
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            markers.markers.push_back(line_marker);
        }
    }
    marker_pub_.publish(markers);
}
