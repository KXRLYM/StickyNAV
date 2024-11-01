#ifndef __KD_RRT__
#define __KD_RRT__

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>
#include <random>
#include <algorithm>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

static constexpr int8_t KD_DIM = 2;
static constexpr int MAX_RAY_NUM = 1600;

namespace KD_RRT{

static double distance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
    double distance_(0);
    distance_ = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    return distance_;
}


struct KDNode {
    using KDNodePtr = std::shared_ptr<KDNode>;
    geometry_msgs::Point position;
    KDNodePtr left;
    KDNodePtr right;

    KDNodePtr parent; // for rrt!
    double gain;    
    double cost; // mostly distance?
    double value;
    geometry_msgs::Point heading;
    bool has_immutable_parent;
    bool active_path;


    // Constructor with default position (0, 0, 0)
    KDNode(const geometry_msgs::Point& pos = geometry_msgs::Point())
        : left(nullptr), right(nullptr), gain(0.0), cost(0.0), parent(nullptr), value(0.0), position(pos), has_immutable_parent(false), active_path(false) {heading = geometry_msgs::Point();}
};


class KDTree {
public:
    boost::random::mt19937 gen;
    KDTree() : root_(nullptr) {}
    KDTree(const ros::Publisher pub) : debug_marker_publisher_(pub) {}
    KDTree(const std::vector<geometry_msgs::Point>& points) : root_(nullptr) {
        //srand(time(nullptr));
        build_tree(points);
    }
    ~KDTree() {
        // Smart pointers will automatically handle the destruction of the nodes
        // If using raw pointers, you would need to delete them explicitly
        clear_tree();
    }

    void clear_tree() {
        clear_tree_recursive(root_); // Call to a recursive helper function
        root_ = nullptr; // Optional: reset the root pointer
    }

    void update_gain_and_cost() {
        update_recursive(root_);
    }


    void build_tree(const std::vector<geometry_msgs::Point>& points) {
        points_ = points;
        root_ = build_tree_recursive(points_, 0);
    }

    void insert_node(const KDNode::KDNodePtr& node) {
        root_ = insert_node_recursive(root_, node, 0);
    }

    void print_tree() const {
        print_tree_recursive(root_, 0);
    }

    geometry_msgs::Point randomSample(float x_min, float x_max, float y_min, float y_max) {
        std::uniform_real_distribution<double> dist_x(x_min, x_max);
        std::uniform_real_distribution<double> dist_y(y_min, y_max);
        //float x =  x_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (x_max - x_min)));
        //float y =  y_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (y_max - y_min)));

        geometry_msgs::Point p; 
        p.x = dist_x(gen);
        p.y = dist_y(gen);
    
        return p;
    }

    /** will return a nullptr if there is nothing..! */
    KDNode::KDNodePtr nn_search(const geometry_msgs::Point query, double* min_distance = nullptr) const {
        KDNode::KDNodePtr guess(nullptr);
        double _min_distance = std::numeric_limits<double>::max();
        nn_search_recursive(query, root_, guess, _min_distance);
        if (min_distance) {
            *min_distance = _min_distance;
        }

        return guess;
    }

    std::vector<KDNode::KDNodePtr> radius_search(const geometry_msgs::Point& query, double radius) const {
        std::vector<KDNode::KDNodePtr> result;
        radius_search_recursive(query, radius, root_, result, 0);
        return result;
    }

    void visualise_tree(float max_gain) {
        visualization_msgs::MarkerArray markers;
        int id = 1;

        if (root_ == nullptr) {
            return; // No nodes to visualize
        }

        visualization_msgs::Marker remove_marker;
        remove_marker.header.frame_id = "map";
        remove_marker.header.stamp = ros::Time::now();
        remove_marker.ns = "rrt_tree";
        remove_marker.id = 0;
        remove_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(remove_marker);


        // Helper lambda to recursively add markers for each node
        std::function<void(const KDNode::KDNodePtr&)> visualize_node;   
        visualize_node = [&](const KDNode::KDNodePtr& node) {
            if (!node) {
                return;
            }

            // Create a marker for the node
            visualization_msgs::Marker node_marker;
            node_marker.header.frame_id = "map";
            node_marker.header.stamp = ros::Time::now();
            node_marker.ns = "rrt_tree";
            node_marker.id = id++;
            node_marker.type = visualization_msgs::Marker::SPHERE;
            node_marker.action = visualization_msgs::Marker::ADD;
            node_marker.pose.position = node->position;
            node_marker.pose.orientation.w = 1.0;
            node_marker.scale.x = 0.2;
            node_marker.scale.y = 0.2;
            node_marker.scale.z = 0.2;
            node_marker.color.r = std::min(1.0, node->gain / (MAX_RAY_NUM));
            node_marker.color.b = 0.0;
            node_marker.color.g = 0.0;
            node_marker.color.a = 1.0;
            //node_marker.lifetime = ros::Duration(20.0);
            markers.markers.push_back(node_marker);

            // Create a marker for the connection lines
            if (node->parent) {
                visualization_msgs::Marker line_marker;
                line_marker.header.frame_id = "map";
                line_marker.header.stamp = ros::Time::now();
                line_marker.ns = "rrt_tree";
                line_marker.id = id++;
                line_marker.type = visualization_msgs::Marker::LINE_STRIP;
                line_marker.action = visualization_msgs::Marker::ADD;
                line_marker.pose.orientation.w = 1.0;
                line_marker.scale.x = 0.01;
                line_marker.color.r = 0.0;
                line_marker.color.g = 1.0;
                line_marker.color.b = 0.0;
                line_marker.color.a = 1.0;
                
                geometry_msgs::Point p1 = node->position;
                geometry_msgs::Point p2 = node->parent->position;
                line_marker.points.push_back(p1);
                line_marker.points.push_back(p2);
                //line_marker.lifetime = ros::Duration(20.0);
                markers.markers.push_back(line_marker);
            }

            // Recursively visualize left and right subtrees
            visualize_node(node->left);
            visualize_node(node->right);
        };

        // Start visualization from the root
        visualize_node(root_);

        // Publish all markers
        debug_marker_publisher_.publish(markers);

    }

    KDNode::KDNodePtr find_highest_gain_node() const {
        KDNode::KDNodePtr highest_gain_node = nullptr;
        double highest_gain = std::numeric_limits<double>::lowest();
        find_highest_gain_node_recursive(root_, highest_gain_node, highest_gain);
        return highest_gain_node;
    }
private:
    KDNode::KDNodePtr build_tree_recursive(const std::vector<geometry_msgs::Point>& points, unsigned depth) {
        if (points.empty()) {
            return nullptr;
        }

        // Determine the axis to split on
        unsigned axis = depth % KD_DIM;

        // Sort points based on the current axis
        std::vector<geometry_msgs::Point> sorted_points = points;
        std::sort(sorted_points.begin(), sorted_points.end(), [axis](const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
            return (axis == 0 ? a.x : a.y) < (axis == 0 ? b.x : b.y);
        });

        // Find the median
        int median_index = sorted_points.size() / 2;
        geometry_msgs::Point median_point = sorted_points[median_index];

        KDNode::KDNodePtr node = std::make_shared<KDNode>(median_point);

        // Recursively build the left and right subtrees
        std::vector<geometry_msgs::Point> left_points(sorted_points.begin(), sorted_points.begin() + median_index);
        std::vector<geometry_msgs::Point> right_points(sorted_points.begin() + median_index + 1, sorted_points.end());

        node->left = build_tree_recursive(left_points, depth + 1);
        node->right = build_tree_recursive(right_points, depth + 1);

        return node;
    }

    KDNode::KDNodePtr insert_node_recursive(const KDNode::KDNodePtr& root, const KDNode::KDNodePtr& newNode, unsigned depth) {
        if (!root) {
            return newNode;
        }

        // Determine the axis to compare on
        unsigned axis = depth % KD_DIM;

        // Compare points and decide whether to go left or right
        if ((axis == 0 && newNode->position.x < root->position.x) ||
            (axis == 1 && newNode->position.y < root->position.y)) {
            root->left = insert_node_recursive(root->left, newNode, depth + 1);
        } else {
            root->right = insert_node_recursive(root->right, newNode, depth + 1);
        }

        return root;
    }

    void print_tree_recursive(const KDNode::KDNodePtr& node, int depth) const {
        if (!node) {
            return;
        }

        // Indentation for depth
        std::string indent(depth * 2, ' ');

        // Print the current node's details
        std::cout << indent << "Depth " << depth
                << ": (" << node->position.x << ", " << node->position.y << ", " << node->position.z << ")"
                << ", Gain: " << node->gain
                << ", Value: " << node->value
                << ", Cost: " << node->cost;

        // Print the parent's coordinates if the parent exists
        if (node->parent) {
            std::cout << " | parent: (" << node->parent->position.x << ", " 
                    << node->parent->position.y << ", " << node->parent->position.z << ")";
        }

        std::cout << "\n";

        // Recursively print left and right subtrees
        print_tree_recursive(node->left, depth + 1);
        print_tree_recursive(node->right, depth + 1);
    }

    void nn_search_recursive(const geometry_msgs::Point& query, const KDNode::KDNodePtr& node,
                             KDNode::KDNodePtr& best_node, double& best_distance, unsigned depth = 0) const {
        if (!node) {    
            return;
        }

        // Calculate the distance from the query point to the current node's point
        double distance_ = distance(query, node->position);

        // Update the best node and distance if this node is closer
        if (distance_ < best_distance) {
            best_distance = distance_;
            best_node = node;
        }

        // Determine the axis to split on
        unsigned axis = depth % KD_DIM;
        double plane_distance = (axis == 0 ? query.x : query.y) -
                                (axis == 0 ? node->position.x : node->position.y);

        // Determine which subtree to search first
        KDNode::KDNodePtr nearer_node = (axis == 0 && query.x < node->position.x) ||
                                        (axis == 1 && query.y < node->position.y) ? node->left : node->right;

        KDNode::KDNodePtr further_node = (axis == 0 && query.x < node->position.x) ||
                                         (axis == 1 && query.y < node->position.y) ? node->right : node->left;

        // Search the nearer subtree
        if (nearer_node) {
            nn_search_recursive(query, nearer_node, best_node, best_distance, depth + 1);
        }

        // Check if we need to search the further subtree
        if (std::abs(plane_distance) < best_distance && further_node) {
            nn_search_recursive(query, further_node, best_node, best_distance, depth + 1);
        }
    }

    void clear_tree_recursive(KDNode::KDNodePtr node) {
        if (!node) return;
        clear_tree_recursive(node->left);
        clear_tree_recursive(node->right);
        node.reset(); // Assuming you're using smart pointers
    }


    void update_recursive(KDNode::KDNodePtr node) {
        if (!node) {
            return;
        }

        
        if (node->parent && !node->active_path) {
            float dist = std::sqrt(std::pow(node->position.x - node->parent->position.x,2) + std::pow(node->position.y - node->parent->position.y,2));
            node->cost = node->parent->cost + dist;
        }

        update_recursive(node->left);
        update_recursive(node->right);
    }

    void radius_search_recursive(const geometry_msgs::Point& query, double radius,
                                 const KDNode::KDNodePtr& node, std::vector<KDNode::KDNodePtr>& result,
                                 unsigned depth) const {
        if (!node) {
            return;
        }

        // Calculate the distance from the query point to the current node's point
        double distance_ = distance(query, node->position);

        // Check if the current node is within the radius
        if (distance_ <= radius) {
            result.push_back(node);
        }

        // Determine the axis to split on
        unsigned axis = depth % KD_DIM;
        double plane_distance = (axis == 0 ? query.x : query.y) -
                                (axis == 0 ? node->position.x : node->position.y);

        // Traverse the subtree on the side of the splitting plane that might contain points within the radius
        if (plane_distance <= radius) {
            radius_search_recursive(query, radius, node->left, result, depth + 1);
            radius_search_recursive(query, radius, node->right, result, depth + 1);
        } else {
            // Only search the side that is within the possible radius range
            if (plane_distance > 0) {
                radius_search_recursive(query, radius, node->right, result, depth + 1);
            } else {
                radius_search_recursive(query, radius, node->left, result, depth + 1);
            }
        }
    }

    void find_highest_gain_node_recursive(const KDNode::KDNodePtr& node,
                                          KDNode::KDNodePtr& highest_gain_node,
                                          double& highest_gain) const {
        if (!node) {
            return; // Base case: if the node is nullptr, return.
        }

        // Check if the current node has a higher gain than the current highest
        
        float value = node->value;
        if (value > highest_gain) {
            highest_gain = value;
            highest_gain_node = node;
        }

        // Recursively check left and right subtrees
        find_highest_gain_node_recursive(node->left, highest_gain_node, highest_gain);
        find_highest_gain_node_recursive(node->right, highest_gain_node, highest_gain);
    }

    KDNode::KDNodePtr root_;
    std::vector<geometry_msgs::Point> points_;
    ros::Publisher debug_marker_publisher_;
};
} // namespace end

#endif