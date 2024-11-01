#include <ros/ros.h>
#include <queue>
#include <unordered_set>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <functional> // For std::hash

#include "ray_casting.h"
#include "octomapper.h"
#include <cmath> // for std::abs

// Custom hash function for octomap::OcTreeKey
struct OcTreeKeyHash {
    std::size_t operator()(const octomap::OcTreeKey& key) const {
        // Combine the hash of each coordinate
        std::size_t h1 = std::hash<int>()(key[0]);
        std::size_t h2 = std::hash<int>()(key[1]);
        std::size_t h3 = std::hash<int>()(key[2]);

        // Combine the hash values using XOR and bit shifting
        return h1 ^ (h2 << 1) ^ (h3 << 1);
    }
};

// Custom equality function for octomap::OcTreeKey
struct OcTreeKeyEqual {
    bool operator()(const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const {
        return lhs == rhs;
    }
};

class TestNode {
public:
    TestNode() {
        // Initialize the subscriber to the /octomap_full topic
        octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("/octomap_full", 1, &TestNode::octomapCallback, this);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/test_node_octree", 1);
        voxel_count_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/voxel_count", 1);
    }
private:

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& map_msg)
    {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    
        geometry_msgs::Point min_point;
        min_point.x = -5.1;
        min_point.y = 5.85;
        min_point.z = 0;
        geometry_msgs::Point max_point;
        max_point.x = -5.0;
        max_point.y = 5.95;
        max_point.z = 3;

        bool isSet(false);
        octomap::point3d start_point;
        float epsilon = 0.001;

        if (octree) {
            float resolution = octree->getResolution();
            for (float x = min_point.x + epsilon; x < max_point.x; x += resolution) {
                for (float y = min_point.y + epsilon; y < max_point.y; y += resolution) {
                    for (float z = min_point.z + epsilon; z < max_point.z; z += resolution) {
                        // Perform the search
                        octomap::OcTreeNode* res = octree->search(x, y, z);
                        if (res && res->getOccupancy() > 0.5 && !isSet) {
                            ROS_INFO("starting the occupied voxel");
                            start_point.x() = x;
                            start_point.y() = y;
                            start_point.z() = z;
                            isSet = true;
                            break;
                        }


                    }
                }
            } 

            std::vector<octomap::point3d> frontiers;
            std::queue<octomap::OcTreeKey> keysToSearch;
            std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash, OcTreeKeyEqual> visited;

            std::vector<octomap::point3d> unknown_frontiers;
            std::queue<octomap::OcTreeKey> unknown_keysToSearch;
            std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash, OcTreeKeyEqual> unknown_visited;
            
            // mark interconnected occupied cells in the bounding box and treat it as an object
            int voxel_num(0);
            if (isSet) {
                octomap::OcTreeKey start_key = octree->coordToKey(start_point);
                keysToSearch.push(start_key);
                visited.insert(start_key);

                while (!keysToSearch.empty()) {
                    octomap::OcTreeKey current_key = keysToSearch.front();
                    keysToSearch.pop();
                    octomap::OcTreeNode* current_node = octree->search(octree->keyToCoord(current_key));
                    if (!current_node) continue;
                    if (current_node->getOccupancy() > 0.5) {
                        int count(0);
                        for (const auto& neighbor_key : getNeighbors(current_key)) {
                            octomap::OcTreeNode* neighbor_node = octree->search(octree->keyToCoord(neighbor_key));
                            if (!neighbor_node || neighbor_node->getOccupancy() < 0.5) count++; // Frontier cell if neighboring cell is unknown
                        }
                        if (count >= 4) {
                            voxel_num++;

                        }

                        // Enqueue neighboring nodes
                        for (const auto& neighbor_key : getNeighbors(current_key)) {
                            // only if the neighbour key is in the changedkey, add to the search list
                            if (visited.find(neighbor_key) == visited.end()) {
                                visited.insert(neighbor_key);
                                keysToSearch.push(neighbor_key);                                
                            }
                        }
                    }
                }
            }
            ROS_INFO("current voxel count is %d", voxel_num);
            geometry_msgs::PoseStamped voxel_count;
            voxel_count.header.stamp = ros::Time::now();
            voxel_count.header.frame_id = "map";
            voxel_count.pose.position.x = static_cast<float>(voxel_num);
            voxel_count_pub_.publish(voxel_count);

            octomap_msgs::Octomap bmap_msg;
            octomap_msgs::fullMapToMsg(*octree, bmap_msg);
            bmap_msg.header.stamp = ros::Time::now();
            bmap_msg.header.frame_id = "map";
            ROS_INFO("New map published!");
            octomap_pub_.publish(bmap_msg);    
            delete octree; 
        } else {
            ROS_ERROR("Invalid octree receieved by the test node");
        }
    }

    std::vector<octomap::OcTreeKey> getNeighbors(const octomap::OcTreeKey& key) {
    std::vector<octomap::OcTreeKey> neighbors;
    const int deltas[3] = {-1, 0, 1};

    for (int dx : deltas) {
        for (int dy : deltas) {
            for (int dz : deltas) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                octomap::OcTreeKey neighbor = key;
                neighbor[0] += dx;
                neighbor[1] += dy;
                neighbor[2] += dz;
                neighbors.push_back(neighbor);
            }
        }
    }

    return neighbors;
}

    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Publisher octomap_pub_;
    ros::Publisher voxel_count_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");

    // Create an instance of the RayCastingNode class
    TestNode node;

    // Enter the ROS event loop
    //ros::spin();

    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}