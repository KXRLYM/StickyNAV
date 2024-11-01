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

bool isClose(float a, float b, float epsilon = 1e-3f) {
    return std::abs(a - b) < epsilon;
}


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

class MapNode
{
public:

    octomap::OcTree target_reconstruction_; // Public member variable

    MapNode ()
        : target_reconstruction_(0.05),
        cuboid_received_(false),
        max_distance_(6.0),
        reconstruction_finished_(false),
        valid_cuboid_(true),
        start_voxel_(0,0,-1)
    {
        // Initialize the subscriber to the /octomap_full topic
        octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>("/octomap_full", 1, &MapNode::octomapCallback, this);
        corner_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/match_cuboid_corners", 1, &MapNode::cornerCallback, this);
        map_corner_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/bounding_corners", 1, &MapNode::mapCornerCallback, this);
        newmap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/unknown_octree", 1);
        sensor_pose_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/sensor_poses", 1, &MapNode::sensorPoseCallback, this);
        reconstruction_pub_ = nh_.advertise<octomap_msgs::Octomap>("/reconstruction", 1);
        debug_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/map_debug", 1);
        recon_completion_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/reconstruction_finished", 1, &MapNode::flagCallback, this);

    }

private:

    // Callback function for handling incoming Octomap messages
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& map_msg)
    {

        if (map_corner_msg_.poses.size() != 5) {
            // corners need to be updated
            ROS_ERROR("four corners and one height must be received to start");
            return;
        }

        Vec3 cube_min(0,0,0);
        Vec3 cube_max(0,0,0);

        if (corner_msg_.poses.size() == 5) {
            cuboid_received_ = true;
            cube_min.x = corner_msg_.poses[MINMIN].position.x;
            cube_min.y = corner_msg_.poses[MINMIN].position.y;
            cube_max.x = corner_msg_.poses[MAXMAX].position.x;
            cube_max.y = corner_msg_.poses[MAXMAX].position.y;
            cube_max.z = corner_msg_.poses[MAXZ].position.z;

        }
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree_temp_ = dynamic_cast<octomap::OcTree*>(tree);

        if (!octree_temp_) {
            ROS_ERROR("Failed to convert the incoming message to Octree");
        }
        octomap::ColorOcTree* octree = new octomap::ColorOcTree(octree_temp_->getResolution());

        // Copy data from the original OcTree to your Coloured OcTree
        for (auto it = octree_temp_->begin_leafs(); it != octree_temp_->end_leafs(); ++it) {
            octomap::point3d point = it.getCoordinate();
            float occupancy = it->getLogOdds();
            octree->updateNode(point, occupancy); 
        }
        
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "map";
         
        if (octree) // can be NULL
        {
            octree->enableChangeDetection(true);
            double resolution = octree->getResolution();

            octomap::point3d pmin(map_corner_msg_.poses[MINMIN].position.x, map_corner_msg_.poses[MINMIN].position.y, 0.0);
            octomap::point3d pmax(map_corner_msg_.poses[MAXMAX].position.x, map_corner_msg_.poses[MAXMAX].position.y, map_corner_msg_.poses[MAXZ].position.z * 0.5);
            octomap::point3d_list unknown_list;
            
            ROS_INFO("corners %f, %f, %f", pmin.x(), pmin.y(), pmin.z());
            ROS_INFO("corners %f, %f, %f", pmax.x(), pmax.y(), pmax.z());

            float epsilon = 0.001;
            octomap::point3d start_point_(0,0,0);
            octomap::point3d start_unknown_point(0,0,0);
            bool isSet(false);
            int at_least_one_occupid(0);

            if (cuboid_received_) {
                for (float x = cube_min.x + epsilon; x < cube_max.x; x += resolution) {
                    for (float y = cube_min.y + epsilon; y < cube_max.y; y += resolution) {
                        for (float z = cube_min.z + epsilon; z < cube_max.z; z += resolution) {
                            // Perform the search
                            octomap::OcTreeNode* res = octree_temp_->search(x, y, z);

                            // Check if the node exists
                            if (res == NULL) {
                                // Node does not exist; create and set a new node with value
                                octree->updateNode(x,y,z, static_cast<float>(0.0));
                                octomap::ColorOcTreeNode* updated_node = octree->search(x, y, z);
                                if (!reconstruction_finished_) {
                                    updated_node->setValue(OCCLUSION);
                                    updated_node->setColor(255, 100, 10);
                                }
                                start_unknown_point.x() = x;
                                start_unknown_point.y() = y;
                                start_unknown_point.z() = z;
                            } else if (octree->isNodeOccupied(res) && !isSet) {
                                ROS_INFO("starting the occupied voxel");
                                start_point_.x() = x;
                                start_point_.y() = y;
                                start_point_.z() = z;
                                start_voxel_.x() = x;
                                start_voxel_.y() = y;
                                start_voxel_.z() = z;
                                isSet = true;
                                at_least_one_occupid++;
                            }


                        }
                    }
                } 
            }

            if (at_least_one_occupid <= 0) {
                valid_cuboid_ = false;
            }

            std::vector<octomap::point3d> frontiers;
            std::queue<octomap::OcTreeKey> keysToSearch;
            std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash, OcTreeKeyEqual> visited;

            std::vector<octomap::point3d> unknown_frontiers;
            std::queue<octomap::OcTreeKey> unknown_keysToSearch;
            std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash, OcTreeKeyEqual> unknown_visited;
            
            // mark interconnected occupied cells in the bounding box and treat it as an object
            octomap::point3d start_point(0,0,-1);
            if (valid_cuboid_) {
                start_point = start_point_;
            } else {
                start_point = start_voxel_;
            } 

            if (isSet && start_voxel_.z() != -1) {

                octomap::OcTreeKey start_key = octree->coordToKey(start_point);
                keysToSearch.push(start_key);
                visited.insert(start_key);

                while (!keysToSearch.empty()) {
                    octomap::OcTreeKey current_key = keysToSearch.front();
                    keysToSearch.pop();
                    octomap::ColorOcTreeNode* current_node = octree->search(octree->keyToCoord(current_key));
                    if (!current_node) continue;
                    double distance = (start_point - octree->keyToCoord(current_key)).norm(); // Assuming `norm()` computes Euclidean distance

                    // Check if the current distance exceeds the threshold
                    if (distance > max_distance_) {
                        continue; // Skip this key if it's too far from the start point
                    }


                    if (current_node->getOccupancy() > 0.5) {
                        int count(0);
                        for (const auto& neighbor_key : getNeighbors(current_key)) {
                            octomap::ColorOcTreeNode* neighbor_node = octree->search(octree->keyToCoord(neighbor_key));
                            if (!neighbor_node || neighbor_node->getOccupancy() < 0.5) count++; // Frontier cell if neighboring cell is unknown
                        }
                        if (count >= 5) {
                            //octomap::OcTreeNode* node = octree->search(octree->keyToCoord(current_key));
                            //ROS_INFO("SPECIAL OBJECT OCCUPANCY %f", node->getOccupancy() );

                            if (cuboid_received_) {
                                octomap::ColorOcTreeNode* updated_node = octree->search(octree->keyToCoord(current_key));
                                // only if the sensor pose is received.. 
                                if (!isClose(updated_node->getValue(), SEEN) && !isClose(updated_node->getValue(),  OCCLUSION)) {
                                    updated_node->setValue(POTENTIAL_OBJECT);

                                    if (reconstruction_finished_) {
                                        // green to completion
                                        updated_node->setColor(10, 255, 10);
                                    } else {
                                        // pink for on going scan
                                        updated_node->setColor(255, 100, 100);
                                    }
                                    
                                }
                            }
                            
                            // ROS_INFO("occupancy: %f", current_node->getOccupancy());
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

            // mark all the area where the map has already seen.
            for (auto it = target_reconstruction_.begin_leafs(); it != target_reconstruction_.end_leafs(); ++it) {
                octomap::point3d point = it.getCoordinate();
                if (it->getLogOdds() > 0.5) {
                    octomap::ColorOcTreeNode* seenNode = octree->search(point);
                    if (seenNode) {
                        seenNode->setValue(SEEN);
                        seenNode->setColor(10,255,10);
                    }
                }
            }
    

            
            for (float x = pmin.x() + epsilon; x < pmax.x(); x += resolution) {
                for (float y = pmin.y() + epsilon; y < pmax.y(); y += resolution) {
                    for (float z = pmin.z() + epsilon; z < pmax.z(); z += resolution) {
                        // Perform the search
                        octomap::OcTreeNode* res = octree_temp_->search(x, y, z);

                        // Check if the node exists
                        if (res == NULL) {
                            // Node does not exist; create and set a new node with value
                            octree->updateNode(x, y, z, static_cast<float>(0.0));

                        }
                    }
                } 
            } 
            if (cuboid_received_) {
                //markConnectedCells(start_unknown_point, octree);
            }
            

            if (sensor_pose_.poses.size() > 0) {
                // remove all the seen cells
                rayCast(octree);
            }
            octomap_msgs::Octomap bmap_msg;
            octomap_msgs::fullMapToMsg(*octree, bmap_msg);

            octomap_msgs::Octomap recon_msg;
            octomap_msgs::fullMapToMsg(target_reconstruction_, recon_msg);
            
            bmap_msg.header.stamp = ros::Time::now();
            bmap_msg.header.frame_id = "map";
            recon_msg.header.stamp = ros::Time::now();
            recon_msg.header.frame_id = "map";
            ROS_INFO("New map published!");
            newmap_pub_.publish(bmap_msg);    
            reconstruction_pub_.publish(recon_msg);
            
            delete octree; 
            delete octree_temp_;
        } else {
            ROS_INFO("no octree?");
        }
    }

    void markConnectedCells(octomap::point3d start_point, octomap::ColorOcTree* octree) {
            std::vector<octomap::point3d> directions = {
                {0.05, 0, 0}, {-0.05, 0, 0},
                {0, 0.05, 0}, {0, -0.05, 0},
                {0, 0, 0.05}, {0, 0, -0.05}
            };

            std::stack<octomap::point3d> toVisit;
            toVisit.push(start_point);

            while (!toVisit.empty()) {
                octomap::point3d current = toVisit.top();
                toVisit.pop();

                // Check if the node exists and meets the criteria
                octomap::ColorOcTreeNode* node = octree->search(current);
                if (node && node->getLogOdds() >= 0.0 && node->getLogOdds() < 0.1) {
                    // Mark the node (example: set a custom value)
                    node->setColor(255, 255, 237); // Assuming your OctoNode has this method
                    node->setValue(OCCLUSION);

                    // Explore neighbors
                    for (const auto& dir : directions) {
                        octomap::point3d neighbor = current + dir; // Get neighbor point
                        toVisit.push(neighbor);
                    }
                }
            }
    }

    void rayCast(octomap::ColorOcTree* octree) {
        int seenCount(0);
        octomap::point3d end; 

        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        pose_array.header.stamp = ros::Time::now();

        geometry_msgs::Pose sensor_pose_origin;
        size_t size = sensor_pose_.poses.size();
        if (size < 2) {
            //ROS_ERROR("Incorrect rays received %li", size);
            return;
        }
        sensor_pose_origin = sensor_pose_.poses[0]; // last element is the sensor origin
        
        for (size_t i = 1; i < size; i++) {
            octomap::point3d ray_origin(sensor_pose_origin.position.x, sensor_pose_origin.position.y, sensor_pose_origin.position.z);
            geometry_msgs::Pose dir_pose = sensor_pose_.poses[i];
            octomap::point3d ray_direction(dir_pose.position.x, dir_pose.position.y, dir_pose.position.z);

            if (octree->castRay(ray_origin, ray_direction, end, true, 8)) {
                octomap::ColorOcTreeNode* node = octree->search(end);
                if (node && (isClose(node->getValue(),POTENTIAL_OBJECT) || isClose(node->getValue(),OCCLUSION))) {
                    target_reconstruction_.setNodeValue(end, static_cast<float>(0.66));
                    node->setValue(SEEN);
                    node->setColor(10,255,10);
                    seenCount++;
                }
                geometry_msgs::Pose pose_end;
                pose_end.position.x = end.x();
                pose_end.position.y = end.y();
                pose_end.position.z = end.z();
                pose_end.orientation.w = 1.0;
                pose_array.poses.push_back(pose_end);
            }
        }

        ROS_INFO("end position is %i", seenCount);
        sensor_pose_.poses = std::vector<geometry_msgs::Pose>();
        debug_pub_.publish(pose_array);
    }


    void sensorPoseCallback(const geometry_msgs::PoseArray sensor_poses) {
        sensor_pose_ = sensor_poses;
        size_t size = sensor_pose_.poses.size();
        ROS_INFO("%li sensor poses received", size);
        return;
    }

    void cornerCallback(const geometry_msgs::PoseArray contents) {
        corner_msg_.header = contents.header;
        corner_msg_.poses = contents.poses;
    }

    void mapCornerCallback(const geometry_msgs::PoseArray contents) {
        map_corner_msg_.header = contents.header;
        map_corner_msg_.poses = contents.poses;
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

    void flagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        reconstruction_finished_ = true;
    }
    

    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber corner_sub_;
    ros::Subscriber map_corner_sub_;
    ros::Subscriber sensor_pose_sub_;
    ros::Subscriber recon_completion_sub_;
    ros::Publisher newmap_pub_;
    ros::Publisher reconstruction_pub_;
    ros::Publisher debug_pub_;
    ros::Publisher count_pub_;
    bool cuboid_received_;
    bool reconstruction_finished_;
    bool valid_cuboid_;
    geometry_msgs::PoseArray corner_msg_;
    geometry_msgs::PoseArray map_corner_msg_;
    geometry_msgs::PoseArray sensor_pose_;
    float max_distance_;
    octomap::point3d start_voxel_;

    

};

// Entry point of the ROS node
int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map_node");

    // Create an instance of the RayCastingNode class
    MapNode node;

    // Enter the ROS event loop
    //ros::spin();

    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
