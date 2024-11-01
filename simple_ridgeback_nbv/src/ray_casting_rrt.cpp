#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ray_casting.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <Eigen/Dense>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>


#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<time.h>
#include<iostream>
#include<stdlib.h>

#include <thread>
#include <atomic>

#include "rrt.h"


#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

static constexpr int8_t MINMIN = 0;
static constexpr int8_t MINMAX = 1;
static constexpr int8_t MAXMAX = 2;
static constexpr int8_t MAXMIN = 3;
static constexpr int8_t MAXZ = 4;


#define MAX(a,b) ((a > b ? a : b))
#define MIN(a,b) ((a > b ? b : a))

namespace ros {
namespace message_traits {

template<>
struct TimeStamp<visualization_msgs::MarkerArray> {
  static ros::Time value(const visualization_msgs::MarkerArray& msg) {
    if (msg.markers.size() != 0) {
        return msg.markers[0].header.stamp;
    } else {
        return ros::Time::now();
    }
  }
};

}  // namespace message_traits
}  // namespace ros

bool isPointInBounds(const octomap::point3d& point, const Vec3 minBound, const Vec3 maxBound) {
    // Get the bounds of the octree
    //ROS_INFO("Point %f, %f, %f", point.x(), point.y(), point.z());
    //ROS_INFO("Octree Bounds %f, %f, %f", minBound.x, minBound.y, minBound.z);
    //ROS_INFO("Octree Bounds %f, %f, %f", maxBound.x, maxBound.y, maxBound.z);
    
    // Check if the point is within the bounds
    if (point.x() >= minBound.x && point.x() <= maxBound.x &&
        point.y() >= minBound.y && point.y() <= maxBound.y &&
        point.z() >= minBound.z && point.z() <= maxBound.z) {
        return true;  // Point is within bounds
    } else {
        return false; // Point is outside bounds
    }
}

bool checkLinearCollision(octomap::OcTree* octree, geometry_msgs::Point p1, geometry_msgs::Point p2, float height) {
    octomap::point3d origin(p1.x, p1.y, 0);
    octomap::point3d end_2d(p2.x, p2.y, 0);
    octomap::point3d end_3d(p2.x, p2.y, height);

    std::vector<octomap::point3d> ray_2d;
    std::vector<octomap::point3d> ray_3d;

    // check the 3d linear collision to the desired height
    octree->computeRay(origin, end_2d, ray_2d);
    octree->computeRay(origin, end_3d, ray_3d);

    for (auto& pt : ray_2d) {
        auto node = octree->search(pt);
        if (node && octree->isNodeOccupied(node)) {
            return true;
        }
    }

    for (auto& pt : ray_3d) {
        auto node = octree->search(pt);
        if (node && octree->isNodeOccupied(node)) {
            return true;
        }
    }

    return false;
}

bool checkConservativeCollision(octomap::OcTree* octree, float x, float y) {
    // Convert the point to octomap point3D
    octomap::point3d origin(x, y, 0);
    int num_angles = 10;
    int num_elevations = 20;
    float radius = 1.0;

    // Define the angle increments
    float angle_increment = 2.0 * M_PI / num_angles;         // Horizontal angle increment
    float elevation_increment = M_PI / num_elevations;       // Vertical angle increment

    // Loop through different horizontal angles
    for (int i = 0; i < num_angles; ++i) {
        float angle = i * angle_increment;

        // Loop through different elevation angles
        for (int j = 0; j < num_elevations; ++j) {
            float elevation = j * elevation_increment - M_PI / 2.0; // Center the dome

            // Calculate the direction of the ray
            float direction_x = std::cos(angle) * std::cos(elevation);
            float direction_y = std::sin(angle) * std::cos(elevation);
            float direction_z = std::sin(elevation);

            float magnitude = std::sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);

            // Avoid division by zero in case the vector is zero
            if (magnitude > 0) {
                direction_x /= magnitude;
                direction_y /= magnitude;
                direction_z /= magnitude;
            }
            octomap::point3d direction(direction_x, direction_y, direction_z);
            octomap::point3d end;

            
            // Cast the ray from the origin in the calculated direction
            if (octree->castRay(origin, direction, end, true, radius)) {
                // Check if the ray hits an occupied voxel
                octomap::OcTreeNode* node = octree->search(end);
                if (node && (octree->isNodeOccupied(node) || (node->getOccupancy() < 0.5001 && node->getOccupancy() >= 0.5))) {
                    return true;
                }
            }
        }
    }

    // If no collision detected
    return false;
}

class RayCastingNode
{
public:
    RayCastingNode()
        : nh(),
        pub_rcgrid(nh.advertise<nav_msgs::OccupancyGrid>("/cast_map", 10)),
        sub_ogrid(nh.subscribe("/projected_map", 10, &RayCastingNode::ogrid_callback, this)),
        timer(nh.createTimer(ros::Duration(0.1), &RayCastingNode::main_loop, this)),
        pub_bounding_box(nh.advertise<visualization_msgs::Marker>("/cuboid_marker", 10)),
        listener(buffer),
        robot_pose(0.0,0.0,0.0),
        robot_heading(0.0,0.0,0.0),
        backup_plan_cuboid(0.0, 0.0, 0.0),
        pub_debug(nh.advertise<geometry_msgs::PoseArray>("/debug_marker_ray", 100)),
        pub_best_pose(nh.advertise<visualization_msgs::Marker>("/best_pose", 10)),
        sub_octomap(nh.subscribe("/octomap_full", 10, &RayCastingNode::normal_callback, this)),
        //cuboid_corner_sub(nh.subscribe("/match_cuboid_corners", 10, &RayCastingNode::corner_callback, this)),
        grid_received(false),
        explorationed_started(false),
        exploration_finished(false),
        pub_move_base_goal(nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1)),
        pub_tree(nh.advertise<visualization_msgs::MarkerArray>("/rrt_tree", 10)),
        pub_rrt_sample_debug(nh.advertise<visualization_msgs::MarkerArray>("/rrt_debug_sample", 10)),
        retry(0),
        rrt_tree(),
        action_client("move_base", true),
        pub_cmd_vel(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10))
    {
        odom_sub.subscribe(nh, "/odom", 10);
        map_sub.subscribe(nh, "/projected_map", 10);

        search_area_sub.subscribe(nh, "/search_area", 10);
        octo_sub.subscribe(nh, "/unknown_octree", 10);

        using sync_pol_map = message_filters::sync_policies::ApproximateTime<visualization_msgs::MarkerArray, octomap_msgs::Octomap>;
        sync_map.reset(new message_filters::Synchronizer<sync_pol_map>(sync_pol_map(2000), search_area_sub, octo_sub));
        sync_map->registerCallback(boost::bind(&RayCastingNode::cuboid_callback, this, _1, _2));
        // Define the sync policy and create the synchronizer

        using sync_pol = message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>;
        sync.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(500), odom_sub, map_sub));
        sync->registerCallback(boost::bind(&RayCastingNode::callback, this, _1, _2));


        ROS_INFO("Waiting for move_base action server to start.");
        action_client.waitForServer(); // Waits for the action server to come up

    }

    /** 
    void corner_callback(const geometry_msgs::PoseArray contents) {
        corner_msg.header = contents.header;
        corner_msg.poses = contents.poses;
    }*/


    float computeNormalGain() {

    }

    float computeTargetedGain(float x, float y, float rec_x, float rec_y, octomap::OcTree* octree, float FOV, int FOV_resolution, float camera_height,
                    Vec3 minBound, Vec3 maxBound) {
        float dy = rec_y - y;
        float dx = rec_x - x;  
        float mag = std::sqrt(dy * dy + dx * dx);
        float dy_norm = (mag != 0.0) ? (dy / mag) : 0.0;
        float dx_norm = (mag != 0.0) ? (dx / mag) : 0.0;
        Vec3 vec_dir(dx_norm, dy_norm, 0.0);

        octomap::point3d origin(x, y, camera_height);
        octomap::point3d direction(dx_norm, dy_norm, 0.0);
        octomap::point3d end;  
        
        float FOV_step = FOV / FOV_resolution;
        float gain(0.0);
        // =========================================================== Looping through FOV to add information gain
        for (int fov_idx = 0; fov_idx < FOV_resolution * FOV_resolution; fov_idx++) {
            
            int i = fov_idx % FOV_resolution; 
            int j = fov_idx / FOV_resolution;  

            float h_angle = -FOV / 2 + i * FOV_step;
            float v_angle = -FOV / 2 + j * FOV_step;

            Vec3 new_vec = Vec3(direction.x(), direction.y(), direction.z()).normalize()                       
                                .rotateZ(h_angle)
                                .rotateY(v_angle);

            octomap::point3d new_direction = octomap::point3d(new_vec.x, new_vec.y, new_vec.z);
            if (new_direction.x() == 0.0 &&  new_direction.y() == 0.0 && new_direction.z() == 0.0) {
                continue;
            }
            if (octree->castRay(origin, new_direction, end, true, 10)) {
                
                /** Use this for debugging hits
                if (x < rec_min_x + 0.5 && y < rec_max_y - 0.5) {
                    geometry_msgs::Pose pose;
                    pose.position.x = end.x();
                    pose.position.y = end.y();
                    pose.position.z = end.z();
                    pose.orientation.w = 1.0;        
                    pose_array.poses.push_back(pose);
                } **/

                if (isPointInBounds(end, minBound, maxBound)) {
                    octomap::OcTreeNode* node = octree->search(end);
                    if (node->getOccupancy() >= 0.5 && node->getOccupancy() < 0.51) {
                        //occlusion
                        gain += 10;
                    }

                    if (node->getOccupancy() >= 0.58 && node->getOccupancy() < 0.59) {
                        //potential object
                        gain += 0.01;
                    }


                }
            }
        } // FOV loop ends
        return gain;
    } 

    void normal_callback(const octomap_msgs::Octomap::ConstPtr& map_msg) {
        if (explorationed_started || exploration_finished) {
            return;
        }

        // wait for odom to update --
        if (robot_heading.x == 0.0 && robot_heading.y == 0.0) {
            ROS_INFO("waiting for odom to update..");
            return;
        } 

        if (!grid_received) {
            ROS_INFO("waiting for projected map to generate");
            return;
        }

        octomap::AbstractOcTree* octo_abs_tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octo_abs_tree);
        
        // grid 2d must be set if start_rrt returns true
        float map_max_x = grid_2d.info.origin.position.x + grid_2d.info.width * grid_2d.info.resolution;
        float map_max_y = grid_2d.info.origin.position.y + grid_2d.info.height * grid_2d.info.resolution;
        float map_min_x = grid_2d.info.origin.position.x;
        float map_min_y = grid_2d.info.origin.position.y;

        geometry_msgs::Point start_rrt_pt;
        start_rrt_pt.x = robot_pose.x;
        start_rrt_pt.y = robot_pose.y;
        start_rrt_pt.z = 0.0;

        geometry_msgs::Point goal_rrt_pt;
        goal_rrt_pt.x = map_max_x;
        goal_rrt_pt.y = map_max_y;
        goal_rrt_pt.z = 0.0;

        int num_sample(0);
        int max_sample(100);
        
        float distance(0.0);
        int best_node_index(-1);

        // begin the rrt tree
        if (start_rrt(start_rrt_pt, goal_rrt_pt)) {
            while (max_sample > num_sample) {
                auto V_rand = rrt_tree.randomSample(map_min_x, map_max_x, map_min_y, map_max_y);
                if (V_rand.x < map_min_x || V_rand.x > map_max_x || V_rand.y < map_min_y || V_rand.y > map_max_y) {
                    continue;
                }

                int grid_index = grid_2d.info.width*std::floor((V_rand.y - grid_2d.info.origin.position.y)/grid_2d.info.resolution) + std::floor((V_rand.x - grid_2d.info.origin.position.x)/grid_2d.info.resolution);
                if (grid_index < grid_2d.info.width * grid_2d.info.height) {
                    if (grid_2d.data[grid_index] == -1 || grid_2d.data[grid_index] > 90) {
                        continue;
                    }
                }
            
                // check if it is in free space and collision free from the closest neighbour
                int nearest_index = rrt_tree.nearestNode(V_rand);
                if (!checkLinearCollision(octree, rrt_tree.returnTree()[nearest_index].position, V_rand, 4)) {
                    auto new_node = rrt_tree.extendTree(nearest_index,V_rand,0.0);
                    if (checkConservativeCollision(octree, new_node.position.x, new_node.position.y)) {
                        // sometimes steering command requires additional collision checker
                        continue;
                    }
                    rrt_tree.addNode(new_node);
                    num_sample++;
                    
                    float temp_distance = std::sqrt((new_node.position.x - robot_pose.x)*(new_node.position.x - robot_pose.x) 
                                                    + (new_node.position.y - robot_pose.y)*(new_node.position.y - robot_pose.y));
                    if (temp_distance > distance) {
                        distance = temp_distance;
                        best_node_index = rrt_tree.returnTree().size() - 1;
                    }
                } else {
                    ROS_INFO("collision! sample again");
                }
            }

            rrt_tree.visualizeTree();

            // Iterate through the tree and generate a path
            std::deque <int> path;
            double velocity = 0.1;

            if (best_node_index != -1) {

                int current_idx = best_node_index;
                path.push_back(current_idx);
                auto rrt_tree_ = rrt_tree.returnTree();

                while (rrt_tree_[current_idx].parent != -1) {
                    ROS_INFO("Current node's parent index: %i", rrt_tree_[current_idx].parent);
                    
                    // Check for valid parent index
                    if (current_idx >= 0 && current_idx < rrt_tree.returnTree().size()) {
                        // Move to the parent node
                        ROS_INFO("adding to the path: %i", rrt_tree_[current_idx].parent);
                        path.push_front(rrt_tree_[current_idx].parent);  // Store pointer to the parent node
                        current_idx = rrt_tree_[current_idx].parent;
                    } else {
                        ROS_ERROR("Invalid parent index: %i", current_idx);
                        break; // Exit the loop
                    }

                    // Debugging: Check if the parent index has changed
                    ROS_INFO("Next node's parent index: %i", current_idx);
                }

                geometry_msgs::Point current_rrt = start_rrt_pt;
                int horizon(0);
                for (auto path_node : path) {
                    ROS_INFO("next path!");
                    double direction_to_x = rrt_tree_[path_node].position.x - current_rrt.x;
                    double direction_to_y = rrt_tree_[path_node].position.y - current_rrt.y;
                    double magnitude = std::sqrt(direction_to_x * direction_to_x + direction_to_y * direction_to_y);
                    if (magnitude > 0.0) {
                        direction_to_x /= magnitude;
                        direction_to_y /= magnitude;
                    }
                    ROS_INFO("distance %f", magnitude);
                    send_velocity(magnitude, velocity, direction_to_x, direction_to_y);
                    current_rrt = rrt_tree_[path_node].position;
                    horizon++;
                    if (horizon > 2) {
                        break;
                    }
                }
            
            }


        }



    }



    void cuboid_callback(const visualization_msgs::MarkerArray::ConstPtr& search_area_msg, const octomap_msgs::Octomap::ConstPtr& map_msg) {
        if (exploration_finished) {
            ROS_INFO("Exploration has finished");
            return;
        } else {
            ROS_INFO("Starting!");
        }
        
        // wait for odom to update --
        if (robot_heading.x == 0.0 && robot_heading.y == 0.0) {
            ROS_INFO("waiting for odom to update..");
            return;
        } 

        if (!explorationed_started) {
            // first time save the coordinates.
            start_pose.position.x = robot_pose.x;
            start_pose.position.y = robot_pose.y;
            start_pose.position.z = robot_pose.z;

            double yaw = atan2(robot_heading.y, robot_heading.x); // Calculate yaw based on direction
            if (std::isnan(yaw)) {
                yaw = 0.0; // Set a default value if yaw is NaN
            }

            // Create quaternion from roll, pitch, and yaw
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            q.normalize(); // Normalize the quaternion
            start_pose.orientation.x = q.x();
            start_pose.orientation.y = q.y();
            start_pose.orientation.z = q.z();
            start_pose.orientation.w = q.w();

            explorationed_started = true;
        } 
        // calculate the 2D bounding rectangle

        double rec_min_x = search_area_msg->markers[0].points[0].x;
        double rec_max_x = search_area_msg->markers[0].points[2].x;
        double rec_min_y = search_area_msg->markers[0].points[0].y;
        double rec_max_y = search_area_msg->markers[0].points[2].y;

        ROS_INFO("min x : %f, min y: %f, max x: %f, max y: %f", rec_min_x, rec_min_y, rec_max_x, rec_max_y);


        double rec_centre_x = rec_min_x + (rec_max_x - rec_min_x) / 2;
        double rec_centre_y = rec_min_y + (rec_max_y - rec_min_y) / 2;

        octomap::AbstractOcTree* octo_abs_tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octo_abs_tree);
        double resolution = octree->getResolution();

        Vec3 minBound(rec_min_x, rec_min_y, 0.0);
        Vec3 maxBound(rec_max_x, rec_max_y, 10.0);

        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "map";
         
        if (octree) // can be NULL
        {

            // =========================================================== Decretisiation of FOV
            float FOV = 20 * 0.0174533f; // degree * 1 rad/degree
            int FOV_resolution = 40;
            float FOV_step = FOV / FOV_resolution;
            float camera_height = 0.29; // [m]

            // =========================================================== Start of RRT
            geometry_msgs::Point start_rrt_pt;
            start_rrt_pt.x = robot_pose.x;
            start_rrt_pt.y = robot_pose.y;
            start_rrt_pt.z = 0.0;

            geometry_msgs::Point goal_rrt_pt;
            goal_rrt_pt.x = rec_centre_x;
            goal_rrt_pt.y = rec_centre_y;
            goal_rrt_pt.z = 0.0;

            int best_node_index(0);
            double best_info_gain(0);
            float best_x(0.0);
            float best_y(0.0);
            float best_dx(0.0);
            float best_dy(0.0);

            float collision_check_height = 2;
            int max_sample(50);
            int num_sample(0);

            visualization_msgs::MarkerArray rrt_sample_debug_markers;

            // =========================================================== RRT expansion
            if (start_rrt(start_rrt_pt, goal_rrt_pt)) {
                ROS_INFO("double checking.. %i", rrt_tree.returnTree()[0].parent);
                // sample near the robot 
                while (max_sample > num_sample) {
                    auto V_rand = rrt_tree.randomSample(rec_min_x, rec_max_x, rec_min_y, rec_max_y);
                    if (V_rand.x < rec_min_x || V_rand.x > rec_max_x || V_rand.y < rec_min_y || V_rand.y > rec_max_y) {
                        continue;
                    }
               
                    // check if it is in free space and collision free from the closest neighbour
                    int nearest_index = rrt_tree.nearestNode(V_rand);
                    if (!checkLinearCollision(octree, rrt_tree.returnTree()[nearest_index].position, V_rand, 4)) {
                        // calculate information gain and extend the tree
                        float gain = computeTargetedGain(V_rand.x, V_rand.y, rec_centre_x, rec_centre_y, octree, FOV, FOV_resolution, camera_height,
                        minBound, maxBound);
                        auto new_node = rrt_tree.extendTree(nearest_index,V_rand,gain);
                        if (checkConservativeCollision(octree, new_node.position.x, new_node.position.y)) {
                            // sometimes steering command requires additional collision checker
                            continue;
                        }
                        rrt_tree.addNode(new_node);
                        num_sample++;
                    } else {
                        ROS_INFO("collision! sample again");
                    }
                }

            rrt_tree.visualizeTree();

            // =========================================================== Need to execute the best branch with the gain.. 
                for (int i = 0; i < rrt_tree.returnTree().size(); i++) {
                    auto tree = rrt_tree.returnTree();
                    float dy = rec_centre_y - tree[i].position.y;
                    float dx = rec_centre_x - tree[i].position.x;  

                    float mag = std::sqrt(dy * dy + dx * dx);
                    float dy_norm = (mag != 0.0) ? (dy / mag) : 0.0;
                    float dx_norm = (mag != 0.0) ? (dx / mag) : 0.0;
                    Vec3 vec_dir(dx_norm, dy_norm, 0.0);

                    octomap::point3d origin(tree[i].position.x, tree[i].position.y, camera_height);
                    octomap::point3d direction(dx_norm, dy_norm, 0.0);
                    octomap::point3d end;  

                    if (tree[i].gain > best_info_gain && !checkConservativeCollision(octree, tree[i].position.x, tree[i].position.y)) {
                        best_info_gain = tree[i].gain;
                        best_node_index = i;
                        best_x = tree[i].position.x;
                        best_y = tree[i].position.y;
                        best_dy = dy;
                        best_dx = dx;
                        ROS_INFO("Best info gain for this iteration %f with an index of %i and parent of %i", best_info_gain, best_node_index, tree[best_node_index].parent);

                    } 
                }
                
            } else {
                ROS_INFO("Waiting for the grid to Occupancy grid update for RRT..");
                return;
            }

            // =========================================================== Terminating condition
            
            if (best_info_gain < 50) {
                retry++;
                if (retry < 1) {

                    // move to observe various sides of the rectangle.
                    float r = std::sqrt((robot_pose.x - rec_centre_x) * (robot_pose.x - rec_centre_x) + (robot_pose.y - rec_centre_y) * (robot_pose.y - rec_centre_y));
                    float step_radian = M_PI / 6; // 30 degrees = PI/6 radians
                    geometry_msgs::Pose backup_pose;

                    // Loop through angles from 0 to 2*PI (360 degrees)
                    for (int i = 0; i < 6; ++i)
                    {
                        // Calculate the current angle
                        float angle = i * step_radian;

                        // Calculate the position on the circle
                        float x = rec_centre_x + (r) * std::cos(angle);
                        float y = rec_centre_y + (r) * std::sin(angle);

                        if (!checkConservativeCollision(octree, x, y)) {
                            backup_pose.position.x = x;
                            backup_pose.position.y = y;
                            double yaw = atan2(rec_centre_y - y, rec_centre_x - x); // Calculate yaw based on direction
                            if (std::isnan(yaw)) {
                                yaw = 0.0; // Set a default value if yaw is NaN
                            }

                            // Create quaternion from roll, pitch, and yaw
                            tf2::Quaternion q;
                            q.setRPY(0.0, 0.0, yaw);
                            q.normalize(); // Normalize the quaternion
                            
                            backup_pose.orientation.x = q.x();
                            backup_pose.orientation.y = q.y();
                            backup_pose.orientation.z = q.z();
                            backup_pose.orientation.w = q.w();        
                            break;
                        }
                    }         

                    if (backup_pose.orientation.w != 0.0) {
                        ROS_INFO("Exploring the cuboid...");
                        move_base_msgs::MoveBaseGoal goal;
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = backup_pose;
                        action_client.sendGoal(goal);
                        bool success = action_client.waitForResult(ros::Duration(60.0)); // Waits for the result with a timeout of 30 seconds
                        retry++;
                        return;
                    } else {
                        retry++;
                        ROS_ERROR("can't explore without collision, tune the parameters again for clutter");
                    }            
                    
                } else {
                    ROS_INFO("Completed the scan, going to where I started");
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = start_pose;
                    action_client.sendGoal(goal);
                    bool success = action_client.waitForResult(ros::Duration(60.0)); // Waits for the result with a timeout of 30 seconds
                    exploration_finished = true;
                    return;
                }
                
            } 
            // add to pose_array to visualise in debug_marker_ray outside the loop
            pub_debug.publish(pose_array);

            // =========================================================== Publishing the best viewpoint
            if (best_dx != 0.0 && best_dy != 0.0) {
                
                visualization_msgs::Marker best;
                best.header.frame_id = "map";
                best.header.stamp = ros::Time::now();
                best.ns = "best";
                best.id = 78;
                best.type = visualization_msgs::Marker::SPHERE;
                best.action = visualization_msgs::Marker::ADD;

                best.pose.position.x = best_x;
                best.pose.position.y = best_y;
                best.pose.position.z = 0.29;
                best.pose.orientation.w = 1.0;
                best.scale.x = 0.2;
                best.scale.y = 0.2;
                best.scale.z = 0.2;

                best.color.r = 1.0;
                best.color.g = 0.0;
                best.color.b = 0.0;
                best.color.a = 0.5; // Semi-transparent

                best.lifetime = ros::Duration();
                // std::cout << best_info_gain << std::endl;
                pub_best_pose.publish(best);

                double yaw = atan2(best_dy, best_dx); // Calculate yaw based on direction
                if (std::isnan(yaw)) {
                    yaw = 0.0; // Set a default value if yaw is NaN
                }

                // Create quaternion from roll, pitch, and yaw
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw);
                q.normalize(); // Normalize the quaternion
                

                geometry_msgs::Pose pose;
                pose.position.x = best_x;
                pose.position.y = best_y;
                pose.position.z = 0.0;
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                pose.orientation.w = q.w();    
            

                 
                // Iterate through the tree and generate a path
                std::deque <int> path;
                rrt_tree.printTree(); 

                double velocity = 0.1;

                int current_idx = best_node_index;
                path.push_back(current_idx);
                auto rrt_tree_ = rrt_tree.returnTree();

                while (rrt_tree_[current_idx].parent != -1) {
                    ROS_INFO("Current node's parent index: %i", rrt_tree_[current_idx].parent);
                    
                    // Check for valid parent index
                    if (current_idx >= 0 && current_idx < rrt_tree.returnTree().size()) {
                        // Move to the parent node
                        ROS_INFO("adding to the path: %i", rrt_tree_[current_idx].parent);
                        path.push_front(rrt_tree_[current_idx].parent);  // Store pointer to the parent node
                        current_idx = rrt_tree_[current_idx].parent;
                    } else {
                        ROS_ERROR("Invalid parent index: %i", current_idx);
                        break; // Exit the loop
                    }

                    // Debugging: Check if the parent index has changed
                    ROS_INFO("Next node's parent index: %i", current_idx);
                }

                geometry_msgs::Point current_rrt = start_rrt_pt;
                for (auto path_node : path) {
                    ROS_INFO("next path!");
                    double direction_to_x = rrt_tree_[path_node].position.x - current_rrt.x;
                    double direction_to_y = rrt_tree_[path_node].position.y - current_rrt.y;
                    double magnitude = std::sqrt(direction_to_x * direction_to_x + direction_to_y * direction_to_y);
                    if (magnitude > 0.0) {
                        direction_to_x /= magnitude;
                        direction_to_y /= magnitude;
                    }
                    ROS_INFO("distance %f", magnitude);
                    send_velocity(magnitude, velocity, direction_to_x, direction_to_y);
                    current_rrt = rrt_tree_[path_node].position;
                }
                


                /**
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = pose;
                action_client.sendGoal(goal);
                bool success = action_client.waitForResult(ros::Duration(60.0));  // Waits for the result with a timeout of 30 seconds 
                */
            }

        }
    }

    void send_velocity(double distance, double velocity, double dir_x, double dir_y) {
        double duration = distance / velocity;
        ROS_INFO("duration for %f", duration);
        ros::Time start_time = ros::Time::now();
        ROS_INFO("Start Time: %f", start_time.toSec());
        ROS_INFO("End Time: %f", (start_time + ros::Duration(duration)).toSec());
        ros::Rate rate(10);
        geometry_msgs::Twist command_vel;
        command_vel.linear.x = dir_x * velocity;
        command_vel.linear.y = dir_y * velocity; 
        
        while (ros::Time::now() - start_time < ros::Duration(duration)) {
            ROS_INFO("current %f", (ros::Time::now()).toSec());
            pub_cmd_vel.publish(command_vel);
            rate.sleep();
        }

        command_vel.linear.x = 0;
        command_vel.linear.y = 0;
        pub_cmd_vel.publish(command_vel);

    }

    bool start_rrt(geometry_msgs::Point& start, geometry_msgs::Point& goal) {
        float rrt_step_size = 1.0; 

        if (grid_received) {
            rrt_tree.setGrid(grid_2d);
            rrt_tree.setPub(pub_tree);
            rrt_tree.setPoints(rrt_step_size, start, goal);
            rrt_tree.init();
            //rrt_tree.addGain(0,0.0); 
            return true;
        }
        
        return false;
    }

    

    void box_callback(const geometry_msgs::PoseArray::ConstPtr& msg, const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    }

    

    void callback(const nav_msgs::Odometry::ConstPtr& odom_msg, const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        //ROS_INFO("Updating odom");

        geometry_msgs::TransformStamped odom_map_transform = buffer.lookupTransform("map","odom", ros::Time(0));
        geometry_msgs::PoseStamped ros_pos_odom;
        ros_pos_odom.header = odom_msg->header;
        ros_pos_odom.pose = odom_msg->pose.pose;
        geometry_msgs::PoseStamped ros_pos_map;
        tf2::doTransform(ros_pos_odom, ros_pos_map, odom_map_transform);
    
        float robot_x = ros_pos_map.pose.position.x;
        float robot_y = ros_pos_map.pose.position.y;
        float map_origin_x = map_msg->info.origin.position.x;
        float map_origin_y = map_msg->info.origin.position.y;
        float res = map_msg->info.resolution;

        robot_pose.x = robot_x;
        robot_pose.y = robot_y;
        robot_pose.z = 0.0;


        const geometry_msgs::Quaternion& quat = ros_pos_map.pose.orientation;    
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll,pitch,yaw);
        Vec2 direction(cos(yaw), sin(yaw)); // unit vector
        //ROS_INFO("headings %f, %f", direction.x, direction.y);
        
        robot_heading.x = direction.x;
        robot_heading.y = direction.y;
        robot_heading.z = 0.0;

    }

    void ogrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& ogrid)
    {
        grid_2d = *ogrid;
        grid_received = true;
    }

    bool bound_check(int32_t location, int32_t width, int32_t height) {
        return location >= 0 && location < width * height;
    }

    void main_loop(const ros::TimerEvent&) const
    {
    }

private:
    ros::NodeHandle nh;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub;
    message_filters::Subscriber<visualization_msgs::MarkerArray> search_area_sub;
    message_filters::Subscriber<octomap_msgs::Octomap> octo_sub;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>>> sync;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<visualization_msgs::MarkerArray, octomap_msgs::Octomap>>> sync_map;
    ros::Publisher pub_rcgrid;
    ros::Publisher pub_unknown_octree;
    ros::Publisher pub_bounding_box;
    ros::Subscriber sub_ogrid;
    ros::Subscriber sub_octomap;
    ros::Publisher pub_debug;
    ros::Publisher pub_rectangle;
    ros::Publisher pub_best_pose;
    ros::Timer timer;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, nav_msgs::OccupancyGrid>>> rectangle_sync;
    //boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, octomap_msgs::Octomap>>> match_sync;
    Vec3 robot_pose;
    Vec3 robot_heading;
    Vec3 backup_plan_cuboid;
    //geometry_msgs::PoseArray corner_msg;
    //ros::Subscriber cuboid_corner_sub;
    ros::Publisher pub_tree;
    nav_msgs::OccupancyGrid grid_2d;
    bool grid_received;
    bool explorationed_started;
    bool exploration_finished;
    ros::Publisher pub_move_base_goal;
    int retry;
    geometry_msgs::Pose start_pose;
    MoveBaseClient action_client;
    RRT::Tree rrt_tree;
    ros::Publisher pub_rrt_sample_debug;
    ros::Publisher pub_cmd_vel;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_casting_node");
    RayCastingNode node;
    ros::spin();
    return 0;
}
