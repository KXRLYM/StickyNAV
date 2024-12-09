#ifndef NBV_RAY_H
#define NBV_RAY_H

#include "ray.h"

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <time.h>
#include <iostream>
#include <stdlib.h>

#include <thread>
#include <atomic>

#include "kd_rrt.h"
#include "octomapper.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/callback_queue.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

static constexpr int8_t MINMIN = 0;
static constexpr int8_t MINMAX = 1;
static constexpr int8_t MAXMAX = 2;
static constexpr int8_t MAXMIN = 3;
static constexpr int8_t MAXZ = 4;

static constexpr int MAX_HARDWARE_HEIGHT = 1200; // [mm]
static constexpr int GLOBAL_OCCUPANCY_COST_MAX = 10;

#define MAX(a, b) ((a > b ? a : b))
#define MIN(a, b) ((a > b ? b : a))

using KDNodePtr = std::shared_ptr<KD_RRT::KDNode>;

/** Assigning a timestamp on a visualization marker so that it can be used for message filters for synchornizing callbacks */
namespace ros
{
    namespace message_traits
    {

        template <>
        struct TimeStamp<visualization_msgs::MarkerArray>
        {
            static ros::Time value(const visualization_msgs::MarkerArray &msg)
            {
                if (msg.markers.size() != 0)
                {
                    return msg.markers[0].header.stamp;
                }
                else
                {
                    return ros::Time::now();
                }
            }
        };

    } // namespace message_traits
} // namespace ros

void set_quat(tf2::Quaternion &quat, geometry_msgs::Pose &pose)
{
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return;
}

bool isPointInBounds(const octomap::point3d &point, const Vec3 minBound, const Vec3 maxBound)
{
    // Get the bounds of the octree
    // ROS_INFO("Point %f, %f, %f", point.x(), point.y(), point.z());
    // ROS_INFO("Octree Bounds %f, %f, %f", minBound.x, minBound.y, minBound.z);
    // ROS_INFO("Octree Bounds %f, %f, %f", maxBound.x, maxBound.y, maxBound.z);

    // Check if the point is within the bounds
    if (point.x() >= minBound.x && point.x() <= maxBound.x &&
        point.y() >= minBound.y && point.y() <= maxBound.y &&
        point.z() >= minBound.z && point.z() <= maxBound.z)
    {
        return true; // Point is within bounds
    }
    else
    {
        return false; // Point is outside bounds
    }
}

bool checkPathCollision(nav_msgs::OccupancyGrid *costmap, geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    int width = costmap->info.width;
    int height = costmap->info.height;
    float resolution = costmap->info.resolution;

    int x1 = static_cast<int>((p1.x - costmap->info.origin.position.x) / resolution);
    int y1 = static_cast<int>((p1.y - costmap->info.origin.position.y) / resolution);
    int x2 = static_cast<int>((p2.x - costmap->info.origin.position.x) / resolution);
    int y2 = static_cast<int>((p2.y - costmap->info.origin.position.y) / resolution);

    // Check bounds
    if (x1 < 0 || x1 >= width || y1 < 0 || y1 >= height ||
        x2 < 0 || x2 >= width || y2 < 0 || y2 >= height)
    {
        return true; // Out of bounds, return true anyway to prevent path formation
    }

    // Bresenham's line algorithm to get the cells in the path
    std::vector<std::pair<int, int>> cells;
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        cells.push_back({x1, y1});

        if (x1 == x2 && y1 == y2)
            break;

        int err2 = err * 2;
        if (err2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if (err2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }

    // Check costs
    int totalCost = 0;
    for (const auto &cell : cells)
    {
        int cellIndex = cell.second * width + cell.first;
        if (cellIndex < 0 || cellIndex >= width * height)
            continue; // Check bounds
        int cost = costmap->data[cellIndex];
        totalCost += cost;

        if (totalCost >= 50)
        {
            return true; // Path cost exceeds threshold
        }
    }

    return false; // Path cost is acceptable
}

bool checkLinearCollision(octomap::ColorOcTree *octree, geometry_msgs::Point p1, geometry_msgs::Point p2, float height, int conservative = 0)
{
    /**
     * Check if there is an obstable in line of two points. It extends 2 rays in an x-y plane and one ray going from the start to the end but at a given height therefore in 3d
     * If conservative is on, then it returns true even if it hits unknown voxels
     */
    octomap::point3d origin(p1.x, p1.y, 0);
    octomap::point3d end_2d(p2.x, p2.y, 0);
    octomap::point3d end_3d(p2.x, p2.y, height);

    std::vector<octomap::point3d> ray_2d;
    std::vector<octomap::point3d> ray_3d;

    // check the 3d linear collision to the desired height
    octree->computeRay(origin, end_2d, ray_2d);
    octree->computeRay(origin, end_3d, ray_3d);

    for (auto &pt : ray_2d)
    {
        auto node = octree->search(pt);
        if (node && node->getOccupancy() > 0.6)
        {
            return true;
        }
        if (node && node->getOccupancy() > 0.45 && conservative)
        {
            return true;
        }
    }

    for (auto &pt : ray_3d)
    {
        auto node = octree->search(pt);
        if (node && node->getOccupancy() > 0.6)
        {
            return true;
        }
        if (node && node->getOccupancy() > 0.45 && conservative)
        {
            return true;
        }
    }

    return false;
}

bool checkObstacleCollision(octomap::ColorOcTree *octree, nav_msgs::OccupancyGrid grid_2d, float x, float y)
{
    /**
     * Same for checkConservativeCollision but it doesn't return true even if it hits unknown voxels. Usually for initial behaviour
     */
    int grid_index = grid_2d.info.width * std::floor((y - grid_2d.info.origin.position.y) / grid_2d.info.resolution) + std::floor((x - grid_2d.info.origin.position.x) / grid_2d.info.resolution);
    if (grid_index < grid_2d.info.width * grid_2d.info.height)
    {
        if (grid_2d.data[grid_index] > GLOBAL_OCCUPANCY_COST_MAX)
        {
            return true;
        }
    }

    // Convert the point to octomap point3D
    octomap::point3d origin(x, y, 0);
    int num_angles = 10;
    int num_elevations = 20;
    float radius = 2.0;
    octomap::point3d end;

    // Define the angle increments
    float angle_increment = 2.0 * M_PI / num_angles;   // Horizontal angle increment
    float elevation_increment = M_PI / num_elevations; // Vertical angle increment

    // Loop through different horizontal angles
    for (int i = 0; i < num_angles; ++i)
    {
        float angle = i * angle_increment;

        // Loop through different elevation angles
        for (int j = 0; j < num_elevations; ++j)
        {
            float elevation = j * elevation_increment - M_PI / 2.0; // Center the dome

            // Calculate the direction of the ray
            float direction_x = std::cos(angle) * std::cos(elevation);
            float direction_y = std::sin(angle) * std::cos(elevation);
            float direction_z = std::sin(elevation);

            float magnitude = std::sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);

            // Avoid division by zero in case the vector is zero
            if (magnitude > 0)
            {
                direction_x /= magnitude;
                direction_y /= magnitude;
                direction_z /= magnitude;
            }
            octomap::point3d direction(direction_x, direction_y, direction_z);

            // Cast the ray from the origin in the calculated direction
            if (octree->castRay(origin, direction, end, true, radius))
            {
                // Check if the ray hits an occupied voxel
                octomap::ColorOcTreeNode *node = octree->search(end);
                if (node && (node->getLogOdds() >= 0.1))
                {
                    return true;
                }
            }
        }
    }

    // If no collision detected
    return false;
}

bool checkConservativeCollision(octomap::ColorOcTree *octree, nav_msgs::OccupancyGrid grid_2d, float x, float y)
{
    // Convert the point to octomap point3D

    int grid_index = grid_2d.info.width * std::floor((y - grid_2d.info.origin.position.y) / grid_2d.info.resolution) + std::floor((x - grid_2d.info.origin.position.x) / grid_2d.info.resolution);
    if (grid_index < grid_2d.info.width * grid_2d.info.height)
    {
        if (grid_2d.data[grid_index] > GLOBAL_OCCUPANCY_COST_MAX)
        {
            return true;
        }
    }

    octomap::point3d origin(x, y, 0);
    int num_angles = 20;
    int num_elevations = 20;
    float radius = 1.0;
    octomap::point3d end;

    // Define the angle increments
    float angle_increment = 2.0 * M_PI / num_angles;   // Horizontal angle increment
    float elevation_increment = M_PI / num_elevations; // Vertical angle increment

    // Loop through different horizontal angles
    for (int i = 0; i < num_angles; ++i)
    {
        float angle = i * angle_increment;

        // Loop through different elevation angles
        for (int j = 0; j < num_elevations; ++j)
        {
            float elevation = j * elevation_increment - M_PI / 2.0; // Center the dome

            // Calculate the direction of the ray
            float direction_x = std::cos(angle) * std::cos(elevation);
            float direction_y = std::sin(angle) * std::cos(elevation);
            float direction_z = std::sin(elevation);

            float magnitude = std::sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);

            // Avoid division by zero in case the vector is zero
            if (magnitude > 0)
            {
                direction_x /= magnitude;
                direction_y /= magnitude;
                direction_z /= magnitude;
            }
            octomap::point3d direction(direction_x, direction_y, direction_z);

            // Cast the ray from the origin in the calculated direction
            if (octree->castRay(origin, direction, end, true, radius))
            {
                octomap::ColorOcTreeNode *node = octree->search(end);
                if (node && (node->getLogOdds() > 0.0))
                {
                    return true;
                }
            }
        }
    }

    octomap::point3d vertical(0, 0, 1);
    if (octree->castRay(origin, vertical, end, true, 1))
    {
        octomap::OcTreeNode *node = octree->search(end);
        if (node && (node->getLogOdds() >= 0.0))
        {
            return true;
        }
    }

    // If no collision detected
    return false;
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const geometry_msgs::TransformStamped &transform)
{
    // Extract translation and rotation
    tf2::Vector3 translation(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    tf2::Quaternion rotation(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);

    // Convert pose to tf2 types
    tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    tf2::Quaternion orientation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    tf2::Matrix3x3 R(rotation);

    // Apply rotation and translation
    position = R * position;
    position += translation;
    orientation *= rotation;

    // Normalize quaternion
    orientation.normalize();

    // Convert tf2 types back to geometry_msgs
    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = position.x();
    transformed_pose.position.y = position.y();
    transformed_pose.position.z = position.z();
    transformed_pose.orientation.x = orientation.x();
    transformed_pose.orientation.y = orientation.y();
    transformed_pose.orientation.z = orientation.z();
    transformed_pose.orientation.w = orientation.w();

    return transformed_pose;
}

geometry_msgs::Pose transformRot(const geometry_msgs::Pose &pose, const geometry_msgs::TransformStamped &transform)
{
    // Extract translation and rotation
    // tf2::Vector3 translation(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    tf2::Quaternion rotation(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);

    // Convert pose to tf2 types
    tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    tf2::Quaternion orientation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    tf2::Matrix3x3 R(rotation);

    // Apply rotation and translation
    position = R * position;
    // position += translation;
    orientation *= rotation;

    // Normalize quaternion
    orientation.normalize();

    // Convert tf2 types back to geometry_msgs
    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = position.x();
    transformed_pose.position.y = position.y();
    transformed_pose.position.z = position.z();
    transformed_pose.orientation.x = orientation.x();
    transformed_pose.orientation.y = orientation.y();
    transformed_pose.orientation.z = orientation.z();
    transformed_pose.orientation.w = orientation.w();

    return transformed_pose;
}

geometry_msgs::Pose transformDir(const geometry_msgs::Pose &pose, const double yaw, int yaw_flag = 1)
{

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    // Create the rotation matrix for yaw
    tf2::Matrix3x3 R(
        cos_yaw, -sin_yaw, 0,
        sin_yaw, cos_yaw, 0,
        0, 0, 1);

    // Convert pose position to tf2 vector
    tf2::Vector3 position(pose.position.x,
                          pose.position.y,
                          pose.position.z);

    ROS_INFO("rotating postion %f, %f, %f", position.x(), position.y(), position.z());
    ROS_INFO("sin yaw and cos yaw: %f, %f", cos_yaw, sin_yaw);

    // Apply rotation to the position
    tf2::Vector3 rotated_position = R * position; // Rotate the position
    ROS_INFO("rotated postion %f, %f, %f", rotated_position.x(), rotated_position.y(), rotated_position.z());
    // Keep the original orientation (or you can apply the rotation if needed)
    tf2::Quaternion orientation(pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w);

    // Normalize quaternion (optional)
    orientation.normalize();

    // Convert tf2 types back to geometry_msgs
    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = rotated_position.x();
    transformed_pose.position.y = rotated_position.y();
    transformed_pose.position.z = rotated_position.z();
    transformed_pose.orientation.x = orientation.x();
    transformed_pose.orientation.y = orientation.y();
    transformed_pose.orientation.z = orientation.z();
    transformed_pose.orientation.w = orientation.w();

    return transformed_pose;
}

geometry_msgs::TransformStamped transformToTransformStamped(
    const tf2::Transform &transform,
    const std::string &frame_id,
    const std::string &child_frame_id,
    const ros::Time &timestamp)
{
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;

    // Convert translation
    transform_stamped.transform.translation.x = transform.getOrigin().x();
    transform_stamped.transform.translation.y = transform.getOrigin().y();
    transform_stamped.transform.translation.z = transform.getOrigin().z();

    // Convert rotation
    tf2::Quaternion quaternion = transform.getRotation();
    transform_stamped.transform.rotation.x = quaternion.x();
    transform_stamped.transform.rotation.y = quaternion.y();
    transform_stamped.transform.rotation.z = quaternion.z();
    transform_stamped.transform.rotation.w = quaternion.w();

    return transform_stamped;
}

namespace NBV
{
    class RayCaster
    {
    public:
        RayCaster();
        static ros::CallbackQueue odom_queue;

    private:
        geometry_msgs::PoseStamped calculate_sensor_position(float x, float y, float rec_x, float rec_y);
        float compute_gain(float x, float y, float rec_x, float rec_y, octomap::ColorOcTree *octree, float FOV, int FOV_resolution, int targeted, int publish = 0);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
        void build_tree(octomap::ColorOcTree *unknown_octree, float map_min_x, float map_min_y, float map_max_x, float map_max_y,
                        float FOV, int FOV_resolution, int targeted = 0, float heading_x = 0.0, float heading_y = 0.0, float goal_x = 0.0, float goal_y = 0.0);
        void normal_callback(const octomap_msgs::Octomap::ConstPtr &map_msg);
        void normal_callback_backup(const octomap_msgs::Octomap::ConstPtr &map_msg);
        void cuboid_callback(const visualization_msgs::MarkerArray::ConstPtr &search_area_msg, const octomap_msgs::Octomap::ConstPtr &map_msg);
        bool start_rrt(geometry_msgs::Point &start, geometry_msgs::Point &goal);
        void publish_best_node(const geometry_msgs::Point position);
        void occ_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr &ogrid);
        void unknown_tree_callback(const octomap_msgs::Octomap &msg);
        void look(float yaw);
        void turn_around();
        void turn();
        void send_velocity(double distance, double velocity, double dir_x, double dir_y);

        ros::NodeHandle nh;

        // store map
        octomap_msgs::Octomap octomap_msg;
        nav_msgs::OccupancyGrid grid_2d;

        std::shared_ptr<KD_RRT::KDTree> kd_tree_;
        KD_RRT::KDNode::KDNodePtr root_node;
        KD_RRT::KDNode::KDNodePtr current_robot_node;

        // All subscribers
        ros::Subscriber odom_sub;
        ros::Subscriber unknown_tree_sub;
        ros::Subscriber occ_grid_sub;
        ros::Subscriber octomap_sub;

        // All publishers
        ros::Publisher pub_cmd_vel;
        ros::Publisher pub_search_cuboid;
        ros::Publisher pub_franka_control;
        ros::Publisher pub_move_base_goal;
        ros::Publisher pub_tree;
        ros::Publisher pub_debug;
        ros::Publisher pub_sensor_poses;
        ros::Publisher pub_best_pose;
        ros::Publisher pub_recon_finish;

        // All synchronizers
        message_filters::Subscriber<visualization_msgs::MarkerArray> mf_search_area_sub;
        message_filters::Subscriber<octomap_msgs::Octomap> mf_octo_sub;
        boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<visualization_msgs::MarkerArray, octomap_msgs::Octomap>>> sync_recon;

        // All transforms

        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener ee_listener;
        tf2_ros::Buffer ee_buffer;
        tf2_ros::TransformListener listener;
        tf2_ros::Buffer base_buffer;
        tf2_ros::TransformListener base_listener;
        tf2_ros::Buffer front_buffer;
        tf2_ros::TransformListener front_listener;
        tf2_ros::Buffer map_buffer;
        tf2_ros::TransformListener map_listener;

        // All flags
        bool search_started;
        bool spin_done;
        bool reconstruction_started;
        bool reconstruction_finished;
        bool kd_tree_initialised;
        bool unknown_tree_arrived;
        bool grid_received;
        bool start_3d_search;
        bool current_search_done;

        // All variables
        Vec3 robot_pose;
        Vec3 robot_heading;
        Vec3 backup_plan_cuboid;
        Vec3 robot_pos;

        int retry;
        geometry_msgs::Pose start_pose;
        MoveBaseClient action_client;
        // moveit::planning_interface::MoveGroupInterface move_group_interface;
    };
} // namespace ends

#endif