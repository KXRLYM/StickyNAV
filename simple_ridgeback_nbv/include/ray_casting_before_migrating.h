#ifndef NBV_RAY_H
#define NBV_RAY_H

#include "ray.h"

#include <nav_msgs/Odometry.h>

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


#include <iostream>
#include <unordered_map>
#include <utility>
#include <limits>


namespace NBV {
    class RayCaster() {
        
    }
    class RayCastingNode {
        public:
        private:

    };
} // namespace ends

void set_quat(tf2::Quaternion& quat, geometry_msgs::Pose& pose) {
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return;
}

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

bool checkLinearCollision(octomap::OcTree* octree, geometry_msgs::Point p1, geometry_msgs::Point p2, float height, int conservative = 0) {
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
        if (node && node->getOccupancy() > 0.6) {
            return true;
        }
        if (node && node->getOccupancy() > 0.45 && conservative) {
            return true;
        }
    }

    for (auto& pt : ray_3d) {
        auto node = octree->search(pt);
        if (node && node->getOccupancy() > 0.6) {
            return true;
        }
        if (node && node->getOccupancy() > 0.45 && conservative) {
            return true;
        }
    }

    return false;
}

bool checkObstacleCollision(octomap::OcTree* octree, float x, float y) {
    // Convert the point to octomap point3D
    octomap::point3d origin(x, y, 0);
    int num_angles = 10;
    int num_elevations = 20;
    float radius = 2.0;
    octomap::point3d end;
    

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

            
            // Cast the ray from the origin in the calculated direction
            if (octree->castRay(origin, direction, end, false, radius)) {
                // Check if the ray hits an occupied voxel
                octomap::OcTreeNode* node = octree->search(end);
                if (node && (node->getOccupancy() > 0.6)) {
                    return true;
                }
            }
        }
    }


    // If no collision detected
    return false;
}

bool checkConservativeCollision(octomap::OcTree* octree, float x, float y) {
    // Convert the point to octomap point3D
    octomap::point3d origin(x, y, 0);
    int num_angles = 20;
    int num_elevations = 20;
    float radius = 1.8;
    octomap::point3d end;

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
            

            
            // Cast the ray from the origin in the calculated direction
            if (octree->castRay(origin, direction, end, false, radius)) {
                octomap::OcTreeNode* node = octree->search(end);
                if (node && (node->getOccupancy() > 0.4)) {
                    return true;
                }
            }
        }
    }

    /**
    octomap::point3d vertical(0,0,1);
    if (octree->castRay(origin,vertical,end,false,8)) {
        octomap::OcTreeNode* node = octree->search(end);
        if (node && (node->getOccupancy() > 0.4)) {
            return true;
        }
    } */
    // If no collision detected
    return false;
}

// Define the voxel position structure
struct VoxelPosition {
    int x, y, z;

    bool operator==(const VoxelPosition& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Implement the hash function for VoxelPosition
namespace std {
    template <>
    struct hash<VoxelPosition> {
        std::size_t operator()(const VoxelPosition& vp) const {
            return std::hash<int>()(vp.x) ^ (std::hash<int>()(vp.y) << 1) ^ (std::hash<int>()(vp.z) << 2);
        }
    };
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, const geometry_msgs::TransformStamped& transform) {
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

geometry_msgs::Pose transformPose2(const geometry_msgs::Pose& pose, const geometry_msgs::TransformStamped& transformRot, const geometry_msgs::TransformStamped& transformPos) {
    // Rotation map to the base
    // Pose base to the arm
    // Extract translation and rotation
    tf2::Vector3 translation(transformPos.transform.translation.x, transformPos.transform.translation.y, transformPos.transform.translation.z);
    tf2::Quaternion rotation(transformPos.transform.rotation.x, transformPos.transform.rotation.y, transformPos.transform.rotation.z, transformPos.transform.rotation.w);
    tf2::Quaternion rotation2(transformRot.transform.rotation.x, transformRot.transform.rotation.y, transformRot.transform.rotation.z, transformRot.transform.rotation.w);
    // Convert pose to tf2 types
    tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    tf2::Quaternion orientation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    tf2::Matrix3x3 R1(rotation);
    tf2::Matrix3x3 R2(rotation2);

    // Apply rotation and translation
    position = (R1 * position + translation);

    orientation *= rotation;
    orientation *= rotation2;

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

geometry_msgs::Pose transformDir(const geometry_msgs::Pose& pose, const double yaw, int yaw_flag = 1) {

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);


    // Create the rotation matrix for yaw
    tf2::Matrix3x3 R(
        cos_yaw, -sin_yaw, 0,
        sin_yaw,  cos_yaw, 0,
        0,           0,      1
    );

    
    // Convert pose position to tf2 vector
    tf2::Vector3 position(pose.position.x, 
                          pose.position.y, 
                          pose.position.z);

    //ROS_INFO("rotating postion %f, %f, %f", position.x(), position.y(), position.z());

    // Apply rotation to the position
    tf2::Vector3 rotated_position = R * position; // Rotate the position
    //ROS_INFO("rotated postion %f, %f, %f", rotated_position.x(), rotated_position.y(), rotated_position.z());
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



geometry_msgs::Pose transformPos(const geometry_msgs::Pose& pose, const geometry_msgs::TransformStamped& transform) {
    // Extract translation and rotation
    tf2::Vector3 translation(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

    // Convert pose to tf2 types
    tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);

    // Apply rotation and translation
    position += translation;

    // Convert tf2 types back to geometry_msgs
    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = position.x();
    transformed_pose.position.y = position.y();
    transformed_pose.position.z = position.z();
    transformed_pose.orientation.x = pose.orientation.x;
    transformed_pose.orientation.y = pose.orientation.y;
    transformed_pose.orientation.z = pose.orientation.z;
    transformed_pose.orientation.w = pose.orientation.w;

    return transformed_pose;
}


geometry_msgs::TransformStamped transformToTransformStamped(
    const tf2::Transform& transform, 
    const std::string& frame_id, 
    const std::string& child_frame_id, 
    const ros::Time& timestamp) 
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

#endif