#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ray_casting.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <Eigen/Dense>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>


#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<time.h>
#include<iostream>
#include<stdlib.h>

#include <std_msgs/String.h>
#include <json/json.h> 
#include <map>
#include <string>

using namespace std;
using namespace Eigen;
namespace ros {
namespace message_traits {


}  // namespace message_traits
}  // namespace ros

Vec3 quaternionToEuler(const geometry_msgs::Quaternion& quat) {
    tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return Vec3(roll, pitch, yaw);
}

// Function to convert Euler angles to a unit direction vector
Vec3 eulerToDirection(float roll, float pitch, float yaw) {
    float cos_roll = std::cos(roll);
    float sin_roll = std::sin(roll);
    float cos_pitch = std::cos(pitch);
    float sin_pitch = std::sin(pitch);
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);

    // Rotation matrix components
    Vec3 dir;
    dir.x = cos_pitch * cos_yaw;
    dir.y = cos_pitch * sin_yaw;
    dir.z = -sin_pitch;

    return dir;
}

float pointToPlaneDistance(const Vec3& point, const Plane& plane) {
    return fabs(plane.normal.dot(point) + plane.d) / plane.normal.length();
}

bool findIntersection(const Vec3& P1, const Vec3& P2, const Vec3& P3, const Vec3& P4, Vec3& intersection) {
    Vec3 d1 = P2 - P1; // Direction vector of line 1
    Vec3 d2 = P4 - P3; // Direction vector of line 2
    Vec3 r = P3 - P1;  // Vector between the start points of the lines

    // Calculate determinants
    float a = d1.dot(d1);
    float b = d1.dot(d2);
    float e = d2.dot(d2);
    float c = d1.dot(r);
    float f = d2.dot(r);

    float denom = a * e - b * b;
    if (std::abs(denom) < 1e-6f) { // Lines are parallel or nearly parallel
        return false;
    }

    float t = (b * f - e * c) / denom;
    float s = (a * f - b * c) / denom;

    // Find the intersection point
    intersection = P1 + d1 * t;
    return true;
}

std::pair<std::vector<Plane>, std::vector<int>> cuboidRansac(const geometry_msgs::PoseArray::ConstPtr& pose_array) {
    //srand(time(NULL));
    srand(time(NULL));

    int i(0);
    int max_iteration(500);
    float threshold = 0.05f;
    vector<int> bestInliers;
    vector<Plane> bestPlanes;

    Vec3 points[6];

    while (i < max_iteration) {
        i++;
        for (int j = 0; j < 6; j++) {
            int random_number = rand() % pose_array->poses.size();
            points[j] = Vec3(pose_array->poses[random_number].position.x, pose_array->poses[random_number].position.y, pose_array->poses[random_number].position.z);
        }
        Vec3 vecA = points[1] - points[0];
        Vec3 vecB = points[2] - points[0];
        Vec3 vecC = vecA.cross(vecB);
        vecC = vecC / vecC.length();
        float d1 = -vecC.dot(points[0]);
        Plane plane1(vecC, d1);

        float distance = pointToPlaneDistance(points[3], plane1);
        Vec3 p4_proj_plane = points[3] - vecC*distance;

        Vec3 vecD = p4_proj_plane - points[3];
        Vec3 vecE = points[4] - points[3];
        Vec3 vecF = vecD.cross(vecE);
        vecF = vecF / vecF.length();
        float d2 = -vecF.dot(points[3]);
        Plane plane2(vecF, d2);

        Vec3 vecG = vecC.cross(vecF);
        vecG = vecG / vecG.length();
        float d3 = -vecG.dot(points[5]);
        Plane plane3(vecG, d3);

        vector<Plane> planes = {plane1, plane2, plane3};

        vector<int> inliers;
        vector<float> dist_pt(pose_array->poses.size(), FLT_MAX);
        for (const auto& plane : planes) {
            for (size_t idx = 0; idx < pose_array->poses.size(); ++idx) {
                Vec3 pt(
                    pose_array->poses[idx].position.x,
                    pose_array->poses[idx].position.y,
                    pose_array->poses[idx].position.z
                );
                dist_pt[idx] = min(dist_pt[idx], pointToPlaneDistance(pt, plane));
            }
        }

        // Find inliers
        for (size_t idx = 0; idx < dist_pt.size(); ++idx) {
            if (dist_pt[idx] <= threshold) {
                inliers.push_back(idx);
            }
        }

        // Update best inliers and planes
        if (inliers.size() > bestInliers.size()) {
            bestInliers = inliers;
            bestPlanes = planes;
        }

    }
    return std::make_pair(bestPlanes, bestInliers);
}

bool line_intersection(Vec3 p1, Vec3 p2, Vec3 p3, Vec3 p4, int id) {
    float x1 = p1.x;
    float y1 = p1.y;
    float x2 = p2.x;
    float y2 = p2.y;
    float x3 = p3.x;
    float y3 = p3.y;
    float x4 = p4.x;
    float y4 = p4.y;

    float denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    // Lines are parallel if the denominator is zero
    if (std::abs(denom) < 1e-6f) {
        return false;
    }

    // Compute uA and uB
    float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    // Check if the intersection point is within the bounds of both segments
    if (uA > 0 && uA < 1 && uB > 0 && uB < 1) {
        // Calculate intersection point
        float intersectionX = x1 + uA * (x2 - x1);
        float intersectionY = y1 + uA * (y2 - y1);
        
        return true;
    }

    return false;
}

int box_intersection(Vec3 p1, Vec3 p2, Vec3 minmin, Vec3 minmax, Vec3 maxmax, Vec3 maxmin) {
    int count(0);
    if(line_intersection(p1, p2, minmin, minmax, 78)) {
        count ++;
    }
    if(line_intersection(p1, p2, minmax, maxmax, 77)) {
        count ++;
    }
    if(line_intersection(p1, p2, maxmax, maxmin, 76)) {
        count ++;
    }
    if(line_intersection(p1, p2, maxmin, minmin, 75)) {
        count ++;
    }
    return count;

}

class RansacNode
{
public:
    RansacNode()
        : nh(),
        pub_matched_cuboid(nh.advertise<visualization_msgs::Marker>("/match_cuboid", 10)),
        pub_search_area(nh.advertise<visualization_msgs::MarkerArray>("/search_area", 10)),
        search_area_timer(nh.createTimer(ros::Duration(0.5), std::bind(&RansacNode::search_timer_callback, this, std::placeholders::_1))),
        tf2_listener(tf_buffer),
        debug_marker(nh.advertise<visualization_msgs::MarkerArray>("/debug_marker",10)),
        odom_only_sub(nh.subscribe<nav_msgs::Odometry>("/odom", 10, &RansacNode::localiseBelief, this)),
        first_corner(99),
        second_corner(99),
        third_corner(99),
        range(10),
        match_corners_pub(nh.advertise<geometry_msgs::PoseArray>("/match_cuboid_corners", 10)),
        exploration_started(false)
    {
        matchpoints_sub.subscribe(nh, "/transformed_poses", 10);
        odom_sub.subscribe(nh, "/odom", 10);
        using sync_pol = message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseArray>;
        matchpoint_sync.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(500), odom_sub, matchpoints_sub));
        matchpoint_sync->registerCallback(boost::bind(&RansacNode::match_callback, this, _1, _2));


        num_obs = {0,0,0,0,0};
        Vec3 minmin_corner(0,0,0);
        Vec3 minmax_corner(0,0,0);    
        Vec3 maxmax_corner(0,0,0);
        Vec3 maxmin_corner(0,0,0);
        Vec3 z_corner(0,0,0);
        belief = {minmin_corner, minmax_corner, maxmax_corner, maxmin_corner, z_corner};

        Vec3 search_min(0,0,0);
        Vec3 search_max(0,0,0);
        Vec3 robot_start_pose(0,0,0);

    }

    void search_timer_callback(const ros::TimerEvent&) {
        if (search_min.x == 0 && search_max.x == 0) {
            return;
        } else {
            visualiseBoundingBox(search_min.x, search_max.x, search_min.y, search_max.y);
        }
    }


    void localiseBelief(const nav_msgs::Odometry::ConstPtr& odom) {
        if (this->num_obs[0] != 0) {
            geometry_msgs::TransformStamped odom_map_transform = tf_buffer.lookupTransform("map","odom", ros::Time(0));
            geometry_msgs::PoseStamped ros_pos_odom;
            ros_pos_odom.header = odom->header;
            ros_pos_odom.pose = odom->pose.pose;
            geometry_msgs::PoseStamped ros_pos_map;
            tf2::doTransform(ros_pos_odom, ros_pos_map, odom_map_transform);
            
            Vec3 robot_origin(ros_pos_map.pose.position.x, ros_pos_map.pose.position.y, ros_pos_map.pose.position.z);
            // Extract and convert orientation
            Vec3 euler = quaternionToEuler(odom->pose.pose.orientation);
            float roll = euler.x;
            float pitch = euler.y;
            float yaw = euler.z;
            // Get unit direction vector
            Vec3 robot_direction = eulerToDirection(roll, pitch, yaw);

            std::vector<Vec3> corners = {this->belief[0], this->belief[1], this->belief[2], this->belief[3]};
            std::vector<int> indices = {};

            robot_origin.z = 0.0;
            
            // add corners to the indices that are visible (doesn't collide with any rectangle sides)
            if (!box_intersection(robot_origin, this->belief[MINMIN], this->belief[MINMIN], this->belief[MINMAX], this->belief[MAXMAX], this->belief[MAXMIN])) {
                indices.push_back(MINMIN);
            } 
            
            if (!box_intersection(robot_origin, this->belief[MINMAX], this->belief[MINMIN], this->belief[MINMAX], this->belief[MAXMAX], this->belief[MAXMIN])) {
                indices.push_back(MINMAX);
            } 
            
            if (!box_intersection(robot_origin, this->belief[MAXMAX], this->belief[MINMIN], this->belief[MINMAX], this->belief[MAXMAX], this->belief[MAXMIN])) {
                indices.push_back(MAXMAX);
            } 
            
            if (!box_intersection(robot_origin, this->belief[MAXMIN], this->belief[MINMIN], this->belief[MINMAX], this->belief[MAXMAX], this->belief[MAXMIN])) {
                indices.push_back(MAXMIN);
            }

            /*
            float min_distance = std::numeric_limits<float>::max();
            float second_min_distance = std::numeric_limits<float>::max();

            int temp_first(0);
            int temp_second(0);

            for (int i = 0; i < corners.size(); i++) {
                float distance = (corners[i] - robot_origin).length();
                if (distance < min_distance) {
                    second_min_distance = min_distance;
                    temp_second = temp_first;

                    min_distance = distance;
                    temp_first = i;
                } else if (distance < second_min_distance) {
                    second_min_distance = distance;
                    temp_second = i;
                }
            }*/

            /** 
            this->first_corner = temp_first;
            this->second_corner = temp_second;*/

        

            if (indices.size() == 3) {
                this->first_corner = indices[0];
                this->second_corner = indices[1];
                this->third_corner = indices[2];   
            } else if (indices.size() == 2) {
                this->first_corner = indices[0];
                this->second_corner = indices[1];
                this->third_corner = 99;
            } else {
                this->first_corner = indices[0];
                this->second_corner = indices[1];
                this->third_corner = indices[2];
            }


            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker first_point;
            first_point.id = 22;
            first_point.pose.position.x = corners[first_corner].x;
            first_point.pose.position.y = corners[first_corner].y;
            first_point.pose.position.z = corners[first_corner].z;
            first_point.color.g = 1.0;
            first_point.color.a = 1.0;                    
            first_point.header.frame_id = "map";
            first_point.type = visualization_msgs::Marker::SPHERE;
            first_point.action = visualization_msgs::Marker::ADD;
            first_point.header.stamp = ros::Time::now();
            first_point.scale.x = 0.1;
            first_point.scale.y = 0.1;
            first_point.scale.z = 0.1;
            first_point.pose.orientation.w = 1.0;
            markers.markers.push_back(first_point);
            
            visualization_msgs::Marker second_point;
            second_point.id = 29;
            second_point.pose.position.x = corners[second_corner].x;
            second_point.pose.position.y = corners[second_corner].y;
            second_point.pose.position.z = corners[second_corner].z;
            second_point.color.b = 1.0;
            second_point.color.a = 1.0;
            second_point.header.frame_id = "map";
            second_point.scale.x = 0.1;
            second_point.scale.y = 0.1;
            second_point.scale.z = 0.1;
            second_point.pose.orientation.w = 1.0;
            second_point.type = visualization_msgs::Marker::SPHERE;
            second_point.header.stamp = ros::Time::now();
            markers.markers.push_back(second_point);

            if (this->third_corner != 99) {
                visualization_msgs::Marker third_point;
                second_point.id = 30;
                second_point.pose.position.x = corners[third_corner].x;
                second_point.pose.position.y = corners[third_corner].y;
                second_point.pose.position.z = corners[third_corner].z;
                second_point.color.g = 1.0;
                second_point.color.a = 1.0;
                second_point.header.frame_id = "map";
                second_point.scale.x = 0.1;
                second_point.scale.y = 0.1;
                second_point.scale.z = 0.1;
                second_point.pose.orientation.w = 1.0;
                second_point.type = visualization_msgs::Marker::SPHERE;
                second_point.header.stamp = ros::Time::now();
                second_point.lifetime = ros::Duration(1);
                markers.markers.push_back(second_point);               
            }

            debug_marker.publish(markers);


        }
    }

    void match_callback(const nav_msgs::Odometry::ConstPtr& odom, const geometry_msgs::PoseArray::ConstPtr& pose_array) {
        ROS_INFO("matching -!");
        geometry_msgs::TransformStamped odom_map_transform = tf_buffer.lookupTransform("map","odom", ros::Time(0));
        geometry_msgs::PoseStamped ros_pos_odom;
        ros_pos_odom.header = odom->header;
        ros_pos_odom.pose = odom->pose.pose;
        geometry_msgs::PoseStamped ros_pos_map;
        tf2::doTransform(ros_pos_odom, ros_pos_map, odom_map_transform);

        float superglue_match_radius = 5;

        /** 
        if (exploration_started &&
         std::sqrt(std::pow(ros_pos_map.pose.position.x - robot_start_pose.x,2) + std::pow(ros_pos_map.pose.position.y - robot_start_pose.y,2)
                > superglue_match_radius)) {
            ROS_INFO("detection ")
            return;
        }
        if (!exploration_started) {
            robot_start_pose.x = ros_pos_map.pose.position.x;
            robot_start_pose.y = ros_pos_map.pose.position.y;
            exploration_started = true;
        }*/


        auto angular_vel = odom->twist.twist.angular;
        auto linear_vel = odom->twist.twist.linear;
        if (pose_array->poses.size() > 8) {
            auto ransac_result = cuboidRansac(pose_array); 
            if (!ransac_result.first.empty()) {
                ROS_INFO("Calculating RANSAC..");
                exploration_started = true;
                robot_start_pose.x = ros_pos_map.pose.position.x;
                robot_start_pose.y = ros_pos_map.pose.position.y;
                // Compute the bounding box (cuboid) from the best planes
                Vec3 min_point(FLT_MAX, FLT_MAX, FLT_MAX);
                Vec3 max_point(-FLT_MAX, -FLT_MAX, -FLT_MAX);

                for (const auto& idx : ransac_result.second) {
                    Vec3 pt(
                        pose_array->poses[idx].position.x,
                        pose_array->poses[idx].position.y,
                        pose_array->poses[idx].position.z
                    );

                    min_point.x = min(min_point.x, pt.x);
                    min_point.y = min(min_point.y, pt.y);
                    min_point.z = min(min_point.z, pt.z);

                    max_point.x = max(max_point.x, pt.x);
                    max_point.y = max(max_point.y, pt.y);
                    max_point.z = max(max_point.z, pt.z);
                }

                Vec3 corner1(min_point.x, min_point.y, 0.0);
                Vec3 corner2(min_point.x, max_point.y, 0.0);
                Vec3 corner3(max_point.x, max_point.y, 0.0);
                Vec3 corner4(max_point.x, min_point.y, 0.0);
                std::vector<Vec3> new_corners = {corner1, corner2, corner3, corner4};

                float area = (max_point.x - min_point.x) * (max_point.y - min_point.y);
                float area_thres = 0.2;
                float height_thres = 0.3;
                if (area < area_thres || max_point.z < height_thres) {
                    ROS_ERROR("Cuboid too small, explore");
                    return;
                }

                if (num_obs[0] == 0) {
                    ROS_INFO("Initialising the cube");
                    this->belief[MINMIN] = new_corners[MINMIN];
                    this->belief[MINMAX] = new_corners[MINMAX];
                    this->belief[MAXMAX] = new_corners[MAXMAX];
                    this->belief[MAXMIN] = new_corners[MAXMIN];
                }

                /**
                Vec3 robot_origin(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
                // Extract and convert orientation
                Vec3 euler = quaternionToEuler(odom->pose.pose.orientation);
                float roll = euler.x;
                float pitch = euler.y;
                float yaw = euler.z;
                // Get unit direction vector
                Vec3 robot_direction = eulerToDirection(roll, pitch, yaw);


                Line odom_line(robot_origin, robot_origin + robot_direction);

                std::vector<Vec3> corners = {this->belief[0], this->belief[1], this->belief[2], this->belief[3]};
                int first_corner(0);
                int second_corner(0);
                float min_distance = std::numeric_limits<float>::max();
                float second_min_distance = std::numeric_limits<float>::max();

                for (int i = 0; i < corners.size(); i++) {
                    float distance = (corners[i] - robot_origin).length();
                    if (distance < min_distance) {
                        second_min_distance = min_distance;
                        second_corner = first_corner;

                        min_distance = distance;
                        first_corner = i;
                    } else if (distance < second_min_distance) {
                        second_min_distance = distance;
                        second_corner = i;
                    }
                } */
                this->num_obs[0]++;
                this->belief[MAXZ].z = ((this->belief[MAXZ].z * (this->num_obs[0] - 1)) + max_point.z) / this->num_obs[0];
                std::vector<Vec3> corners = {this->belief[0], this->belief[1], this->belief[2], this->belief[3]};

                int first_corner = this->first_corner;
                int second_corner = this->second_corner;
                int third_corner = this->third_corner;

                // Updating the belief using the running average only the closest corners
                // Compute the center and dimensions of the bounding box
                int alignment(0); // 1 for min corners are x-axis aligned 2 for min corners are y_axis aligned, 3 for max corners are x-axis aligned and 4 for max corners to y_axis aligned
                if ((first_corner == MINMIN && second_corner == MAXMIN) || (second_corner == MINMIN && first_corner == MAXMIN)) {
                    std::cout << "min corners x-axis aligned" << std::endl;
                    alignment = 1;
                    this->num_obs[alignment]++;
                } else if ((first_corner == MINMIN && second_corner == MINMAX) || (second_corner == MINMIN && first_corner == MINMAX)) {
                    std::cout << "min corners y-axis aligned" << std::endl;
                    alignment = 2;
                    this->num_obs[alignment]++;
                } else if ((first_corner == MINMAX && second_corner == MAXMAX) || (second_corner == MINMAX && first_corner == MAXMAX)) {
                    std::cout << "max corners x-axis aligned" << std::endl;
                    alignment = 3;
                    this->num_obs[alignment]++;
                } else if ((first_corner == MAXMIN && second_corner == MAXMAX) || (second_corner == MAXMIN && first_corner == MAXMAX)) {
                    std::cout << "max corners y-axis aligned" << std::endl;
                    alignment = 4;
                    this->num_obs[alignment]++;
                } else {
                    std::cout << "corners not upated" << std::endl;
                    std::cout << first_corner << std::endl;
                    std::cout << second_corner << std::endl;
                }


                if (alignment == 1) {
                    if (new_corners[first_corner].y <= new_corners[second_corner].y) {
                        //need to use a smaller value
                        this->belief[first_corner].y = ((this->belief[first_corner].y * (this->num_obs[alignment] - 1)) + new_corners[first_corner].y) / this->num_obs[alignment];
                        this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[first_corner].y) / this->num_obs[alignment];
                    } else {
                        this->belief[first_corner].y = ((this->belief[first_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];
                        this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];                
                    }
                } else if (alignment == 2) {
                    if (new_corners[first_corner].x <= new_corners[second_corner].x) {
                        //need to use a smaller value
                        this->belief[first_corner].x = ((this->belief[first_corner].x * (this->num_obs[alignment] - 1)) + new_corners[first_corner].x) / this->num_obs[alignment];
                        this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[first_corner].x) / this->num_obs[alignment];
                    } else {
                        this->belief[first_corner].x = ((this->belief[first_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];
                        this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];                
                    }
                } else if (alignment == 3) {
                    if (new_corners[first_corner].y >= new_corners[second_corner].y) {
                        //need to use a bigger value
                        this->belief[first_corner].y = ((this->belief[first_corner].y * (this->num_obs[alignment] - 1)) + new_corners[first_corner].y) / this->num_obs[alignment];
                        this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[first_corner].y) / this->num_obs[alignment];
                    } else {
                        this->belief[first_corner].y = ((this->belief[first_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];
                        this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];                
                    }
                }else if (alignment == 4) {
                    if (new_corners[first_corner].x >= new_corners[second_corner].x) {
                        //need to use a bigger value
                        this->belief[first_corner].x = ((this->belief[first_corner].x * (this->num_obs[alignment] - 1)) + new_corners[first_corner].x) / this->num_obs[alignment];
                        this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[first_corner].x) / this->num_obs[alignment];
                    } else {
                        this->belief[first_corner].x = ((this->belief[first_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];
                        this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];                
                    }
                } 
                

                if (third_corner != 99) {
                    // Updating the belief using the running average only the closest corners
                    // Compute the center and dimensions of the bounding box
                    int alignment(0); // 1 for min corners are x-axis aligned 2 for min corners are y_axis aligned, 3 for max corners are x-axis aligned and 4 for max corners to y_axis aligned
                    if ((second_corner == MINMIN && third_corner == MAXMIN) || (third_corner == MINMIN && second_corner == MAXMIN)) {
                        std::cout << "min corners x-axis aligned 2" << std::endl;
                        alignment = 1;
                        this->num_obs[alignment]++;
                    } else if ((second_corner == MINMIN && third_corner == MINMAX) || (third_corner == MINMIN && second_corner == MINMAX)) {
                        std::cout << "min corners y-axis aligned 2" << std::endl;
                        alignment = 2;
                        this->num_obs[alignment]++;
                    } else if ((second_corner == MINMAX && third_corner == MAXMAX) || (third_corner == MINMAX && second_corner == MAXMAX)) {
                        std::cout << "max corners x-axis aligned 2" << std::endl;
                        alignment = 3;
                        this->num_obs[alignment]++;
                    } else if ((second_corner == MAXMIN && third_corner == MAXMAX) || (third_corner == MAXMIN && second_corner == MAXMAX)) {
                        std::cout << "max corners y-axis aligned 2" << std::endl;
                        alignment = 4;
                        this->num_obs[alignment]++;
                    } else {
                        std::cout << "corners not upated" << std::endl;
                    }


                    if (alignment == 1) {
                        if (new_corners[second_corner].y <= new_corners[third_corner].y) {
                            //need to use a smaller value
                            this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];
                            this->belief[third_corner].y = ((this->belief[third_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];
                        } else {
                            this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[third_corner].y) / this->num_obs[alignment];
                            this->belief[third_corner].y = ((this->belief[third_corner].y * (this->num_obs[alignment] - 1)) + new_corners[third_corner].y) / this->num_obs[alignment];                
                        }
                    } else if (alignment == 2) {
                        if (new_corners[second_corner].x <= new_corners[third_corner].x) {
                            //need to use a smaller value
                            this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];
                            this->belief[third_corner].x = ((this->belief[third_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];
                        } else {
                            this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[third_corner].x) / this->num_obs[alignment];
                            this->belief[third_corner].x = ((this->belief[third_corner].x * (this->num_obs[alignment] - 1)) + new_corners[third_corner].x) / this->num_obs[alignment];                
                        }
                    } else if (alignment == 3) {
                        if (new_corners[second_corner].y >= new_corners[third_corner].y) {
                            //need to use a bigger value
                            this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];
                            this->belief[third_corner].y = ((this->belief[third_corner].y * (this->num_obs[alignment] - 1)) + new_corners[second_corner].y) / this->num_obs[alignment];
                        } else {
                            this->belief[second_corner].y = ((this->belief[second_corner].y * (this->num_obs[alignment] - 1)) + new_corners[third_corner].y) / this->num_obs[alignment];
                            this->belief[third_corner].y = ((this->belief[third_corner].y * (this->num_obs[alignment] - 1)) + new_corners[third_corner].y) / this->num_obs[alignment];                
                        }
                    }else if (alignment == 4) {
                        if (new_corners[second_corner].x >= new_corners[third_corner].x) {
                            //need to use a bigger value
                            this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];
                            this->belief[third_corner].x = ((this->belief[third_corner].x * (this->num_obs[alignment] - 1)) + new_corners[second_corner].x) / this->num_obs[alignment];
                        } else {
                            this->belief[second_corner].x = ((this->belief[second_corner].x * (this->num_obs[alignment] - 1)) + new_corners[third_corner].x) / this->num_obs[alignment];
                            this->belief[third_corner].x = ((this->belief[third_corner].x * (this->num_obs[alignment] - 1)) + new_corners[third_corner].x) / this->num_obs[alignment];                
                        }
                    } 
                }

                Vec3 centre((this->belief[MINMIN].x+this->belief[MAXMIN].x)/2, (this->belief[MINMIN].y + this->belief[MINMAX].y) / 2, this->belief[MAXZ].z/2);
                Vec3 centre_length(this->belief[MAXMIN].x - this->belief[MINMIN].x, this->belief[MINMAX].y - this->belief[MINMIN].y, this->belief[MAXZ].z);

                ROS_INFO("MINMIN [x: %f y: %f z:%f]", this->belief[MINMIN].x, this->belief[MINMIN].y, this->belief[MINMIN].z);
                ROS_INFO("MINMAX [x: %f y: %f z:%f]", this->belief[MINMAX].x, this->belief[MINMAX].y, this->belief[MINMAX].z);
                ROS_INFO("MAXMAX [x: %f y: %f z:%f]", this->belief[MAXMAX].x, this->belief[MAXMAX].y, this->belief[MAXMAX].z);
                ROS_INFO("MAXMIN [x: %f y: %f z:%f]", this->belief[MAXMIN].x, this->belief[MAXMIN].y, this->belief[MAXMIN].z);
                
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "matched_cuboid";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position.x = centre.x;
                marker.pose.position.y = centre.y;
                marker.pose.position.z = centre.z;

                marker.scale.x = centre_length.x;
                marker.scale.y = centre_length.y;
                marker.scale.z = centre_length.z;

                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.5; // Semi-transparent

                marker.lifetime = ros::Duration();
                pub_matched_cuboid.publish(marker);

                geometry_msgs::PoseArray match_corners;
                match_corners.header.stamp = ros::Time::now();
                match_corners.header.frame_id = "map";
                for (Vec3 corner : this->belief) {
                    geometry_msgs::Pose new_corner;
                    new_corner.position.x = corner.x;
                    new_corner.position.y = corner.y;
                    if (corner.z != 0.0) {
                        new_corner.position.z = corner.z;
                    } else {
                        new_corner.position.z = 0.0;
                    }
                    new_corner.orientation.w = 1.0;
                    match_corners.poses.push_back(new_corner);
                }
                match_corners_pub.publish(match_corners);

                double extension = 6;
                search_min.x = this->belief[MINMIN].x - extension;
                search_min.y = this->belief[MINMIN].y - extension;
                search_max.x = this->belief[MAXMAX].x + extension;
                search_max.y = this->belief[MAXMAX].y + extension;

            } else {
                ROS_INFO("No valid results");
            }
        }
        
    }

    void visualiseBoundingBox(double min_x, double max_x, double min_y, double max_y)
        {
            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker bounding_box;
            
            bounding_box.header.frame_id = "map";
            bounding_box.header.stamp = ros::Time::now();
            bounding_box.ns = "bounding_box";
            bounding_box.id = 0;
            bounding_box.type = visualization_msgs::Marker::LINE_STRIP;
            bounding_box.action = visualization_msgs::Marker::ADD;
            bounding_box.pose.orientation.w = 1.0;
            bounding_box.scale.x = 0.1;  // Line width
            bounding_box.color.r = 0.0;
            bounding_box.color.g = 1.0;
            bounding_box.color.b = 1.0;
            bounding_box.color.a = 1.0;

            // Define points of the bounding box
            geometry_msgs::Point p1, p2, p3, p4;

            //ROS_INFO("Boundig box: %f, %f, %f, %f", min_x, max_x, min_y, max_y);

            p1.x = min_x;
            p1.y = min_y;
            p1.z = 0;
            bounding_box.points.push_back(p1);

            p2.x = max_x;
            p2.y = min_y;
            p2.z = 0;
            bounding_box.points.push_back(p2);

            p3.x = max_x;
            p3.y = max_y;
            p3.z = 0;
            bounding_box.points.push_back(p3);

            p4.x = min_x;
            p4.y = max_y;
            p4.z = 0;
            bounding_box.points.push_back(p4);

            // Close the loop
            bounding_box.points.push_back(p1);

            markers.markers.push_back(bounding_box);
            pub_search_area.publish(markers);
        }
  

private:
    ros::NodeHandle nh;
    ros::Timer search_area_timer;
    ros::Publisher pub_matched_cuboid;
    ros::Publisher debug_marker;
    ros::Publisher match_corners_pub;
    ros::Subscriber cast_map_sub;
    ros::Publisher pub_search_area;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Subscriber<geometry_msgs::PoseArray> matchpoints_sub;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseArray>>> matchpoint_sync;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener;
    std::vector<Vec3> belief; // four corners and one z vector
    std::vector<int> num_obs; // observing (1) height and (2) four edges
    ros::Subscriber odom_only_sub;
    ros::Subscriber sub_key_map;
    int first_corner;
    int second_corner;
    int third_corner;
    int range;
    int field_of_view;
    Vec3 search_min;
    Vec3 search_max;
    Vec3 robot_start_pose;
    bool exploration_started;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RANSAC_node");
    RansacNode node;
    ros::spin();
    return 0;
}
