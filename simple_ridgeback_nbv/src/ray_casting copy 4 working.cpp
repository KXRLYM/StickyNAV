#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

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

void calculateCombinedBoundingBox(const octomap::point3d& min1, const octomap::point3d& max1,
                                  const Vec3& min2, const Vec3& max2,
                                  Vec3& combined_min, Vec3& combined_max) {
    // Calculate the minimum point of the combined bounding box
    combined_min = Vec3(
        std::min(min1.x(), min2.x),
        std::min(min1.y(), min2.y),
        std::min(min1.z(), min2.z)
    );

    // Calculate the maximum point of the combined bounding box
    combined_max = Vec3(
        std::max(max1.x(), max2.x),
        std::max(max1.y(), max2.y),
        std::max(max1.z(), max2.z)
    );
}

bool isPointIn2DRectangle(float x, float y, Vec3 rec_min, Vec3 rec_max) {
    return (x >= rec_min.x && x <= rec_max.x &&
            y >= rec_min.y && y <= rec_max.y);
}

bool checkConservativeCollision(octomap::OcTree* octomap, float x, float y) {

    // Iterate over all leaf nodes in the octree
    for (octomap::OcTree::leaf_iterator it = octomap->begin_leafs(); it != octomap->end_leafs(); ++it) {
        if (octomap->isNodeOccupied(*it) || (it->getOccupancy() < 0.5001 && it->getOccupancy() >= 0.5)) {
            octomap::point3d node_coord = it.getCoordinate();

            // Check if the x and y coordinates match
            if (std::fabs(node_coord.x() - x) < octomap->getResolution() / 2.0 &&
                std::fabs(node_coord.y() - y) < octomap->getResolution() / 2.0) {
                
                // If z coordinate is within bounds of the node, consider it occupied

                return true;
            }
        }
    }

    return false;
}

// Uses the improved version of Smit's algorithm to determine if the given ray will intersect
// the grid between tMin and tMax. This version causes an additional efficiency penalty,
// but takes into account the negative zero case.
// tMin and tMax are then updated to incorporate the new intersection values.
// Returns true if the ray intersects the grid, and false otherwise.
// See: http://www.cs.utah.edu/~awilliam/box/box.pdf

bool rayBoxIntersection(const Ray3D& ray, Vec3 minBound, Vec3 maxBound, float& tMin, float& tMax,
                        float t0, float t1) noexcept {
                            
    float tYMin, tYMax, tZMin, tZMax;
    const float x_inv_dir = 1 / ray.direction().x;
    if (x_inv_dir >= 0) {
        tMin = (minBound.x - ray.origin().x) * x_inv_dir;
        tMax = (maxBound.x - ray.origin().x) * x_inv_dir;
    } else {
        tMin = (maxBound.x - ray.origin().x) * x_inv_dir;
        tMax = (minBound.x - ray.origin().x) * x_inv_dir;
    }

    const float y_inv_dir = 1 / ray.direction().y;
    if (y_inv_dir >= 0) {
        tYMin = (minBound.y - ray.origin().y) * y_inv_dir;
        tYMax = (maxBound.y - ray.origin().y) * y_inv_dir;
    } else {
        tYMin = (maxBound.y - ray.origin().y) * y_inv_dir;
        tYMax = (minBound.y - ray.origin().y) * y_inv_dir;
    }

    if (tMin > tYMax || tYMin > tMax) return false;
    if (tYMin > tMin) tMin = tYMin;
    if (tYMax < tMax) tMax = tYMax;

    const float z_inv_dir = 1 / ray.direction().z;
    if (z_inv_dir >= 0) {
        tZMin = (minBound.z - ray.origin().z) * z_inv_dir;
        tZMax = (maxBound.z - ray.origin().z) * z_inv_dir;
    } else {
        tZMin = (maxBound.z - ray.origin().z) * z_inv_dir;
        tZMax = (minBound.z - ray.origin().z) * z_inv_dir;
    }

    if (tMin > tZMax || tZMin > tMax) return false;
    if (tZMin > tMin) tMin = tZMin;
    if (tZMax < tMax) tMax = tZMax;
    return (tMin < t1 && tMax > t0);
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

bool isVecInBounds(const Vec3& point, const Vec3 minBound, const Vec3 maxBound) {

    // Check if the point is within the bounds
    if (point.x >= minBound.x && point.x <= maxBound.x &&
        point.y >= minBound.y && point.y <= maxBound.y &&
        point.z >= minBound.z && point.z <= maxBound.z) {
        return true;  // Point is within bounds
    } else {
        return false; // Point is outside bounds
    }
}


class RayCastingNode
{
public:
    RayCastingNode()
        : nh(),
        pub_rcgrid(nh.advertise<nav_msgs::OccupancyGrid>("/cast_map", 10)),
        sub_ogrid(nh.subscribe("/projected_map", 10, &RayCastingNode::ogrid_callback, this)),
        pub_unknown_octree(nh.advertise<octomap_msgs::Octomap>("/unknown_octree", 10)),
        timer(nh.createTimer(ros::Duration(0.1), &RayCastingNode::main_loop, this)),
        pub_bounding_box(nh.advertise<visualization_msgs::Marker>("/cuboid_marker", 10)),
        listener(buffer),
        robot_pose(0.0,0.0,0.0),
        robot_heading(0.0,0.0,0.0),
        pub_debug(nh.advertise<geometry_msgs::PoseArray>("/debug_marker_ray", 100)),
        pub_best_pose(nh.advertise<visualization_msgs::Marker>("/best_pose", 10)),
        pub_rectangle(nh.advertise<visualization_msgs::MarkerArray>("/search_area", 10)),
        sub_octomap(nh.subscribe("/octomap_full", 10, &RayCastingNode::cuboid_callback, this)),
        cuboid_corner_sub(nh.subscribe("/match_cuboid_corners", 10, &RayCastingNode::corner_callback, this))
    {
        odom_sub.subscribe(nh, "/odom", 10);
        map_sub.subscribe(nh, "/projected_map", 10);
        //cuboid_corner_sub.subscribe(nh, "/match_cuboid_corners", 10);
        //octo_sub.subscribe(nh, "/octomap_full", 10);

        // Define the sync policy and create the synchronizer
        using sync_pol = message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>;
        sync.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(500), odom_sub, map_sub));
        sync->registerCallback(boost::bind(&RayCastingNode::callback, this, _1, _2));

        //using matched_cuboid_pol = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, octomap_msgs::Octomap>;
        //match_sync.reset(new message_filters::Synchronizer<matched_cuboid_pol>(matched_cuboid_pol(500), cuboid_corner_sub, octo_sub));
        //match_sync->registerCallback(boost::bind(&RayCastingNode::cuboid_callback, this, _1, _2));
    
    }

    void corner_callback(const geometry_msgs::PoseArray contents) {
        corner_msg.header = contents.header;
        corner_msg.poses = contents.poses;
    }

    void cuboid_callback(const octomap_msgs::Octomap::ConstPtr& map_msg) {
        //ROS_INFO("Beginning ray_cast");

        if (corner_msg.poses.size() != 5) {
            ROS_ERROR("four corners and one height must be received");
            return;
        }

        // calculate the 2D bounding rectangle
        double extension = 3; // m
        double rec_min_x = corner_msg.poses[MINMIN].position.x - extension;
        double rec_max_x = corner_msg.poses[MAXMAX].position.x + extension;
        double rec_min_y = corner_msg.poses[MINMIN].position.y - extension;
        double rec_max_y = corner_msg.poses[MAXMAX].position.y + extension;

        Vec3 rec_min_2d(corner_msg.poses[MINMIN].position.x, corner_msg.poses[MINMIN].position.y, 0.0);
        Vec3 rec_max_2d(corner_msg.poses[MAXMAX].position.x, corner_msg.poses[MAXMAX].position.y, 0.0);


        double rec_centre_x = rec_min_x + (rec_max_x - rec_min_x) / 2;
        double rec_centre_y = rec_min_y + (rec_max_y - rec_min_y) / 2;


        visualizeBoundingBox(rec_min_x, rec_max_x, rec_min_y, rec_max_y);

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        double resolution = octree->getResolution();
        octomap::OcTree* empty_octree = new octomap::OcTree(resolution);

        Vec3 minBound(corner_msg.poses[MINMIN].position.x, corner_msg.poses[MINMIN].position.y, 0.0);
        Vec3 maxBound(corner_msg.poses[MAXMAX].position.x, corner_msg.poses[MAXMAX].position.y, corner_msg.poses[MAXZ].position.z);
            
        if (octree) // can be NULL
        {

            octomap::point3d pmin(corner_msg.poses[MINMIN].position.x, corner_msg.poses[MINMIN].position.y, 0.0);
            octomap::point3d pmax(corner_msg.poses[MAXMAX].position.x, corner_msg.poses[MAXMAX].position.y, corner_msg.poses[MAXZ].position.z *0.5);
            octomap::point3d_list unknown_list;
            octree->getUnknownLeafCenters(unknown_list, pmin, pmax);
            auto oct_minBound = octomap::point3d(std::numeric_limits<double>::max(),
                                        std::numeric_limits<double>::max(),
                                        std::numeric_limits<double>::max());
            auto oct_maxBound = octomap::point3d(std::numeric_limits<double>::lowest(),
                                        std::numeric_limits<double>::lowest(),
                                        std::numeric_limits<double>::lowest());
            for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
                if (octree->isNodeOccupied(*it)) {
                    // If the node is occupied, mark it as occupied in the new octree
                    octomap::point3d node_coord = it.getCoordinate();
                    empty_octree->updateNode(node_coord, true);
                    

                    octomap::point3d node_point = it.getCoordinate();
                    // Update min and max bounds
                    oct_minBound.x() = std::min(oct_minBound.x(), node_point.x());
                    oct_minBound.y() = std::min(oct_minBound.y(), node_point.y());
                    oct_minBound.z() = std::min(oct_minBound.z(), node_point.z());
                    oct_maxBound.x() = std::max(oct_maxBound.x(), node_point.x());
                    oct_maxBound.y() = std::max(oct_maxBound.y(), node_point.y());
                    oct_maxBound.z() = std::max(oct_maxBound.z(), node_point.z());
                }
            }


            //ROS_INFO("octomap bounding box min:[%f, %f, %f] max: [%f, %f, %f]", oct_minBound.x(), oct_minBound.y(), oct_minBound.z(), oct_maxBound.x(), oct_maxBound.y(), oct_maxBound.z());
            //ROS_INFO("superglue bounding box min:[%f, %f, %f] max: [%f, %f, %f]", minBound.x, minBound.y, minBound.z, maxBound.x, maxBound.y, maxBound.z );

            Vec3 combined_min(0,0,0);
            Vec3 combined_max(0,0,0);
            calculateCombinedBoundingBox(oct_minBound, oct_maxBound, minBound, maxBound, combined_min, combined_max);
            //ROS_INFO("combined bounding box min:[%f, %f, %f] max: [%f, %f, %f]", combined_min.x, combined_min.y, combined_min.z, combined_max.x, combined_max.y, combined_max.z );

            for (const auto& point : unknown_list) {
                empty_octree->updateNode(point, static_cast<float>(0.0));
                octomap::OcTreeNode* node = empty_octree->search(point);

                if (node) {
                    // If node is found, update its value with a likelihood of 0.5
                    // Note: The OcTree library uses log-odds for probability. 
                    // A likelihood of 0.5 corresponds to a log-odds of 0.0.
                    node->setValue(0.0); // 0.0 log-odds represents 0.5 probability
                }
            }

            
            octomap_msgs::Octomap bmap_msg;
            octomap_msgs::fullMapToMsg(*empty_octree, bmap_msg);

            
            bmap_msg.header.stamp = ros::Time::now();
            bmap_msg.header.frame_id = "map";
            ROS_INFO("New map published!");
            pub_unknown_octree.publish(bmap_msg);

            // wait for odom to update --
            if (robot_heading.x == 0.0 && robot_heading.y == 0.0) {
                ROS_INFO("waiting for odom to update..");
                return;
            } 
            
            int range = 10;
            int horizontal_fov = 10;
            float step_size = 1;

            int best_info_gain{0};
            float best_x(0.0);  
            float best_y(0.0);

            float fov_rad = horizontal_fov * 0.0174533f;
            int horizontal_resolution = 5;
            float fov_step = fov_rad / (horizontal_resolution - 1);

            geometry_msgs::PoseArray pose_array;
            pose_array.header.stamp = ros::Time::now();
            pose_array.header.frame_id = "map";

            // Compute number of steps in x and y directions
            int num_steps_x = std::ceil((rec_max_x - rec_min_x) / step_size);
            int num_steps_y = std::ceil((rec_max_y - rec_min_y) / step_size);

            // Total number of iterations
            int total_iterations = num_steps_x * num_steps_y;
            //ROS_INFO("%i", total_iterations);

            for (int i = 0; i < total_iterations; ++i) {
                // Compute x and y from the flat index
                float y = rec_min_y + (i / num_steps_x) * step_size;
                float x = rec_min_x + (i % num_steps_x) * step_size;

                float dy = rec_centre_y - y;
                float dx = rec_centre_x - x;
                float mag = std::sqrt(dy * dy + dx * dx);
                float dy_norm = (mag != 0.0) ? (dy / mag) : 0.0;
                float dx_norm = (mag != 0.0) ? (dx / mag) : 0.0;

                Vec3 origin(x , y, 0.29); // x and z of the camera
                Vec3 direction(dx_norm, dy_norm, 0.0);
                Ray3D ray(origin, direction);

                //if ( 5.6 < x && x < 5.8 && y < -2.7&& y > -2.8) {  // 
                    int current_info_gain = ray_cast_3d(ray, empty_octree, range, combined_min, combined_max, pose_array);

                    
                    for (int idx = 0; idx < horizontal_resolution * horizontal_resolution; idx++) {
                        // Compute horizontal and vertical indices
                        int i = idx % horizontal_resolution;
                        int j = idx / horizontal_resolution;

                        // Calculate the ray direction for each grid point
                        float h_angle = -fov_rad / 2 + i * fov_step;
                        float v_angle = -fov_rad / 2 + j * fov_step;

                        // Rotate the direction vector to get the new ray direction
                        Vec3 new_direction = direction.normalize()
                            .rotateY(h_angle)
                            .rotateZ(v_angle);

                        // Create a new ray with the updated direction
                        Ray3D ray(origin, new_direction);

                        // Cast the ray and count the intersections
                        int count = ray_cast_3d(ray, empty_octree, range, combined_min, combined_max, pose_array);

                        // Accumulate the information gain
                        current_info_gain += static_cast<float>(count);

                        
                        if (!checkConservativeCollision(empty_octree, x, y)) {
                            if (current_info_gain > best_info_gain) {
                                best_info_gain = current_info_gain;
                                best_x = x;
                                best_y = y;
                            }
                        }
                    }
                    
                    /** 
                    for (int i = 1; i <= horizontal_fov; i++) {
                        Vec3 new_direction_p = direction.normalize().rotateY(i*0.0174533*2);
                        Vec3 new_direction_n = direction.normalize().rotateY(-i*0.0174533);
                        Vec3 new_direction_r = direction.normalize().rotateZ(i*0.0174533);
                        Vec3 new_direction_l = direction.normalize().rotateZ(-i*0.0174533);
                        Ray3D ray_p(origin, new_direction_p);
                        Ray3D ray_n(origin, new_direction_n);
                        Ray3D ray_r(origin, new_direction_r);
                        Ray3D ray_l(origin, new_direction_l);
                        int countp = ray_cast_3d(ray_p, empty_octree, range, combined_min, combined_max, pose_array);
                        int countn = ray_cast_3d(ray_n, empty_octree, range, combined_min, combined_max, pose_array);
                        int countr = ray_cast_3d(ray_r, empty_octree, range, combined_min, combined_max, pose_array);
                        int countl = ray_cast_3d(ray_l, empty_octree, range, combined_min, combined_max, pose_array);
                        current_info_gain += countp;
                        current_info_gain += countn;
                        current_info_gain += countr;
                        current_info_gain += countl;
                    }  */

                    if (current_info_gain > best_info_gain) {
                        best_info_gain = current_info_gain;
                        best_x = x;
                        best_y = y;
                    }

                    ROS_INFO("current best info gain %i", best_info_gain);
                    double yaw = atan2(dy_norm, dx_norm); // Calculate yaw based on direction
                    if (std::isnan(yaw)) {
                        yaw = 0.0; // Set a default value if yaw is NaN
                    }

                    // Create quaternion from roll, pitch, and yaw
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, yaw);
                    q.normalize(); // Normalize the quaternion

                    geometry_msgs::Pose pose;
                    pose.position.x = x;
                    pose.position.y = y;
                    pose.position.z = 0.29;
                    pose.orientation.x = q.x();
                    pose.orientation.y = q.y();
                    pose.orientation.z = q.z();
                    pose.orientation.w = q.w();        
                    pose_array.poses.push_back(pose); 
                //}

                
                
                /** 
                for (int i = 1; i <= horizontal_fov; i++) {
                    Vec3 new_direction_p = direction.normalize().rotateY(i*0.0174533*2);
                    //Vec3 new_direction_n = direction.normalize().rotateY(-i*0.0174533);
                    //Vec3 new_direction_r = direction.normalize().rotateZ(i*0.0174533);
                    //Vec3 new_direction_l = direction.normalize().rotateZ(-i*0.0174533);
                    Ray3D ray_p(origin, new_direction_p);
                    //Ray3D ray_n(origin, new_direction_n);
                    //Ray3D ray_r(origin, new_direction_r);
                    //Ray3D ray_l(origin, new_direction_l);
                    int countp = ray_cast_3d(ray_p, octree, range, minBound, maxBound);
                    //int countn = ray_cast_3d(ray_n, octree, range, minBound, maxBound);
                    //int countr = ray_cast_3d(ray_r, octree, range, minBound, maxBound);
                    //int countl = ray_cast_3d(ray_l, octree, range, minBound, maxBound);
                    current_info_gain += countp;
                    //current_info_gain += countn;
                    //current_info_gain += countr;
                    //current_info_gain += countl;
                }  **/

            }

            
            pub_debug.publish(pose_array);

            /**

            // Iterate through all cells within the bounding box
            for (int y = rec_min_y; y <= rec_max_y; y += step_size) {
                for (int x = rec_min_x; x <= rec_max_x; x += step_size) {
                    if (!isPointIn2DRectangle(x, y, rec_min_2d, rec_max_2d)) {
                        Vec3 origin(x, y, 0.0);
                        int dy = rec_centre_y - y;
                        int dx = rec_centre_x - x;
                        float mag = std::sqrt(dy * dy + dx * dx);
                        float dy_norm = dy / mag;
                        float dx_norm = dx / mag;

                        Vec3 direction(dx_norm, dy_norm, 0.0);
                        Ray3D ray(origin, direction);
                        int current_info_gain = ray_cast_3d(ray, octree, range, minBound, maxBound);

                        
                        for (int i = 1; i <= horizontal_fov; i++) {
                            Vec3 new_direction_p = direction.normalize().rotateY(i*0.0174533);
                            //Vec3 new_direction_n = direction.normalize().rotateY(-i*0.0174533);
                            //Vec3 new_direction_r = direction.normalize().rotateZ(i*0.0174533);
                            //Vec3 new_direction_l = direction.normalize().rotateZ(-i*0.0174533);
                            Ray3D ray_p(origin, new_direction_p);
                            //Ray3D ray_n(origin, new_direction_n);
                            //Ray3D ray_r(origin, new_direction_r);
                            //Ray3D ray_l(origin, new_direction_l);
                            int countp = ray_cast_3d(ray_p, octree, range, minBound, maxBound);
                            //int countn = ray_cast_3d(ray_n, octree, range, minBound, maxBound);
                            //int countr = ray_cast_3d(ray_r, octree, range, minBound, maxBound);
                            //int countl = ray_cast_3d(ray_l, octree, range, minBound, maxBound);
                            current_info_gain += countp;
                            //current_info_gain += countn;
                            //current_info_gain += countr;
                            //current_info_gain += countl;
                        }  

                        ROS_INFO("current info gain %i", current_info_gain);

                        if (current_info_gain > best_info_gain) {
                            best_info_gain = current_info_gain;
                            best_x = x;
                            best_y = y;
                        }

                        ROS_INFO("best info gain %i", best_info_gain);
                        ROS_INFO("best x %f", best_x);
                        ROS_INFO("best y %f", best_x);
                    }

                }
            }  */

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

            ros::Duration(5.0).sleep();

        delete tree;        // Free the original tree
        delete empty_octree;  // Free the empty tree when done
        
        }
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

    int ray_cast_3d(const Ray3D& ray, octomap::OcTree* octree, float max_range, Vec3 minBound, Vec3 maxBound, geometry_msgs::PoseArray& pose_array) {
        
        auto grid_res = octree->getResolution();
        float tMin;
        float tMax;
        const bool ray_intersects_grid = rayBoxIntersection(ray, minBound , maxBound, tMin, tMax, 0, max_range);

        tMin = MAX(tMin, 0);
        tMax = MIN(tMax, max_range);

        if (!ray_intersects_grid) {
            return 0;
        }  else {
            
            geometry_msgs::Pose pose1;
            pose1.position.x = ray.point_at_parameter(tMin).x;
            pose1.position.y = ray.point_at_parameter(tMin).y;
            pose1.position.z = ray.point_at_parameter(tMin).z;
            pose1.orientation.x = 0.0;
            pose1.orientation.y = 0.0;
            pose1.orientation.z = 0.0;
            pose1.orientation.w = 1.0;        
            //pose_array.poses.push_back(pose1);   
            geometry_msgs::Pose pose2; 
            pose2.position.x = ray.point_at_parameter(tMax).x;
            pose2.position.y = ray.point_at_parameter(tMax).y;
            pose2.position.z = ray.point_at_parameter(tMax).z;
            pose2.orientation.x = 0.0;
            pose2.orientation.y = 0.0;
            pose2.orientation.z = 0.0;
            pose2.orientation.w = 1.0;        
            //pose_array.poses.push_back(pose2);   
        }
        
        /**
        if (isVecInBounds(ray.origin(), minBound, maxBound)) {
            ROS_INFO("wth");
            tMin = 0;
        } */
        
        if (tMin == 0 && tMax > 1.0) {
            // So that the robot doesn't look at the object straight upfront?
            tMin = 1.0;
        }
        const Vec3 ray_start = ray.origin() + ray.direction() * tMin;
        const Vec3 ray_end = ray.origin() + ray.direction() * tMax;

        
        int current_X_index = MAX(1, std::ceil((ray_start.x - minBound.x) / grid_res));
        const int end_X_index = MAX(1, std::ceil((ray_end.x - minBound.x) / grid_res));

        int stepX;
        float tDeltaX;
        float tMaxX;
        if (ray.direction().x > 0.0) {
            stepX = 1;
            tDeltaX = grid_res / ray.direction().x;
            tMaxX = tMin + (minBound.x + current_X_index * grid_res
                            - ray_start.x) / ray.direction().x;
        } else if (ray.direction().x < 0.0) {
            stepX = -1;
            tDeltaX = grid_res / -ray.direction().x;
            const int previous_X_index = current_X_index - 1;
            tMaxX = tMin + (minBound.x + previous_X_index * grid_res
                            - ray_start.x) / ray.direction().x;
        } else {
            stepX = 0;
            tDeltaX = tMax;
            tMaxX = tMax;
        }

        int current_Y_index = MAX(1, std::ceil((ray_start.y - minBound.y) / grid_res));
        const int end_Y_index = MAX(1, std::ceil((ray_end.y - minBound.y) / grid_res));
        int stepY;
        float tDeltaY;
        float tMaxY;
        if (ray.direction().y > 0.0) {
            stepY = 1;
            tDeltaY = grid_res / ray.direction().y;
            tMaxY = tMin + (minBound.y + current_Y_index * grid_res
                            - ray_start.y) / ray.direction().y;
        } else if (ray.direction().y < 0.0) {
            stepY= -1;
            tDeltaY = grid_res / -ray.direction().y;
            const int previous_Y_index = current_Y_index - 1;
            tMaxY = tMin + (minBound.y + previous_Y_index * grid_res
                            - ray_start.y) / ray.direction().y;
        } else {
            stepY = 0;
            tDeltaY = tMax;
            tMaxY = tMax;
        }

        int current_Z_index = MAX(1, std::ceil((ray_start.z - minBound.z) / grid_res));
        const int end_Z_index = MAX(1, std::ceil((ray_end.z - minBound.z) / grid_res));
        int stepZ;
        float tDeltaZ;
        float tMaxZ;
        if (ray.direction().z > 0.0) {
            stepZ = 1;
            tDeltaZ = grid_res / ray.direction().z;
            tMaxZ = tMin + (minBound.z + current_Z_index * grid_res
                            - ray_start.z) / ray.direction().z;
        } else if (ray.direction().z < 0.0) {
            stepZ = -1;
            tDeltaZ = grid_res / -ray.direction().z;
            const int previous_Z_index = current_Z_index - 1;
            tMaxZ = tMin + (minBound.z + previous_Z_index * grid_res
                            - ray_start.z) / ray.direction().z;
        } else {
            stepZ = 0;
            tDeltaZ = tMax;
            tMaxZ = tMax;
        }

        // current_X_index != end_X_index & current_Y_index != end_Y_index & current_Z_index != end_Z_index
        while (current_X_index != end_X_index && current_Y_index != end_Y_index && current_Z_index != end_Z_index) {
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                // X-axis traversal.
                current_X_index += stepX;
                tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                // Y-axis traversal.
                current_Y_index += stepY;
                tMaxY += tDeltaY;
            } else {
                // Z-axis traversal.
                current_Z_index += stepZ;
                tMaxZ += tDeltaZ;
            }
            double worldX = minBound.x + (current_X_index -1) * grid_res + grid_res / 2;
            double worldY = minBound.y + (current_Y_index -1) * grid_res + grid_res / 2;
            double worldZ = minBound.z + (current_Z_index -1) * grid_res + grid_res / 2;
            octomap::point3d match(worldX, worldY, worldZ);
            // Debug output for current indices and world coordinates
            
            if (isPointInBounds(match, minBound, maxBound)) {

            
            octomap::OcTreeNode* node = octree->search(match);
                if (node) {
                    // Update the node at the specified point

                    if (node->getOccupancy() < 0.5001 && node->getOccupancy() >= 0.5) {
                        geometry_msgs::Pose hit; 
                        hit.position.x = worldX;
                        hit.position.y = worldY;
                        hit.position.z = worldZ;
                        hit.orientation.x = 0.0;
                        hit.orientation.y = 0.0;
                        hit.orientation.z = 0.3;
                        hit.orientation.w = 1.0;        
                        //pose_array.poses.push_back(hit);   
                        return 1;
                    } else if (node->getOccupancy() > 0.5) {
                        return 0;
                    }

                } else {
                    //ROS_INFO("No node found; unexplored?");
                } 
            }
        }

        return 0;
    }

    void visualizeBoundingBox(double min_x, double max_x, double min_y, double max_y)
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
            pub_rectangle.publish(markers);
        }

    void ogrid_callback(const nav_msgs::OccupancyGrid& ogrid)
    {

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
    //message_filters::Subscriber<geometry_msgs::PoseArray> cuboid_corner_sub;
    //message_filters::Subscriber<octomap_msgs::Octomap> octo_sub;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>>> sync;
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
    geometry_msgs::PoseArray corner_msg;
    ros::Subscriber cuboid_corner_sub;
    

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_casting_node");
    RayCastingNode node;
    ros::spin();
    return 0;
}
