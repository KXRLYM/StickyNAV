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

/** 
void processChunk(int start_y, int end_y, int rec_min_x, int rec_max_x, int step_size,
                octomap::OcTree* octree, double range,
                const Vec3& minBound, const Vec3& maxBound, int horizontal_fov,
                std::atomic<int>& best_info_gain, std::atomic<int>& best_x,
                std::atomic<int>& best_y, float rec_centre_x, float rec_centre_y) {
    for (int y = start_y; y <= end_y; y += step_size) {
        for (int x = rec_min_x; x <= rec_max_x; x += step_size) {
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
                Vec3 new_direction_p = direction.normalize().rotateY(i * 0.0174533);
                Vec3 new_direction_n = direction.normalize().rotateY(-i * 0.0174533);
                Vec3 new_direction_r = direction.normalize().rotateZ(i * 0.0174533);
                Vec3 new_direction_l = direction.normalize().rotateZ(-i * 0.0174533);
                Ray3D ray_p(origin, new_direction_p);
                Ray3D ray_n(origin, new_direction_n);
                Ray3D ray_r(origin, new_direction_r);
                Ray3D ray_l(origin, new_direction_l);
                int countp = ray_cast_3d(ray_p, octree, range, minBound, maxBound);
                int countn = ray_cast_3d(ray_n, octree, range, minBound, maxBound);
                int countr = ray_cast_3d(ray_r, octree, range, minBound, maxBound);
                int countl = ray_cast_3d(ray_l, octree, range, minBound, maxBound);
                current_info_gain += countp + countn + countr + countl;
            }

            // Update best_info_gain and coordinates in a thread-safe manner
            if (current_info_gain > best_info_gain) {
                best_info_gain = current_info_gain;
                best_x = x;
                best_y = y;
            }
        }
    }
}*/

#define MAX(a,b) ((a > b ? a : b))

bool isPointIn2DRectangle(const octomap::point3d& point, const geometry_msgs::Pose min_point, const geometry_msgs::Pose max_point) {
    return (point.x() >= min_point.position.x && point.x() <= max_point.position.x &&
            point.y() >= min_point.position.y && point.y() <= max_point.position.y);
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
    ROS_INFO("Octree Bounds %f, %f, %f", minBound.x, minBound.y, minBound.z);
    ROS_INFO("Octree Bounds %f, %f, %f", maxBound.x, maxBound.y, maxBound.z);
    
    // Check if the point is within the bounds
    if (point.x() >= minBound.x && point.x() <= maxBound.x &&
        point.y() >= minBound.y && point.y() <= maxBound.y &&
        point.z() >= minBound.z && point.z() <= maxBound.z) {
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
        pub_debug(nh.advertise<visualization_msgs::Marker>("/debug_marker_ray", 10)),
        pub_best_pose(nh.advertise<visualization_msgs::Marker>("/best_pose", 10)),
        pub_rectangle(nh.advertise<visualization_msgs::MarkerArray>("/search_area", 10))
    {
        odom_sub.subscribe(nh, "/odom", 10);
        map_sub.subscribe(nh, "/test_map", 10);
        cuboid_corner_sub.subscribe(nh, "/match_cuboid_corners", 10);
        octo_sub.subscribe(nh, "/octomap_full", 10);

        // Define the sync policy and create the synchronizer
        using sync_pol = message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>;
        sync.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(500), odom_sub, map_sub));
        sync->registerCallback(boost::bind(&RayCastingNode::callback, this, _1, _2));

        //using sync_pol1 = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, nav_msgs::OccupancyGrid>;
        //rectangle_sync.reset(new message_filters::Synchronizer<sync_pol1>(sync_pol1(500), rectangle_sub, map_sub));
        //rectangle_sync->registerCallback(boost::bind(&RayCastingNode::box_callback, this, _1, _2));

        using matched_cuboid_pol = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, octomap_msgs::Octomap>;
        match_sync.reset(new message_filters::Synchronizer<matched_cuboid_pol>(matched_cuboid_pol(500), cuboid_corner_sub, octo_sub));
        match_sync->registerCallback(boost::bind(&RayCastingNode::cuboid_callback, this, _1, _2));
    }

    void cuboid_callback(const geometry_msgs::PoseArray::ConstPtr& corner_msg, const octomap_msgs::Octomap::ConstPtr& map_msg) {
        ROS_INFO("Beginning ray_cast");

        // calculate the 2D bounding rectangle
        double extension = 4; // m
        double rec_min_x = corner_msg->poses[MINMIN].position.x - extension;
        double rec_max_x = corner_msg->poses[MAXMAX].position.x + extension;
        double rec_min_y = corner_msg->poses[MINMIN].position.y - extension;
        double rec_max_y = corner_msg->poses[MAXMAX].position.y + extension;

        int rec_centre_x = rec_min_x + (rec_max_x - rec_min_x) / 2;
        int rec_centre_y = rec_min_y + (rec_max_y - rec_min_y) / 2;

        Vec3 minBound(corner_msg->poses[MINMIN].position.x, corner_msg->poses[MINMIN].position.y, 0.0);
        Vec3 maxBound(corner_msg->poses[MAXMAX].position.x, corner_msg->poses[MAXMAX].position.y, corner_msg->poses[MAXZ].position.z);

        visualizeBoundingBox(rec_min_x, rec_max_x, rec_min_y, rec_max_y);

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        double resolution = octree->getResolution();
        octomap::OcTree* empty_octree = new octomap::OcTree(resolution);
            
        if (octree) // can be NULL
        {
            if (corner_msg->poses.size() != 5) {
                ROS_ERROR("four corners and one height must be received");
                return;
            }

            octomap::point3d pmin(corner_msg->poses[MINMIN].position.x, corner_msg->poses[MINMIN].position.y, 0.0);
            octomap::point3d pmax(corner_msg->poses[MAXMAX].position.x, corner_msg->poses[MAXMAX].position.y, corner_msg->poses[MAXZ].position.z *0.8);
            octomap::point3d_list unknown_list;
            octree->getUnknownLeafCenters(unknown_list, pmin, pmax);

            for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
                if (octree->isNodeOccupied(*it)) {
                    // If the node is occupied, mark it as occupied in the new octree
                    empty_octree->updateNode(it.getCoordinate(), true);
                }
            }

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
            /** 
            for (const auto& point : unknown_list) {
                empty_octree->updateNode(point, true); // Mark the point as occupied
                
            }
            octomap_msgs::Octomap bmap_msg;
            octomap_msgs::binaryMapToMsg(*empty_octree, bmap_msg);



            for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
                if (isPointIn2DRectangle(it.getCoordinate(), corner_msg->poses[MINMIN], corner_msg->poses[MAXMAX])) {
                }
            }  */
            
            bmap_msg.header.stamp = ros::Time::now();
            bmap_msg.header.frame_id = "map";
            pub_unknown_octree.publish(bmap_msg);

            // wait for odom to update --
            if (robot_heading.x == 0.0 && robot_heading.y == 0.0) {
                ROS_INFO("waiting for odom to update..");
                return;
            } 
            
            int range = 10;
            int horizontal_fov = 4;
            float step_size = 2;
              /**
            std::atomic<int> best_info_gain{0};
            std::atomic<int> best_x{0};
            std::atomic<int> best_y{0};

            // Number of threads to use
            size_t num_threads = std::thread::hardware_concurrency();
            std::vector<std::thread> threads;

            int range_y = rec_max_y - rec_min_y;
            int chunk_size = range_y / num_threads;

            for (size_t i = 0; i < num_threads; ++i) {
                int start_y = rec_min_y + i * chunk_size;
                int end_y = (i == num_threads - 1) ? rec_max_y : start_y + chunk_size - 1;

                threads.emplace_back(processChunk, start_y, end_y, rec_min_x, rec_max_x, step_size,
                                    octree, range, minBound, maxBound, horizontal_fov,
                                    std::ref(best_info_gain), std::ref(best_x), std::ref(best_y), rec_centre_x, rec_centre_y);
            }

            // Join all threads
            for (auto& thread : threads) {
                thread.join();
            }

            ROS_INFO("Best info gain: %i at x: %i, y: %i", best_info_gain.load(), best_x.load(), best_y.load());

          
            */
            int best_info_gain{0};
            float best_x(0.0);  
            float best_y(0.0);

            // Iterate through all cells within the bounding box
            for (int y = rec_min_y; y <= rec_max_y; y += step_size) {
                for (int x = rec_min_x; x <= rec_max_x; x += step_size) {
                    

                    Vec3 origin(x, y, 0.0);
                    int dy = rec_centre_y - y;
                    int dx = rec_centre_x - x;
                    float mag = std::sqrt(dy * dy + dx * dx);
                    float dy_norm = dy / mag;
                    float dx_norm = dx / mag;

                    Vec3 direction(dx_norm, dy_norm, 0.0);
                    Ray3D ray(origin, direction);
                    int current_info_gain = ray_cast_3d(ray, octree, range, minBound, maxBound);

                    /**
                    for (int i = 1; i <= horizontal_fov; i++) {
                        Vec3 new_direction_p = direction.normalize().rotateY(i*0.0174533);
                        Vec3 new_direction_n = direction.normalize().rotateY(-i*0.0174533);
                        Vec3 new_direction_r = direction.normalize().rotateZ(i*0.0174533);
                        Vec3 new_direction_l = direction.normalize().rotateZ(-i*0.0174533);
                        Ray3D ray_p(origin, new_direction_p);
                        Ray3D ray_n(origin, new_direction_n);
                        Ray3D ray_r(origin, new_direction_r);
                        Ray3D ray_l(origin, new_direction_l);
                        int countp = ray_cast_3d(ray_p, octree, range, minBound, maxBound);
                        int countn = ray_cast_3d(ray_n, octree, range, minBound, maxBound);
                        int countr = ray_cast_3d(ray_r, octree, range, minBound, maxBound);
                        int countl = ray_cast_3d(ray_l, octree, range, minBound, maxBound);
                        current_info_gain += countp;
                        current_info_gain += countn;
                        current_info_gain += countr;
                        current_info_gain += countl;
                    }  */



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

            visualization_msgs::Marker best;
            best.header.frame_id = "map";
            best.header.stamp = ros::Time::now();
            best.ns = "best";
            best.id = 78;
            best.type = visualization_msgs::Marker::SPHERE;
            best.action = visualization_msgs::Marker::ADD;

            best.pose.position.x = best_x;
            best.pose.position.y = best_y;
            best.pose.position.z = 0.0;
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
        
            /** 
            Ray3D ray(robot_pose, robot_heading.normalize());
            Vec3 minBound(corner_msg->poses[MINMIN].position.x, corner_msg->poses[MINMIN].position.y, 0.0);
            Vec3 maxBound(corner_msg->poses[MAXMAX].position.x, corner_msg->poses[MAXMAX].position.y, corner_msg->poses[MAXZ].position.z);

            int total_count(0);
            int countm = ray_cast_3d(ray, octree, range, minBound, maxBound);
            total_count += countm;
            
            for (int i = 1; i <= horizontal_fov; i++) {
                Vec3 new_direction_p = robot_heading.normalize().rotateY(i*0.0174533/2);
                Vec3 new_direction_n = robot_heading.normalize().rotateY(-i*0.0174533/2);
                Vec3 new_direction_r = robot_heading.normalize().rotateZ(i*0.0174533/2);
                Vec3 new_direction_l = robot_heading.normalize().rotateZ(-i*0.0174533/2);
                Ray3D ray_p(robot_pose, new_direction_p);
                Ray3D ray_n(robot_pose, new_direction_n);
                Ray3D ray_r(robot_pose, new_direction_r);
                Ray3D ray_l(robot_pose, new_direction_l);
                int countp = ray_cast_3d(ray_p, octree, range, minBound, maxBound);
                int countn = ray_cast_3d(ray_n, octree, range, minBound, maxBound);
                int countr = ray_cast_3d(ray_r, octree, range, minBound, maxBound);
                int countl = ray_cast_3d(ray_l, octree, range, minBound, maxBound);
                total_count += countp;
                total_count += countn;
                total_count += countr;
                total_count += countl;
            }
            ROS_INFO("UNEXPLORED COUNT %i", total_count);
        } 
        
    } */

        delete tree;        // Free the original tree
        delete empty_octree;  // Free the empty tree when done
        
        }
    }

    

    void box_callback(const geometry_msgs::PoseArray::ConstPtr& msg, const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        /**/


        // Check if the data size is correct (2 points for the bounding box)
        if (msg->poses.size() != 2) {
            ROS_WARN("Received incorrect data size for bounding box.");
            return;
        }

        // Extract the coordinates from the message
        float min_x = msg->poses[0].position.x;
        float min_y = msg->poses[0].position.y;
        float max_x = msg->poses[1].position.x;
        float max_y = msg->poses[1].position.y;

        // Convert float coordinates to double for more precise calculations
        double min_x_d = static_cast<double>(min_x);
        double min_y_d = static_cast<double>(min_y);
        double max_x_d = static_cast<double>(max_x);
        double max_y_d = static_cast<double>(max_y);

        // Get the grid information
        double resolution = map_msg->info.resolution;
        double origin_x = map_msg->info.origin.position.x;
        double origin_y = map_msg->info.origin.position.y;
        int width = map_msg->info.width;
        int height = map_msg->info.height;

        // Calculate the bounding box in grid cells
        int x_min = std::max(0, static_cast<int>((min_x_d - origin_x) / resolution));
        int y_min = std::max(0, static_cast<int>((min_y_d - origin_y) / resolution));
        int x_max = std::min(width - 1, static_cast<int>((max_x_d - origin_x) / resolution));
        int y_max = std::min(height - 1, static_cast<int>((max_y_d - origin_y) / resolution));

        // Calculate the center of the bounding box
        int centre_x = x_min + (x_max - x_min) / 2;
        int centre_y = y_min + (y_max - y_min) / 2;

        // Create a new occupancy grid to hold the modified data
        auto pgrid = std::make_unique<nav_msgs::OccupancyGrid>();
        pgrid->info = map_msg->info;
        pgrid->info.width = width; // Set to original width
        pgrid->info.height = height; // Set to original height
        pgrid->info.origin.position.x = map_msg->info.origin.position.x;
        pgrid->info.origin.position.y = map_msg->info.origin.position.y;
        pgrid->header = map_msg->header;

        // Copy original data to pgrid_data
        std::vector<int8_t> pgrid_data(map_msg->data);

        // Iterate through all cells within the bounding box
        for (int y = y_min; y <= y_max; ++y) {
            for (int x = x_min; x <= x_max; ++x) {
                if (x == x_min && y == y_min ) {
                    int index = y * width + x;
                    // Mark cells inside the bounding box
                    // Perform ray casting if needed
                    Vec2 origin(x, y);
                    int dy = centre_y - y;
                    int dx = centre_x - x;
                    float mag = std::sqrt(dy * dy + dx * dx);
                    float dy_norm = dy / mag;
                    float dx_norm = dx / mag;
                    int horizontal_fov = 60;
                    int max_range = 7;
                    Vec2 direction(dx_norm, dy_norm);
                    Ray ray(origin, direction);
                    ray_cast(ray, *map_msg, max_range, pgrid_data);

                    for (int i = 1; i <= horizontal_fov; i++) {
                        Vec2 new_direction_p = direction.rotate_vector(i*0.0174533/2);
                        Vec2 new_direction_n = direction.rotate_vector(-i*0.0174533/2);
                        Ray ray_p(origin, new_direction_p);
                        Ray ray_n(origin, new_direction_n);
                        ray_cast(ray_p, *map_msg, max_range, pgrid_data);
                        ray_cast(ray_n, *map_msg, max_range, pgrid_data);
                    }


                    pgrid_data[index] = 99;
                }
            }
        }

        // Update the new occupancy grid with modified data
        pgrid->data = pgrid_data;

        // Publish the updated occupancy grid
        //pub_rcgrid.publish(*pgrid);
    }

    

    void callback(const nav_msgs::Odometry::ConstPtr& odom_msg, const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        ROS_INFO("Updating odom");

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
        ROS_INFO("headings %f, %f", direction.x, direction.y);
        
        robot_heading.x = direction.x;
        robot_heading.y = direction.y;
        robot_heading.z = 0.0;

        /**
        float horizontal_fov = 86; // 86 degree in [rads]
        float max_range = 5; // in [m] sensor max range is 50;

        int32_t dx, dy;
        dx = std::abs(robot_x - map_origin_x) / res;
        dy = std::abs(robot_y - map_origin_y) / res;
        int index = dy * map_msg->info.width + dx;

        Vec2 origin(dx, dy); // in [cells]
        Ray ray(origin, direction);
        std::vector<int8_t> pgrid_data(map_msg->info.width * map_msg->info.height, 50); // Default value 50
        
        
        for (unsigned int i = 1; i <= horizontal_fov; i++) {
            Vec2 direction_p(cos(yaw+(i*0.0174533/2)), sin(yaw+(i*0.0174533/2)));
            Vec2 direction_n(cos(yaw-(i*0.0174533/2)), sin(yaw-(i*0.0174533/2)));
            Ray ray_p(origin, direction_p);
            Ray ray_n(origin, direction_n);
            
            ray_cast(ray_p, *map_msg, max_range, pgrid_data);
            ray_cast(ray_n, *map_msg, max_range, pgrid_data);
            
        }
        ray_cast(ray, *map_msg, max_range, pgrid_data);
        

        auto pgrid = std::make_unique<nav_msgs::OccupancyGrid>();
        pgrid->info = map_msg->info;
        pgrid->header = map_msg->header;

        pgrid_data[0] = 3;
        pgrid_data[1] = 3;
        pgrid_data[index] = 70;
        pgrid_data[index-1] = 70;
        pgrid_data[index+1] = 70;
        pgrid_data[index-map_msg->info.width] = 70;
        pgrid_data[index+map_msg->info.width] = 70;

        pgrid->data = pgrid_data;
        pub_rcgrid.publish(*pgrid);*/

    }

    void ray_cast(const Ray& ray, const nav_msgs::OccupancyGrid& grid, float max_range, std::vector<int8_t>& marked_grid) {
        float grid_res = grid.info.resolution;
        int stepX, stepY;
        float tDeltaX, tDeltaY;
        float tMaxX, tMaxY;

        const Vec2 ray_start = ray.origin() + ray.direction() * 0.0;
        const Vec2 ray_end = ray.origin() + ray.direction() * std::ceil(max_range / grid_res); // in [cells]

        int32_t current_X_index = static_cast<int32_t>(ray.origin().x); // in [cells]
        int32_t current_Y_index = static_cast<int32_t>(ray.origin().y); // in [cells]

        if (ray.direction().x > 0.0) {
            stepX = 1;
            tDeltaX = grid_res / ray.direction().x; // how much to go to move out of the cell
            tMaxX = (std::ceil(ray.origin().x / grid_res) * grid_res - ray.origin().x) / ray.direction().x;
        } else if (ray.direction().x < 0.0) {
            stepX = -1;
            tDeltaX = grid_res / -ray.direction().x;
            tMaxX = (ray.origin().x - std::floor(ray.origin().x / grid_res) * grid_res) / -ray.direction().x;
        } else {
            stepX = 0;
            tMaxX = std::numeric_limits<float>::infinity();
        }

        if (ray.direction().y > 0.0) {
            stepY = 1;  
            tDeltaY = grid_res / ray.direction().y;
            tMaxY = (std::ceil(ray.origin().y / grid_res) * grid_res - ray.origin().y) / ray.direction().y;
        } else if (ray.direction().y < 0.0) {
            stepY = -1;
            tDeltaY = grid_res / -ray.direction().y;
            tMaxY = (ray.origin().y - std::floor(ray.origin().y / grid_res) * grid_res) / -ray.direction().y;
        } else {
            stepY = 0;
            tMaxY = std::numeric_limits<float>::infinity();
        }

        while (tMaxX < max_range && tMaxY < max_range) {
            int32_t index = current_Y_index * grid.info.width + current_X_index;
            if (grid.data[index] == OCC_GRID_OCCUPIED) {
                break;
            }
            if (current_Y_index > grid.info.height || current_X_index > grid.info.width) {
                break;
            }
            if (bound_check(index, grid.info.width, grid.info.height)) {
                marked_grid[index] = 130;
                if (grid.data[index] == 125) {
                    marked_grid[index] = 200;
                }
            }

            if (tMaxX < tMaxY) {
                tMaxX += tDeltaX;
                current_X_index += stepX;
            } else {
                tMaxY += tDeltaY;
                current_Y_index += stepY;
            }
        }

    }

    int ray_cast_3d(const Ray3D& ray, octomap::OcTree* octree, float max_range, Vec3 minBound, Vec3 maxBound) {
        
        auto grid_res = octree->getResolution();
        float tMin;
        float tMax;
        const bool ray_intersects_grid = rayBoxIntersection(ray, minBound , maxBound, tMin, tMax, 0, max_range);


        if (!ray_intersects_grid) {
            return 0;
        } /** else {

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "raycast_marker";
            marker.id = 4;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = ray.point_at_parameter(tMin).x;
            marker.pose.position.y = ray.point_at_parameter(tMin).y;
            marker.pose.position.z = ray.point_at_parameter(tMin).z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5; // Semi-transparent

            marker.lifetime = ros::Duration();
            pub_debug.publish(marker);

            visualization_msgs::Marker marker2;
            marker2.header.frame_id = "map";
            marker2.header.stamp = ros::Time::now();
            marker2.ns = "raycast_marker";
            marker2.id = 8;
            marker2.type = visualization_msgs::Marker::SPHERE;
            marker2.action = visualization_msgs::Marker::ADD;

            marker2.pose.position.x = ray.point_at_parameter(tMax).x;
            marker2.pose.position.y = ray.point_at_parameter(tMax).y;
            marker2.pose.position.z = ray.point_at_parameter(tMax).z;
            marker2.pose.orientation.w = 1.0;
            marker2.scale.x = 0.2;
            marker2.scale.y = 0.2;
            marker2.scale.z = 0.2;

            marker2.color.r = 1.0;
            marker2.color.g = 0.0;
            marker2.color.b = 0.0;
            marker2.color.a = 0.5; // Semi-transparent

            marker.lifetime = ros::Duration();
            pub_debug.publish(marker2);

        }  */      
        
        tMin = MAX(tMin, 0);
        tMax = MAX(tMax, max_range);
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
        while (current_Y_index != end_Y_index) {
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

            octomap::OcTreeNode* node = octree->search(match);

            if (node) {
                // Update the node at the specified point
                if (node->getOccupancy() == 0.5) {
                    return 1;
                } else if (node->getOccupancy() > 0.5) {
                    return 0;
                }

            } else {
                return 0;
            } 
        }
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

            ROS_INFO("Boundig box: %f, %f, %f, %f", min_x, max_x, min_y, max_y);

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
    message_filters::Subscriber<geometry_msgs::PoseArray> cuboid_corner_sub;
    message_filters::Subscriber<octomap_msgs::Octomap> octo_sub;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>>> sync;
    ros::Publisher pub_rcgrid;
    ros::Publisher pub_unknown_octree;
    ros::Publisher pub_bounding_box;
    ros::Subscriber sub_ogrid;
    ros::Publisher pub_debug;
    ros::Publisher pub_rectangle;
    ros::Publisher pub_best_pose;
    ros::Timer timer;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, nav_msgs::OccupancyGrid>>> rectangle_sync;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, octomap_msgs::Octomap>>> match_sync;
    Vec3 robot_pose;
    Vec3 robot_heading;
    

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_casting_node");
    RayCastingNode node;
    ros::spin();
    return 0;
}
