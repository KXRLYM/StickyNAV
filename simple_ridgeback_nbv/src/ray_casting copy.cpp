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


#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<time.h>
#include<iostream>
#include<stdlib.h>

static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

using namespace std;
using namespace Eigen;

struct Plane {
    Vec3 normal;
    float d;

    Plane() : normal(Vec3()), d(0) {}
    Plane(const Vec3& n, float d) : normal(n), d(d) {}
};

float pointToPlaneDistance(const Vec3& point, const Plane& plane) {
    return fabs(plane.normal.dot(point) + plane.d) / plane.normal.length();
}

void cuboidRansac(const geometry_msgs::PoseArray &pose_array) 
{

    srand(time(NULL));

    int i(0);
    int max_iteration(100);
    float threshold = 0.1f;
    vector<int> bestInliers;
    vector<Plane> bestPlanes;

    Vec3 points[6];

    while (i < max_iteration) {
        i++;
        for (int j = 0; j < 6; j++) {
            int random_number = rand() % pose_array.poses.size();
            points[j] = Vec3(pose_array.poses[random_number].position.x, pose_array.poses[random_number].position.y, pose_array.poses[random_number].position.z);
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
        vector<float> dist_pt(pose_array.poses.size(), FLT_MAX);
        for (const auto& plane : planes) {
            for (size_t idx = 0; idx < pose_array.poses.size(); ++idx) {
                Vec3 pt(
                    pose_array.poses[idx].position.x,
                    pose_array.poses[idx].position.y,
                    pose_array.poses[idx].position.z
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
}

// Helper function to compute the extremes of the coordinates
void computeExtremeBoundingBox(const geometry_msgs::PoseArray &pose_array, geometry_msgs::Point &min_point, geometry_msgs::Point &max_point)
{
    min_point.x = min_point.y = min_point.z = std::numeric_limits<double>::max();
    max_point.x = max_point.y = max_point.z = -std::numeric_limits<double>::max();

    for (const auto &pose : pose_array.poses)
    {
        const auto &point = pose.position;
        if (point.x < min_point.x) min_point.x = point.x;
        if (point.y < min_point.y) min_point.y = point.y;
        if (point.z < min_point.z) min_point.z = point.z;
        if (point.x > max_point.x) max_point.x = point.x;
        if (point.y > max_point.y) max_point.y = point.y;
        if (point.z > max_point.z) max_point.z = point.z;
    }
}

class RayCastingNode
{
public:
    RayCastingNode()
        : nh(),
        pub_rcgrid(nh.advertise<nav_msgs::OccupancyGrid>("/cast_map", 10)),
        sub_ogrid(nh.subscribe("/projected_map", 10, &RayCastingNode::ogrid_callback, this)),
        timer(nh.createTimer(ros::Duration(0.1), &RayCastingNode::main_loop, this)),
        sub_pose_array(nh.subscribe("/matches", 1000, &RayCastingNode::pose_array_callback, this)),
        pub_bounding_box(nh.advertise<visualization_msgs::Marker>("/cuboid_marker", 10)),
        listener()
    {
        odom_sub.subscribe(nh, "/odom", 10);
        map_sub.subscribe(nh, "/test_map", 10);
        rectangle_sub.subscribe(nh, "/rectangle", 10);
        // Define the sync policy and create the synchronizer
        using sync_pol = message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>;
        sync.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(500), odom_sub, map_sub));
        sync->registerCallback(boost::bind(&RayCastingNode::callback, this, _1, _2));

        using sync_pol1 = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, nav_msgs::OccupancyGrid>;
        rectangle_sync.reset(new message_filters::Synchronizer<sync_pol1>(sync_pol1(500), rectangle_sub, map_sub));
        rectangle_sync->registerCallback(boost::bind(&RayCastingNode::box_callback, this, _1, _2));
    }

    void box_callback(const geometry_msgs::PoseArray::ConstPtr& msg, const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        ROS_INFO("Beginning ray_cast");

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
        pub_rcgrid.publish(*pgrid);
    }

    void pose_array_callback(const geometry_msgs::PoseArray &pose_array) {
        ROS_INFO("received");

        srand(time(NULL));

        int i(0);
        int max_iteration(100);
        float threshold = 0.1f;
        vector<int> bestInliers;
        vector<Plane> bestPlanes;

        Vec3 points[6];

        while (i < max_iteration) {

            for (int j = 0; j < 6; j++) {
                int random_number = rand() % pose_array.poses.size();
                points[j] = Vec3(pose_array.poses[random_number].position.x, pose_array.poses[random_number].position.y, pose_array.poses[random_number].position.z);
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
            vector<float> dist_pt(pose_array.poses.size(), FLT_MAX);
            for (const auto& plane : planes) {
                for (size_t idx = 0; idx < pose_array.poses.size(); ++idx) {
                    Vec3 pt(
                        pose_array.poses[idx].position.x,
                        pose_array.poses[idx].position.y,
                        pose_array.poses[idx].position.z
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
    
        if (!bestPlanes.empty()) {
            // Compute the bounding box (cuboid) from the best planes
            Vec3 min_point(FLT_MAX, FLT_MAX, FLT_MAX);
            Vec3 max_point(-FLT_MAX, -FLT_MAX, -FLT_MAX);

            for (const auto& idx : bestInliers) {
                Vec3 pt(
                    pose_array.poses[idx].position.x,
                    pose_array.poses[idx].position.y,
                    pose_array.poses[idx].position.z
                );

                min_point.x = min(min_point.x, pt.x);
                min_point.y = min(min_point.y, pt.y);
                min_point.z = min(min_point.z, pt.z);

                max_point.x = max(max_point.x, pt.x);
                max_point.y = max(max_point.y, pt.y);
                max_point.z = max(max_point.z, pt.z);
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "extreme_cuboid";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // Compute the center and dimensions of the bounding box
            marker.pose.position.x = (min_point.x + max_point.x) / 2;
            marker.pose.position.y = (min_point.y + max_point.y) / 2;
            marker.pose.position.z = (min_point.z + max_point.z) / 2;

            marker.scale.x = max_point.x - min_point.x;
            marker.scale.y = max_point.y - min_point.y;
            marker.scale.z = max_point.z - min_point.z;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5; // Semi-transparent

            marker.lifetime = ros::Duration();

            pub_bounding_box.publish(marker);

        }

        /** 
        geometry_msgs::Point min_point, max_point;
        computeExtremeBoundingBox(pose_array, min_point, max_point);



        // Publish the cuboid as a marker in RViz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "extreme_cuboid";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = (min_point.x + max_point.x) / 2;
        marker.pose.position.y = (min_point.y + max_point.y) / 2;
        marker.pose.position.z = (min_point.z + max_point.z) / 2;

        marker.scale.x = max_point.x - min_point.x;
        marker.scale.y = max_point.y - min_point.y;
        marker.scale.z = max_point.z - min_point.z;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5; // Semi-transparent

        marker.lifetime = ros::Duration();

        pub_bounding_box.publish(marker);*/
        

    }

    void callback(const nav_msgs::Odometry::ConstPtr& odom_msg, const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        /** 
        ROS_INFO("Received");
        float robot_x = odom_msg->pose.pose.position.x;
        float robot_y = odom_msg->pose.pose.position.y;
        float map_origin_x = map_msg->info.origin.position.x;
        float map_origin_y = map_msg->info.origin.position.y;
        float res = map_msg->info.resolution;
        
        const geometry_msgs::Quaternion& quat = odom_msg->pose.pose.orientation;    
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll,pitch,yaw);
        Vec2 direction(cos(yaw), sin(yaw)); // unit vector
        ROS_INFO("headings %f, %f", direction.x, direction.y);

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
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::OccupancyGrid>>> sync;
    ros::Publisher pub_rcgrid;
    ros::Publisher pub_bounding_box;
    ros::Subscriber sub_ogrid;
    ros::Subscriber sub_pose_array;
    ros::Timer timer;
    tf::TransformListener listener;
    message_filters::Subscriber<geometry_msgs::PoseArray> rectangle_sub;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, nav_msgs::OccupancyGrid>>> rectangle_sync;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_casting_node");
    RayCastingNode node;
    ros::spin();
    return 0;
}
