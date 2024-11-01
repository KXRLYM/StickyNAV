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

#include<trajectory_msgs/JointTrajectory.h>

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

#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<time.h>
#include<iostream>
#include<stdlib.h>


#include <thread>
#include <atomic>

#include "rrt.h"
#include "kd_rrt.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/callback_queue.h>

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


#define MAX(a,b) ((a > b ? a : b))
#define MIN(a,b) ((a > b ? b : a))

using KDNodePtr = std::shared_ptr<KD_RRT::KDNode>;

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
    static ros::CallbackQueue odom_queue;

    RayCastingNode()
        : nh(),
        special_odom_sub(nh.subscribe("/odom", 10, &RayCastingNode::special_odom_callback, this)),
        unknown_tree_sub(nh.subscribe("/unknown_octree", 10, &RayCastingNode::special_tree_callback, this)),
        pub_rcgrid(nh.advertise<nav_msgs::OccupancyGrid>("/cast_map", 10)),
        sub_ogrid(nh.subscribe("/projected_map", 10, &RayCastingNode::ogrid_callback, this)),
        timer(nh.createTimer(ros::Duration(0.1), &RayCastingNode::main_loop, this)),
        pub_bounding_box(nh.advertise<visualization_msgs::Marker>("/cuboid_marker", 10)),
        listener(buffer),
        ee_listener(ee_buffer),
        base_listener(base_buffer),
        front_listener(front_buffer),
        map_listener(map_buffer),
        unknown_tree_arrived(false),
        robot_pose(0.0,0.0,0.0),
        robot_heading(0.0,0.0,0.0),
        backup_plan_cuboid(0.0, 0.0, 0.0),
        pub_debug(nh.advertise<geometry_msgs::PoseArray>("/debug_marker_ray", 100)),
        pub_debug2(nh.advertise<geometry_msgs::PoseArray>("/debug_marker_ray2", 100)),
        pub_best_pose(nh.advertise<visualization_msgs::Marker>("/best_pose", 10)),
        sub_octomap(nh.subscribe("/octomap_full", 10, &RayCastingNode::normal_callback, this)),
        //cuboid_corner_sub(nh.subscribe("/match_cuboid_corners", 10, &RayCastingNode::corner_callback, this)),
        grid_received(false),
        explorationed_started(false),
        exploration_finished(false),
        pub_move_base_goal(nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1)),
        pub_tree(nh.advertise<visualization_msgs::MarkerArray>("/rrt_tree", 10)),
        pub_rrt_sample_debug(nh.advertise<visualization_msgs::MarkerArray>("/rrt_debug_sample", 10)),
        pub_franka_control(nh.advertise<trajectory_msgs::JointTrajectory>("/position_joint_trajectory_controller/command",10)),
        retry(0),
        rrt_tree(),
        action_client("move_base", true),
        pub_cmd_vel(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
        kd_tree_(pub_tree),
        kd_tree_initialised(false),
        pub_search_cuboid(nh.advertise<geometry_msgs::PoseArray>("/match_cuboid_corners", 10)),
        robot_pos(0.0,0.0,0.0),
        current_robot_node(nullptr),
        root_node(nullptr)
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

    float computeNormalGain(float x, float y, float rec_x, float rec_y, octomap::OcTree* octree, float FOV, int FOV_resolution) {
        float dy = rec_y - y;
        float dx = rec_x - x;  
        float mag = std::sqrt(dy * dy + dx * dx);
        float dy_norm = (mag != 0.0) ? (dy / mag) : 0.0;
        float dx_norm = (mag != 0.0) ? (dx / mag) : 0.0;
        Vec3 vec_dir(dx_norm, dy_norm, 0.0);
        vec_dir.normalize();

        octomap::point3d origin(x, y, 0.0);
        octomap::point3d direction(vec_dir.x, vec_dir.y, 0.0);
        octomap::point3d end; 

        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "map";

        geometry_msgs::Pose origin_pose;
        origin_pose.position.x = origin.x();
        origin_pose.position.y = origin.y();
        
        geometry_msgs::Pose direction_pose;
        direction_pose.position.x = vec_dir.x;
        direction_pose.position.y = vec_dir.y;

        tf2::Transform refTomap_tf;
        refTomap_tf.setOrigin(tf2::Vector3(x, y, 0.0));
        tf2::Quaternion ref_quat;
        ref_quat.setRPY(0.0, 0.0, atan2(dy_norm, dx_norm));
        refTomap_tf.setRotation(ref_quat);

        geometry_msgs::TransformStamped refTomap = transformToTransformStamped(refTomap_tf, "map", "new_base_link", ros::Time::now());
        
        tf2::Transform mapToref_tf = refTomap_tf.inverse();
        geometry_msgs::TransformStamped mapToref = transformToTransformStamped(mapToref_tf, "new_base_link", "map", ros::Time::now());

        // base link frame!
        geometry_msgs::Pose base_link_origin_base_link;
        base_link_origin_base_link.position.x = 0.0;
        base_link_origin_base_link.position.y = 0.0;
        base_link_origin_base_link.orientation.w = 1.0;

        geometry_msgs::Pose test_pose_ref = transformPose(base_link_origin_base_link, refTomap);
        pose_array.poses.push_back(test_pose_ref);
        
        geometry_msgs::Pose ball_map;
        ball_map.position.x = rec_x;
        ball_map.position.y = rec_y;


        geometry_msgs::TransformStamped eyeTobase;

        try {
            eyeTobase = ee_buffer.lookupTransform("base_link",  "front_realsense",ros::Time(0));
            
        } catch (tf2::TransformException &ex) { 
                ROS_WARN("Could not transform point: %s", ex.what());
                return 0.0;
        }

        geometry_msgs::Pose base_link_origin_map = transformPose(base_link_origin_base_link, refTomap);
        geometry_msgs::Pose ball_base_link = transformPose(ball_map, mapToref);

        geometry_msgs::Pose vec_dir_base_link;
        vec_dir_base_link.position.x = 1;
        vec_dir_base_link.position.y = 0;
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, atan2(vec_dir_base_link.position.y, vec_dir_base_link.position.x));
        vec_dir_base_link.orientation.x = quat.x();
        vec_dir_base_link.orientation.y = quat.y();
        vec_dir_base_link.orientation.z = quat.z();
        vec_dir_base_link.orientation.w = quat.w();

        geometry_msgs::Pose vec_dir_map = transformPose(vec_dir_base_link, refTomap);

        
        float angle_of_rot_base_link = atan2(ball_base_link.position.y - base_link_origin_base_link.position.y,  ball_base_link.position.x - base_link_origin_base_link.position.x);
        
        geometry_msgs::Pose transformed_vec_dir_base_link;
        geometry_msgs::Pose transformed_vec_dir_map;
        transformed_vec_dir_base_link = transformDir(vec_dir_base_link, angle_of_rot_base_link);
        tf2::Quaternion cam_quat;
        cam_quat.setRPY(0, 0, angle_of_rot_base_link);
        transformed_vec_dir_base_link.orientation.x = cam_quat.x();
        transformed_vec_dir_base_link.orientation.y = cam_quat.y();
        transformed_vec_dir_base_link.orientation.z = cam_quat.z();
        transformed_vec_dir_base_link.orientation.w = cam_quat.w();
        transformed_vec_dir_map = transformPose(transformed_vec_dir_base_link, refTomap);

        //pose_array.poses.push_back(transformed_vec_dir_map);

        geometry_msgs::Pose transformed_cam_ray;
        transformed_cam_ray = transformPose(base_link_origin_base_link, eyeTobase);
        transformed_cam_ray = transformDir(transformed_cam_ray, angle_of_rot_base_link);
        transformed_cam_ray.orientation.x = cam_quat.x();
        transformed_cam_ray.orientation.y = cam_quat.y();
        transformed_cam_ray.orientation.z = cam_quat.z();
        transformed_cam_ray.orientation.w = cam_quat.w();

        geometry_msgs::Pose transformed_cam_ray_dir;
        transformed_cam_ray_dir = transformPose(vec_dir_base_link, eyeTobase);
        transformed_cam_ray_dir = transformDir(transformed_cam_ray_dir, angle_of_rot_base_link);
        //transformed_cam_ray_dir.position.x += transformed_cam_ray.position.x;
        //transformed_cam_ray_dir.position.y += transformed_cam_ray.position.y;
        transformed_cam_ray_dir.orientation.x = cam_quat.x();
        transformed_cam_ray_dir.orientation.y = cam_quat.y();
        transformed_cam_ray_dir.orientation.z = cam_quat.z();
        transformed_cam_ray_dir.orientation.w = cam_quat.w();

        geometry_msgs::Pose transformed_cam_ray_dir1;
        geometry_msgs::Pose transformed_cam_ray1;
        transformed_cam_ray1 = transformPose(transformed_cam_ray, refTomap);
        transformed_cam_ray_dir1 = transformPose(transformed_cam_ray_dir, refTomap);

        
        float FOV_step = FOV / FOV_resolution;
        float gain(0.0);

        // =========================================================== Looping through FOV to add information gain
        for (int fov_idx = 0; fov_idx < FOV_resolution * FOV_resolution; fov_idx++) {
            
            int i = fov_idx % FOV_resolution; 
            int j = fov_idx / FOV_resolution;  

            float h_angle = -FOV / 2 + i * FOV_step;
            float v_angle = -FOV / 2 + j * FOV_step;

            Vec3 new_vec = Vec3(vec_dir_base_link.position.x, vec_dir_base_link.position.y, vec_dir_base_link.position.z).normalize()                       
                                .rotateZ(h_angle)
                                .rotateY(v_angle);
            
            // from baselink to an end effector
            geometry_msgs::Pose ray_base_link;
            ray_base_link.position = base_link_origin_base_link.position;
            tf2::Quaternion new_quat;
            new_quat.setRPY(0, v_angle, h_angle);
            set_quat(new_quat, ray_base_link);

            geometry_msgs::Pose ray_base_link_map;
            ray_base_link_map = transformPose(ray_base_link, refTomap);
            //pose_array.poses.push_back(ray_base_link_map);

            geometry_msgs::Pose ray_dir_base_link;
            ray_dir_base_link.position.x = new_vec.x;
            ray_dir_base_link.position.y = new_vec.y;
            ray_dir_base_link.position.z = new_vec.z;
            set_quat(new_quat, ray_dir_base_link);

            
            geometry_msgs::Pose ray_dir_base_link_map;
            ray_dir_base_link_map = transformPose(ray_dir_base_link, refTomap);
            //pose_array.poses.push_back(ray_dir_base_link_map);
            
            // baselink to ee
            geometry_msgs::Pose ray_eye_base_link;
            ray_eye_base_link = transformPose(ray_base_link, eyeTobase);

            // baselink to ee
            geometry_msgs::Pose ray_eye_dir_base_link;
            ray_eye_dir_base_link = transformPose(ray_dir_base_link, eyeTobase);
            
            // rotate to the target
            geometry_msgs::Pose rotated_ray_eye_base_link;
            rotated_ray_eye_base_link = transformDir(ray_eye_base_link, angle_of_rot_base_link);
            tf2::Quaternion rot_quat;
            rot_quat.setRPY(0,v_angle, h_angle + angle_of_rot_base_link);
            set_quat(rot_quat, rotated_ray_eye_base_link);
            
            geometry_msgs::Pose rotated_ray_eye_dir_base_link;
            rotated_ray_eye_dir_base_link = transformDir(ray_eye_dir_base_link, angle_of_rot_base_link);
            set_quat(rot_quat, rotated_ray_eye_dir_base_link);

            // transform to the map
            geometry_msgs::Pose ray_map;
            ray_map = transformPose(rotated_ray_eye_base_link, refTomap);

            geometry_msgs::Pose ray_dir_map;
            ray_dir_map = transformPose(rotated_ray_eye_dir_base_link, refTomap);

            // then publish!
            pose_array.poses.push_back(ray_map);
            pose_array.poses.push_back(ray_dir_map);

            octomap::point3d origin_EE(ray_map.position.x, ray_map.position.y, ray_map.position.z);
            octomap::point3d direction_EE(ray_dir_map.position.x - ray_map.position.x, ray_dir_map.position.y -  ray_map.position.y , ray_dir_map.position.z - ray_map.position.z);
        

            if (octree->castRay(origin_EE, direction_EE, end, true, 5)) {
                geometry_msgs::Pose pose;
                pose.position.x = end.x();
                pose.position.y = end.y();
                pose.position.z = end.z();
                pose.orientation.w = 1.0;        
                pose_array.poses.push_back(pose);

                octomap::OcTreeNode* node = octree->search(end);
                
                if (node->getOccupancy() >= 0.5 && node->getOccupancy() < 0.51) {
                    //ROS_INFO("usual occ %f", node->getOccupancy());
                    //unexplored
                    gain += 1;
                }

            }
        } // FOV loop ends */
        pub_debug.publish(pose_array);
        return gain;
    }

    void special_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        //ROS_INFO("special odom!");
        
        /** */
        geometry_msgs::TransformStamped odom_map_transform = buffer.lookupTransform("map","odom", ros::Time(0));
        geometry_msgs::PoseStamped ros_pos_odom;
        ros_pos_odom.header = odom_msg->header;
        ros_pos_odom.pose = odom_msg->pose.pose;
        geometry_msgs::PoseStamped ros_pos_map;
        tf2::doTransform(ros_pos_odom, ros_pos_map, odom_map_transform);
    
        float robot_x = ros_pos_map.pose.position.x;
        float robot_y = ros_pos_map.pose.position.y;
        
        robot_pos.x = robot_x;
        robot_pos.y = robot_y;
        robot_pos.z = 0.0;

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


    void turn_around() {
        // move the camera down
        trajectory_msgs::JointTrajectory jt;
        jt.joint_names.push_back("fr3_joint6");
        jt.points.resize(1);
        jt.points[0].positions.resize(1);
        jt.points[0].positions[0] = 2.4; // looking down;
        jt.points[0].time_from_start = ros::Duration(1.0);
        //pub_franka_control.publish(jt);

        // make the base turn
        geometry_msgs::Twist command_vel;
        float angular_velocity = 0.5;
        ros::Rate rate(10);
        float duration(6.28318530 /  angular_velocity);
        ros::Time start_time = ros::Time::now();
        while (ros::Time::now() - start_time < ros::Duration(duration)) {
            ROS_INFO("ROBOT TURNING FOR AN INITIAL SCAN");
            command_vel.angular.z = angular_velocity;
            pub_cmd_vel.publish(command_vel);
            rate.sleep();
        }
        
        command_vel.angular.z = 0.0;
        pub_cmd_vel.publish(command_vel);

        // move the camera up
        jt.points[0].positions[0] = 3.14;
        //pub_franka_control.publish(jt);

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
            //ROS_INFO("current %f", (ros::Time::now()).toSec());
            pub_cmd_vel.publish(command_vel);
            rate.sleep();
            ROS_INFO("robot %f", robot_pos.x);
        }

        command_vel.linear.x = 0;
        command_vel.linear.y = 0;
        pub_cmd_vel.publish(command_vel);

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

        if (!unknown_tree_arrived) {
            ROS_INFO("waiting for mapping to update unknown tree");
            return;
        }

        // ========================================= Get environment parameters + mapping
        // Initialise two trees, one for octomap and one for processed octomap for unknown voxels
        octomap::AbstractOcTree* octo_abs_tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octo_abs_tree);

        octomap::AbstractOcTree* octo_abs_tree_unknown = octomap_msgs::msgToMap(octomap_msg);
        octomap::OcTree* unknown_octree = dynamic_cast<octomap::OcTree*>(octo_abs_tree_unknown);

        // get the map bounds
        float map_max_x = grid_2d.info.origin.position.x + grid_2d.info.width * grid_2d.info.resolution;
        float map_max_y = grid_2d.info.origin.position.y + grid_2d.info.height * grid_2d.info.resolution;
        float map_min_x = grid_2d.info.origin.position.x;
        float map_min_y = grid_2d.info.origin.position.y;

        Vec3 minBound(map_min_x, map_min_y, 0.0);
        Vec3 maxBound(map_max_x, map_min_y, 1.5);

        // ========================================= Check to initialise the search
        
        geometry_msgs::Point start_rrt_pt;
        start_rrt_pt.x = robot_pos.x;
        start_rrt_pt.y = robot_pos.y;
        start_rrt_pt.z = 0.0;

        if (!kd_tree_initialised) {
            // set the root node for the first time
            kd_tree_initialised = 1;

            KD_RRT::KDNode::KDNodePtr rootNode = std::make_shared<KD_RRT::KDNode>(start_rrt_pt);
            kd_tree_.insert_node(rootNode);
            ROS_INFO("Root node %f, %f", rootNode->position.x, rootNode->position.y);
            root_node = rootNode;
            turn_around();
            return;
            
        }

        // ========================================= debugging pose array
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "map";


        // =========================================================== Decretisiation of FOV
        float FOV = 90 * 0.0174533f; // degree * 1 rad/degree
        int FOV_resolution = 40;
        float FOV_step = FOV / FOV_resolution;

        // ========================================= RRT* parameters
        int num_sample(0);
        int max_sample(10);
        int max_step_size(1);
        int neighbourhood_radius(3);
        float update_robot_radius(4);
        float map_search_radius(10);

        int n_local_max(5);
        int n_local(0);
    
        while (max_sample > num_sample || n_local < n_local_max) {

            // ======================================================== Filtering samples
            //auto V_rand = kd_tree_.randomSample(map_min_x, map_max_x, map_min_y, map_max_y);
            auto V_rand = kd_tree_.randomSample(robot_pos.x - map_search_radius, robot_pos.x + map_search_radius,
                                            robot_pos.y - map_search_radius, robot_pos.y + map_search_radius);

            if (V_rand.x < map_min_x || V_rand.x > map_max_x || V_rand.y < map_min_y || V_rand.y > map_max_y) {
                continue;
            }

            // ensure the point is in free space
            int grid_index = grid_2d.info.width*std::floor((V_rand.y - grid_2d.info.origin.position.y)/grid_2d.info.resolution) + std::floor((V_rand.x - grid_2d.info.origin.position.x)/grid_2d.info.resolution);
            if (grid_index < grid_2d.info.width * grid_2d.info.height) {
                if (grid_2d.data[grid_index] == -1 || grid_2d.data[grid_index] > 90) {
                    continue;
                }
            }
            ROS_INFO("Sampled at %f, %f", V_rand.x, V_rand.y);

            // ======================================================== collision checking potential new node
            auto nearest_node = kd_tree_.nn_search(V_rand);

            if (nearest_node == nullptr) {
                continue;
            } else {
                // extend the tree based on the cost?
                //ROS_INFO("found the nearest_node %f, %f", nearest_node->position.x, nearest_node->position.y);
                
                geometry_msgs::Point newNodePosition;
                float dist = std::sqrt(std::pow(V_rand.x - nearest_node->position.x, 2) + std::pow(V_rand.y - nearest_node->position.y, 2));
                float dx = (V_rand.x - nearest_node->position.x) / dist;
                float dy = (V_rand.y - nearest_node->position.y) / dist;
                newNodePosition.x = nearest_node->position.x + max_step_size * dx;
                newNodePosition.y = nearest_node->position.y + max_step_size * dy;
                newNodePosition.z = nearest_node->position.z; // usually in 2D

                KD_RRT::KDNode::KDNodePtr newNode = std::make_shared<KD_RRT::KDNode>(newNodePosition);

                if (checkLinearCollision(octree, nearest_node->position, newNodePosition, 4)) {
                    continue;
                } else {
                    if (checkConservativeCollision(octree, newNode->position.x, newNode->position.y)) {
                        // sometimes steering command requires additional collision checker
                        continue;
                    }

                    float local_dist = std::sqrt(std::pow(newNodePosition.x - robot_pos.x, 2) + std::pow(newNodePosition.y - robot_pos.y, 2));
                    if (local_dist < neighbourhood_radius) {
                        n_local++;
                    }
                    // too many local points
                    /** 
                    geometry_msgs::Point robot_position;
                    robot_position.x = robot_pos.x;
                    robot_position.y = robot_pos.y;
                    auto check_close_neighbours = kd_tree_.radius_search(robot_position, neighbourhood_radius);
                    if (check_close_neighbours.size() > n_local_max) {
                        break;
                    }*/
                    

                    // set the cost
                    double temp_cost_ = nearest_node->cost + max_step_size;

                    // identify all the affected neighbours
                    auto neighbours = kd_tree_.radius_search(newNodePosition, neighbourhood_radius);

                    // for all neighbours check the cost then set the parent based on the lowest cost.
                    KD_RRT::KDNode::KDNodePtr best_parent(nearest_node);

                     
                    for (const auto& neighbour : neighbours) {
                        double temp_dist_ = std::sqrt(std::pow(newNodePosition.x - neighbour->position.x, 2) + std::pow(newNodePosition.y - neighbour->position.y, 2));
                        if (neighbour->cost + temp_dist_ < temp_cost_ && !checkLinearCollision(octree, neighbour->position, newNodePosition, 4)) {
                            temp_cost_ = neighbour->cost + temp_dist_;
                            best_parent = neighbour;
                        }
                    }
                    auto V_facing = kd_tree_.randomSample(map_min_x, map_max_x, map_min_y, map_max_y);

                    geometry_msgs::Point heading;
                    heading.x = V_facing.x;
                    heading.y = V_facing.y;

                    newNode->heading = heading;
                    newNode->cost = temp_cost_;
                    newNode->parent = best_parent;
                       
                    kd_tree_.insert_node(newNode);
                    float gain = computeNormalGain(newNode->position.x, newNode->position.y, V_facing.x, V_facing.y, unknown_octree, FOV, FOV_resolution);
                    newNode->gain = gain + newNode->parent->gain; 

                    num_sample++;

                    // Require rewiring the neighbouring nodes if the cost is cheaper to go past this newly added node!
                    for (const auto& neighbour : neighbours) {
                        double temp_dist_ = std::sqrt(std::pow(newNodePosition.x - neighbour->position.x, 2) + std::pow(newNodePosition.y - neighbour->position.y, 2));
                        if (newNode->cost + temp_dist_ < neighbour->cost && !checkLinearCollision(octree, neighbour->position, newNodePosition, 4) && !(neighbour->has_immutable_parent)) {
                            //ROS_INFO("rewiring");
                            float temp_gain_ = neighbour->gain - neighbour->parent->gain;
                            neighbour->gain = newNode->gain + temp_gain_;
                            neighbour->parent = newNode;
                            neighbour->cost = newNode->cost + temp_dist_;
                        }
                    }

                    
                } // linear collision check ends
            } // nearest point isn't a null ptr
        } // sampling + building tree ends


        // =================================================================== Calculating the best gain node
        float value(0.0);

        KD_RRT::KDNode::KDNodePtr best_node = kd_tree_.find_highest_gain_node();
        
        if (best_node == nullptr) {
            ROS_INFO("gain be all zeros?");
            return;
        }

        kd_tree_.print_tree();
        kd_tree_.visualise_tree(best_node->gain);
        publish_best_node(best_node->position);

        // move to the best node
        std::deque<KD_RRT::KDNode::KDNodePtr> path;

        KD_RRT::KDNode::KDNodePtr node_it = best_node;
        
        // =================================================================== Generating a path to the best node
        if (current_robot_node == nullptr) {
            // first time for the path creation all the way to the origin
            while (node_it->parent != nullptr) {
                path.push_front(node_it);
                node_it = node_it->parent;
            }
            //path.push_back(best_node);

        } else {
            // looking for a common parent in a loop
            KD_RRT::KDNode::KDNodePtr common_parent(nullptr);
            bool parent_found(false);

            while (current_robot_node->parent != nullptr && !parent_found) { 
                // start iterating through the current path we are on
                path.push_back(current_robot_node);
                KD_RRT::KDNode::KDNodePtr temp_node_it = node_it;

                while (temp_node_it->parent != nullptr) {
                    // start iterating through the current best path from the origin
                    if (current_robot_node == temp_node_it) {
                        common_parent = current_robot_node;
                        parent_found = true;
                        break;
                    }
                    if (current_robot_node == temp_node_it->parent) {
                        common_parent = current_robot_node;
                        parent_found = true;
                        break;
                    }
                    if (current_robot_node->parent == temp_node_it->parent) {
                        common_parent = temp_node_it->parent;
                        parent_found = true;
                        break;
                    }
                    temp_node_it = temp_node_it->parent;
                }

                if (parent_found) {
                    std::deque<KD_RRT::KDNode::KDNodePtr>::iterator it = path.end();
                    while (node_it != common_parent) {
                        path.insert(it,node_it);
                        node_it = node_it->parent;  
                    }
                    path.insert(it, common_parent);
                } else {
                    // move one step back in the robot path
                    current_robot_node = current_robot_node->parent;
                }
                
            }

            if (current_robot_node->parent == nullptr && !parent_found) {
                // manually add origin if no common parent is found
                path.push_back(current_robot_node);
            }
        }

        ROS_INFO("size of the path %li", path.size());

        float velocity = 0.1;
        float angular_velocity = 0.1;

        geometry_msgs::Point current_rrt = start_rrt_pt;
        for (auto path_node : path) {
            path_node->has_immutable_parent = true;
            path_node->active_path = true;

            ROS_INFO("Onto the next node!");

            move_base_msgs::MoveBaseGoal goal;

            double yaw = atan2(path_node->heading.y, path_node->heading.x); // Calculate yaw based on direction
            if (std::isnan(yaw)) {
                yaw = 0.0; // Set a default value if yaw is NaN
            }

            // Create quaternion from roll, pitch, and yaw
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            q.normalize(); // Normalize the quaternion

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position = path_node->position;
            goal.target_pose.pose.orientation.x = q.x();
            goal.target_pose.pose.orientation.y = q.y();
            goal.target_pose.pose.orientation.z = q.z();
            goal.target_pose.pose.orientation.w = q.w();

            action_client.sendGoal(goal);
            bool success = action_client.waitForResult(ros::Duration(60.0)); // Waits for the result with a timeout of 30 seconds
            path_node->gain = 0.0;
        }
        // robot is now at the best node
        current_robot_node = best_node;
        //root_node->cost = best_node->cost;
        best_node->cost = 0.0;
        //best_node->gain = 0.0;
        //best_node->parent = nullptr;
        /** 
        kd_tree_.update_gain_and_cost();
        for (auto path_node:path) {
            path_node->active_path = false;
        }*/

        /**  rewiring the previous rootnode
        KD_RRT::KDNode::KDNodePtr node_root_it = best_node;
        while ((node_root_it->parent)->parent != nullptr) {
            node_root_it = node_root_it->parent;
        }
        root_node->parent = node_root_it;    
        best_node->parent = nullptr;
        root_node = best_node;

        kd_tree_.print_tree();*/

        // update the gain of the node near the robot at the best node
        auto neighbours = kd_tree_.radius_search(best_node->position, update_robot_radius);
        for (const auto neighbour : neighbours) {
            ROS_INFO("updating neighbours at %f, %f", neighbour->position.x, neighbour->position.y);

            auto V_facing = kd_tree_.randomSample(map_min_x, map_max_x, map_min_y, map_max_y);
            geometry_msgs::Point heading;
            heading.x = V_facing.x;
            heading.y = V_facing.y;
            float gain = computeNormalGain(neighbour->position.x, neighbour->position.y, heading.x, heading.y, unknown_octree, FOV, FOV_resolution);
            neighbour->heading = heading;

            /** 
            float temp_cost_ = std::sqrt(stf::pow(best_node->position.x - neighbour->position.x) + std::pow(best_node->position.y - neighbour->position.y));
            if (neighbour->cost > temp_cost_) {
                neighbour->parent = best_node;
                neighbour->cost = temp_cost_;
            }*/

            if (neighbour->parent != nullptr) {
                neighbour->gain = gain + neighbour->parent->gain;
            } else {
                neighbour->gain = gain;
            }
            ROS_INFO("new gain %f", gain);
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
            set_quat(q, start_pose);
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

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*map_msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        double resolution = octree->getResolution();

        Vec3 minBound(rec_min_x, rec_min_y, 0.0);
        Vec3 maxBound(rec_max_x, rec_max_y, 10.0);

        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "map";
         
        if (octree) // can be NULL
        {
            
            // =========================================================== Decretisiation of FOV
            float FOV = 60 * 0.0174533f; // degree * 1 rad/degree
            int FOV_resolution = 40;
            float FOV_step = FOV / FOV_resolution;

            float camera_height = 0.29; // [m]
            float camera_start_buffer = 1.0; // [m]

            // =========================================================== Decretisiation of Search Area
            double VP_step_size = 0.8; // [m / cell]
            int VP_num_steps_x = std::ceil((rec_max_x - rec_min_x) / VP_step_size);
            int VP_num_steps_y = std::ceil((rec_max_y - rec_min_y) / VP_step_size);

            double best_info_gain(0.0);
            double best_x(0.0);
            double best_y(0.0);
            double best_dx(0.0);
            double best_dy(0.0);
        

            // =========================================================== Looping through all grid cell to add information gain
            for (int search_idx = 0; search_idx < VP_num_steps_x * VP_num_steps_y; search_idx++) {
                // need to set a new origin and direction towards the center of the bounding box
                
                float x = rec_min_x + (search_idx % VP_num_steps_x) * VP_step_size;
                float y = rec_min_y + (search_idx / VP_num_steps_x) * VP_step_size;

                float dy = rec_centre_y - y;
                float dx = rec_centre_x - x;
                float mag = std::sqrt(dy * dy + dx * dx);
                float dy_norm = (mag != 0.0) ? (dy / mag) : 0.0;
                float dx_norm = (mag != 0.0) ? (dx / mag) : 0.0;
                Vec3 vec_dir(dx_norm, dy_norm, 0.0);

                octomap::point3d origin(x, y, camera_height);
                octomap::point3d direction(dx_norm, dy_norm, 0.0);
                octomap::point3d end;

                double current_gain(0.0);
                double current_total_gain(0.0);
                double dist = std::sqrt((robot_pose.x - x)*(robot_pose.x - x) + (robot_pose.y - y)*(robot_pose.y - y));
                /** Use this to debug search grid
                
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
                pose_array.poses.push_back(pose); **/
            
            
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
                                current_gain = current_gain + 10;
                            }

                            if (node->getOccupancy() >= 0.58 && node->getOccupancy() < 0.59) {
                                //potential object
                                current_gain += 0.01;
                            }


                        }
                    } 

                } // FOV loop ends

                 
                //float alpha = 0.2;
                //current_total_gain = static_cast<double>(current_gain) / (dist*alpha);
                if (current_gain > best_info_gain && !checkConservativeCollision(octree, x, y)) {
                    best_info_gain = current_gain;
                    best_x = x;
                    best_y = y;
                    best_dx = dx_norm;
                    best_dy = dy_norm;
                    ROS_INFO("current best info gain %f", best_info_gain);
                } else if (current_gain == best_info_gain && !checkConservativeCollision(octree, x, y)) {
                    best_info_gain = current_gain;
                }

            } // search loop ends
            
            // =========================================================== Terminating condition
            if (best_info_gain < 50) {
                retry++;
                if (retry < 1) {

                    // move to observe various sides of the rectangle.
                    float r = std::sqrt((robot_pose.x - rec_centre_x) * (robot_pos.x - rec_centre_x) + (robot_pos.y - rec_centre_y) * (robot_pos.y - rec_centre_y));
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

            geometry_msgs::Point start;
            start.x = robot_pose.x;
            start.y = robot_pose.y;
            start.z = 0.0;
            start_rrt(start, pose.position);
            
            action_client.cancelAllGoals();
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = pose;
            action_client.sendGoal(goal);
            bool success = action_client.waitForResult(ros::Duration(30.0));  // Waits for the result with a timeout of 30 seconds

        }
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

    void publish_best_node(const geometry_msgs::Point position) {
        visualization_msgs::Marker best;
        best.header.frame_id = "map";
        best.header.stamp = ros::Time::now();
        best.ns = "best";
        best.id = 78;
        best.type = visualization_msgs::Marker::SPHERE;
        best.action = visualization_msgs::Marker::ADD;

        best.pose.position.x = position.x;
        best.pose.position.y = position.y;
        best.pose.position.z = 0.0;

        best.scale.x = 0.5;
        best.scale.y = 0.5;
        best.scale.z = 0.5;

        best.color.r = 1.0;
        best.color.g = 0.0;
        best.color.b = 0.0;
        best.color.a = 0.5; // Semi-transparent

        best.lifetime = ros::Duration();
        // std::cout << best_info_gain << std::endl;
        pub_best_pose.publish(best);
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
        // grid 2d must be set if start_rrt returns true
        float map_max_x = grid_2d.info.origin.position.x + grid_2d.info.width * grid_2d.info.resolution;
        float map_max_y = grid_2d.info.origin.position.y + grid_2d.info.height * grid_2d.info.resolution;
        float map_min_x = grid_2d.info.origin.position.x;
        float map_min_y = grid_2d.info.origin.position.y;

        geometry_msgs::PoseArray match_corners;
        match_corners.header.stamp = ros::Time::now();
        match_corners.header.frame_id = "map";

        geometry_msgs::Pose minmin;
        minmin.position.x = map_min_x;
        minmin.position.y = map_min_y;
        match_corners.poses.push_back(minmin);
        geometry_msgs::Pose minmax;
        minmax.position.x = map_min_x;
        minmax.position.y = map_max_y;
        match_corners.poses.push_back(minmax);
        geometry_msgs::Pose maxmax;
        maxmax.position.x = map_max_x;
        maxmax.position.y = map_max_y;
        match_corners.poses.push_back(maxmax);
        geometry_msgs::Pose maxmin;
        maxmin.position.x = map_max_x;
        maxmin.position.y = map_min_y;
        match_corners.poses.push_back(maxmin);
        geometry_msgs::Pose h;
        h.position.z = 1;
        match_corners.poses.push_back(h);

        if (!explorationed_started && !exploration_finished) {
            pub_search_cuboid.publish(match_corners);
        }

    }

    bool bound_check(int32_t location, int32_t width, int32_t height) {
        return location >= 0 && location < width * height;
    }

    void main_loop(const ros::TimerEvent&) const
    {
    }

    void special_tree_callback(const octomap_msgs::Octomap& msg) {
        octomap_msg = msg;
        unknown_tree_arrived = true;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber special_odom_sub;
    ros::Subscriber unknown_tree_sub;
    octomap_msgs::Octomap octomap_msg;
    bool unknown_tree_arrived;
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
    ros::Publisher pub_debug2;
    ros::Publisher pub_rectangle;
    ros::Publisher pub_best_pose;
    ros::Timer timer;
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
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, nav_msgs::OccupancyGrid>>> rectangle_sync;
    //boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, octomap_msgs::Octomap>>> match_sync;
    Vec3 robot_pose;
    Vec3 robot_heading;
    Vec3 backup_plan_cuboid;
    Vec3 robot_pos;
    //geometry_msgs::PoseArray corner_msg;
    //ros::Subscriber cuboid_corner_sub;
    ros::Publisher pub_tree;
    nav_msgs::OccupancyGrid grid_2d;
    bool grid_received;
    bool explorationed_started;
    bool exploration_finished;
    bool kd_tree_initialised;
    ros::Publisher pub_move_base_goal;
    int retry;
    geometry_msgs::Pose start_pose;
    MoveBaseClient action_client;
    RRT::Tree rrt_tree;
    ros::Publisher pub_rrt_sample_debug;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_search_cuboid;
    ros::Publisher pub_franka_control;
    KD_RRT::KDTree kd_tree_;
    KD_RRT::KDNode::KDNodePtr root_node;
    KD_RRT::KDNode::KDNodePtr current_robot_node;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_casting_node");
    RayCastingNode node;
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
