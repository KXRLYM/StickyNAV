#include "ray_casting.h"

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

#include "kd_rrt.h"
#include "octomapper.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/callback_queue.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

std::mutex tree_mutex_;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace NBV {
    RayCaster::RayCaster() 
        : nh(),
        /* Subscriber -------------------------------------------------------------------------------*/
        odom_sub(nh.subscribe("/odom", 10, &RayCaster::odom_callback, this)),
        unknown_tree_sub(nh.subscribe("/unknown_octree", 10, &RayCaster::unknown_tree_callback, this)),
        occ_grid_sub(nh.subscribe("/move_base/global_costmap/costmap", 10, &RayCaster::occ_grid_callback, this)),
        octomap_sub(nh.subscribe("/octomap_full", 10, &RayCaster::normal_callback, this)),
        /* Publisher -------------------------------------------------------------------------------*/
        pub_debug(nh.advertise<geometry_msgs::PoseArray>("/debug_marker_ray", 100)),
        pub_sensor_poses(nh.advertise<geometry_msgs::PoseArray>("/sensor_poses", 100)),
        pub_best_pose(nh.advertise<visualization_msgs::Marker>("/best_pose", 10)),
        pub_move_base_goal(nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1)),
        pub_tree(nh.advertise<visualization_msgs::MarkerArray>("/rrt_tree", 10)),
        pub_franka_control(nh.advertise<trajectory_msgs::JointTrajectory>("/position_joint_trajectory_controller/command",10)),
        pub_cmd_vel(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
        pub_search_cuboid(nh.advertise<geometry_msgs::PoseArray>("/bounding_corners", 10)),
        pub_recon_finish(nh.advertise<geometry_msgs::PoseStamped>("/reconstruction_finished", 1)),
        /* Transform Listener -------------------------------------------------------------------*/
        listener(buffer),
        ee_listener(ee_buffer),
        base_listener(base_buffer),
        front_listener(front_buffer),
        map_listener(map_buffer),
        /* Flgs -------------------------------------------------------------------------------*/
        unknown_tree_arrived(false),
        search_started(false),
        spin_done(false), 
        grid_received(false),
        reconstruction_started(false),
        reconstruction_finished(false),
        kd_tree_initialised(false),
        start_3d_search(false),
        current_search_done(true),
        /* Kd tree variables ------------------------------------------------------------------*/
        current_robot_node(nullptr),
        root_node(nullptr),
        robot_pos(0.0,0.0,0.0),
        robot_heading(0.0,0.0,0.0),
        /* Miscel -------------------------------------------------------------------------------*/
        retry(0),
        action_client("move_base", true)
        //move_group_interface("arm") // name of the planning group
    {

        /** Setting up the normal callback to be called during exploration */
        mf_search_area_sub.subscribe(nh, "/search_area", 10);
        mf_octo_sub.subscribe(nh, "/unknown_octree", 10);

        using sync_pol_recon = message_filters::sync_policies::ApproximateTime<visualization_msgs::MarkerArray, octomap_msgs::Octomap>;
        sync_recon.reset(new message_filters::Synchronizer<sync_pol_recon>(sync_pol_recon(2000), mf_search_area_sub, mf_octo_sub));
        sync_recon->registerCallback(boost::bind(&RayCaster::cuboid_callback, this, _1, _2));

        // Waits for the action server to come up
        ROS_INFO("Waiting for move_base action server to start.");
        action_client.waitForServer(); 

        kd_tree_ = std::make_shared<KD_RRT::KDTree>(pub_tree);

    }
    

    float RayCaster::compute_gain(float x, float y, float rec_x, float rec_y, octomap::ColorOcTree* octree, float FOV, int FOV_resolution, int targeted, int publish /* =0 */) {
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

        geometry_msgs::PoseArray sensor_pose_array;
        sensor_pose_array.header.stamp = ros::Time::now();
        sensor_pose_array.header.frame_id = "map";

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
        //pose_array.poses.push_back(test_pose_ref);
        
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
        
        geometry_msgs::Pose transformed_vec_dir_base_link;
        geometry_msgs::Pose transformed_vec_dir_map;
        transformed_vec_dir_map = transformPose(transformed_vec_dir_base_link, refTomap);
        geometry_msgs::Pose transformed_cam_ray;
        transformed_cam_ray = transformPose(base_link_origin_base_link, eyeTobase);

        geometry_msgs::Pose transformed_cam_ray_dir;
        transformed_cam_ray_dir = transformPose(vec_dir_base_link, eyeTobase);

        geometry_msgs::Pose transformed_cam_ray_dir1;
        geometry_msgs::Pose transformed_cam_ray1;
        transformed_cam_ray1 = transformPose(transformed_cam_ray, refTomap);
        transformed_cam_ray_dir1 = transformPose(transformed_cam_ray_dir, refTomap);
        pose_array.poses.push_back(transformed_cam_ray1);
        //pose_array.poses.push_back(transformed_cam_ray_dir1);
        sensor_pose_array.poses.push_back(transformed_cam_ray1);
        
        float FOV_step = FOV / FOV_resolution;
        float gain(0.0);

        // =========================================================== Looping through FOV to add information gain
        for (int fov_idx = 0; fov_idx < FOV_resolution * FOV_resolution; fov_idx++) {
            
            int i = fov_idx % FOV_resolution; 
            int j = fov_idx / FOV_resolution;  

            float h_angle = -FOV / 2 + i * FOV_step;
            float v_angle = -FOV / 2 + j * FOV_step;

            float h_angle_rad = h_angle * (M_PI / 180.0f);
            float v_angle_rad = v_angle * (M_PI / 180.0f);

            Vec3 new_vec = Vec3(vec_dir_base_link.position.x, vec_dir_base_link.position.y, vec_dir_base_link.position.z).normalize()                       
                                .rotateZ(h_angle_rad)
                                .rotateY(v_angle_rad);
            
            geometry_msgs::Pose debug_link;
            debug_link.position.x = new_vec.x;
            debug_link.position.y = new_vec.y;
            debug_link.position.z = new_vec.z;
            debug_link.orientation.w = 1;
            //pose_array.poses.push_back(debug_link);

            // rotate to the target
            geometry_msgs::Pose rotated_debug_link;
            rotated_debug_link = transformRot(debug_link, eyeTobase);
            rotated_debug_link = transformRot(rotated_debug_link, refTomap);
            //pose_array.poses.push_back(rotated_debug_link);

            geometry_msgs::Pose debugg_link;
            debugg_link = transformPose(debug_link, eyeTobase);
            debugg_link = transformPose(debugg_link, refTomap);
            pose_array.poses.push_back(debugg_link);

            octomap::point3d origin_EE(transformed_cam_ray1.position.x, transformed_cam_ray1.position.y, transformed_cam_ray1.position.z);
            octomap::point3d direction_EE(rotated_debug_link.position.x,rotated_debug_link.position.y,rotated_debug_link.position.z);
            //ROS_INFO("Origin_EE: %f, %f, %f", origin_EE.x(), origin_EE.y(), origin_EE.z());
            //ROS_INFO("direction_EE: %f, %f, %f", direction_EE.x(), direction_EE.y(), direction_EE.z());
            sensor_pose_array.poses.push_back(rotated_debug_link);

            if (octree->castRay(origin_EE, direction_EE, end, true, 8)) {
                geometry_msgs::Pose pose;
                pose.position.x = end.x();
                pose.position.y = end.y();
                pose.position.z = end.z();
                pose.orientation.w = 1.0;        
                //pose_array.poses.push_back(pose);

                octomap::ColorOcTreeNode* node = octree->search(pose.position.x, pose.position.y, pose.position.z);
                
                if (targeted) {
                    if (node->getValue() == POTENTIAL_OBJECT) {
                        gain += 1;
                    }

                    if (node->getValue() == OCCLUSION) {
                        gain += 1;
                    }

                    /**
                    if (node->getOccupancy() >= 0.5 && node->getOccupancy() < 0.51) {
                        //unknown space
                        gain += 1;
                    }

                    if (node->getOccupancy() >= 0.65 && node->getOccupancy() < 0.67) {
                        //potential object
                        gain += 10;
                    }

                    if (node->getOccupancy() >= 0.52 && node->getOccupancy() < 0.53) {
                        //special occlusion
                        ROS_INFO("special occ %f", node->getOccupancy());
                        gain += 10;
                    } */

                } else {
                    if (node->getLogOdds() >= 0.0 && node->getLogOdds() < 0.1) {
                        
                        //unexplored
                        gain += 1;
                    }
                }
            }
        } // FOV loop ends */
        pub_debug.publish(pose_array);
        
        if (publish) {
            pub_sensor_poses.publish(sensor_pose_array);
        }
        return gain;
    }

void RayCaster::build_tree(octomap::ColorOcTree* unknown_octree, float map_min_x, float map_min_y, float map_max_x, float map_max_y,
                            float FOV, int FOV_resolution, int targeted /* = 0*/,
                            float heading_x /*= 0.0 */, float heading_y /*= 0.0*/, float goal_x /*= 0.0*/, float goal_y /*= 0.0*/) {
    // ========================================= RRT* parameters

    int num_sample(0);
    int max_sample(50);
    int max_step_size(1);
    int neighbourhood_radius(2.5);
    int max_sample_try(5000);
    int current_sample_try(0);
    float update_robot_radius(4);
    float map_search_radius(5);

    int n_local_max(5);
    int n_local(0);

    while (max_sample > num_sample || n_local_max > n_local) {
        current_sample_try++;
        if (current_sample_try > max_sample_try) {
            ROS_ERROR("Please teleop the robot to a safe position and run it again");
            return;
        }
        // ======================================================== Filtering samples
        auto V_rand = kd_tree_->randomSample(map_min_x, map_max_x,
                                        map_min_y, map_max_y);

        //ROS_INFO("Sampled at %f, %f", V_rand.x, V_rand.y);

        // ======================================================== collision checking potential new node
        auto nearest_node = kd_tree_->nn_search(V_rand);

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

            int grid_index = grid_2d.info.width*std::floor((newNodePosition.y - grid_2d.info.origin.position.y)/grid_2d.info.resolution) + std::floor((newNodePosition.x - grid_2d.info.origin.position.x)/grid_2d.info.resolution);
            if (grid_index < grid_2d.info.width * grid_2d.info.height) {
                if (grid_2d.data[grid_index] > GLOBAL_OCCUPANCY_COST_MAX) {
                    continue;
                }
            }
    
            
            if (checkPathCollision(&grid_2d, nearest_node->position, newNodePosition)) {
                continue;
            } else {
                // (checkConservativeCollision(unknown_octree, grid_2d, newNode->position.x, newNode->position.y)) {
                //    // sometimes steering command requires additional collision checker
                //    continue;
                //}

                float local_dist = std::sqrt(std::pow(newNodePosition.x - robot_pos.x, 2) + std::pow(newNodePosition.y - robot_pos.y, 2));
                if (local_dist < neighbourhood_radius) {
                    n_local++;
                }

                // set the cost
                double temp_cost_ = nearest_node->cost + max_step_size;

                // identify all the affected neighbours
                auto neighbours = kd_tree_->radius_search(newNodePosition, neighbourhood_radius);

                // for all neighbours check the cost then set the parent based on the lowest cost.
                KD_RRT::KDNode::KDNodePtr best_parent(nearest_node);

                    
                for (const auto& neighbour : neighbours) {
                    double temp_dist_ = std::sqrt(std::pow(newNodePosition.x - neighbour->position.x, 2) + std::pow(newNodePosition.y - neighbour->position.y, 2));
                    if (neighbour->cost + temp_dist_ < temp_cost_ && !checkPathCollision(&grid_2d, neighbour->position, newNodePosition)) {
                        temp_cost_ = neighbour->cost + temp_dist_;
                        best_parent = neighbour;
                    }
                }

                newNode->cost = temp_cost_;
                newNode->parent = best_parent;
                geometry_msgs::Point heading;
                float gain(0);
                if (targeted) {
                    heading.x = heading_x;
                    heading.y = heading_y;
                    newNode->heading = heading;
                    gain = compute_gain(newNode->position.x, newNode->position.y, heading_x, heading_y, unknown_octree, FOV, FOV_resolution, targeted);

                } else {
                    // randomise the heading
                    auto V_facing = kd_tree_->randomSample(map_min_x, map_max_x, map_min_y, map_max_y);
                    heading.x = V_facing.x;
                    heading.y = V_facing.y;
                    gain = compute_gain(newNode->position.x, newNode->position.y, V_facing.x, V_facing.y, unknown_octree, FOV, FOV_resolution, 0);
                    newNode->heading = heading;
                }

                newNode->gain = gain;
                newNode->value = gain - newNode->cost;
                kd_tree_->insert_node(newNode);
                

                // Require rewiring the neighbouring nodes if the cost is cheaper to go past this newly added node!
                
                for (const auto& neighbour : neighbours) {
                    double temp_dist_ = std::sqrt(std::pow(newNodePosition.x - neighbour->position.x, 2) + std::pow(newNodePosition.y - neighbour->position.y, 2));
                    if (newNode->cost + temp_dist_ < neighbour->cost && !checkPathCollision(&grid_2d, neighbour->position, newNodePosition)) {
                        neighbour->parent = newNode;
                        neighbour->cost = newNode->cost + temp_dist_;
                        neighbour->value = neighbour->gain - neighbour->cost;
                    }
                }

                int robot_dist = std::sqrt(std::pow(newNodePosition.x - robot_pos.x, 2) + std::pow(newNodePosition.y - robot_pos.y, 2));
                if (robot_dist < update_robot_radius) {
                    n_local++;
                } else {
                    num_sample++;
                }
            } // linear collision check ends
        } // nearest point isn't a null ptr
    } // sampling + building tree ends

}    

void RayCaster::normal_callback_backup(const octomap_msgs::Octomap::ConstPtr& map_msg) {
    ROS_INFO("Waiting for move_base action server to start.");
    action_client.waitForServer(); 

    if (robot_heading.x == 0.0 && robot_heading.y == 0.0) {
        ROS_INFO("waiting for odom to update..");
        return;
    }

    octomap::AbstractOcTree* octo_abs_tree_unknown = octomap_msgs::msgToMap(octomap_msg);
    octomap::ColorOcTree* octree = dynamic_cast<octomap::ColorOcTree*>(octo_abs_tree_unknown);

    if (!octree) {
        ROS_INFO("invalid octree");
        return;
    }

    float map_max_x = grid_2d.info.origin.position.x + grid_2d.info.width * grid_2d.info.resolution;
    float map_max_y = grid_2d.info.origin.position.y + grid_2d.info.height * grid_2d.info.resolution;
    float map_min_x = grid_2d.info.origin.position.x;
    float map_min_y = grid_2d.info.origin.position.y;

    float map_center_x = (map_max_x - map_min_x) / 2;
    float map_center_y = (map_max_y - map_min_y) / 2;

    float FOV = 60;
    int FOV_resolution = 20;

    /** 

    move_group_interface.getCurrentState();

    geometry_msgs::Pose target_pose = move_group_interface.getCurrentPose().pose;
    target_pose.position.x = 0.4;  // Example position
    target_pose.position.y = 0.1;
    target_pose.position.z = 0.4;
    target_pose.orientation.w = 1.0;  // Example orientation

    move_group_interface.setPoseTarget(target_pose);

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        // Execute the planned motion
        move_group_interface.execute(plan);
    }
    else
    {
        ROS_WARN("Planning failed.");
    }*/


    float gain = compute_gain(robot_pos.x, robot_pos.y, 7.0, 0.1, octree, FOV, FOV_resolution, 1, 1);
    ROS_INFO("gain is %f", gain);


    
}

void RayCaster::normal_callback(const octomap_msgs::Octomap::ConstPtr& map_msg) {
    if (reconstruction_started || reconstruction_finished) {
        return;
    }

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
    // Initialise two trees, one for octomap and one for unknown octomap for unknown voxels


    octomap::AbstractOcTree* octo_abs_tree = octomap_msgs::msgToMap(*map_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octo_abs_tree);

    octomap::AbstractOcTree* octo_abs_tree_unknown = octomap_msgs::msgToMap(octomap_msg);
    octomap::ColorOcTree* unknown_octree = dynamic_cast<octomap::ColorOcTree*>(octo_abs_tree_unknown);

    if (!unknown_octree) {
        ROS_INFO("invalid octree");
        return;
    }
    //octomap::ColorOcTree* unknown_octree = new octomap::ColorOcTree(unknown_octree_temp_->getResolution());

    // get the map bounds, map is from flattening those octomaps into 2d 
    float map_max_x = grid_2d.info.origin.position.x + grid_2d.info.width * grid_2d.info.resolution;
    float map_max_y = grid_2d.info.origin.position.y + grid_2d.info.height * grid_2d.info.resolution;
    float map_min_x = grid_2d.info.origin.position.x;
    float map_min_y = grid_2d.info.origin.position.y;

    Vec3 minBound(map_min_x, map_min_y, 0.0);
    Vec3 maxBound(map_max_x, map_min_y, 1.5);

    // ========================================= check if the search has begun and run the initial behaviour.
    if (!search_started) {
        turn_around();
        search_started = true;
        
        int pi_res = 10;
        float pi_step = (2 * M_PI) / pi_res;
        float force_root_search_radius(2.5);
        move_base_msgs::MoveBaseGoal goal;

        for (int i = 0; i <= pi_res; i++) {
            float pi_x = force_root_search_radius * cos(i * pi_step);
            float pi_y = force_root_search_radius * sin(i * pi_step);
            geometry_msgs::Point goal_pose;
            goal_pose.x = pi_x + robot_pos.x;
            goal_pose.y = pi_y + robot_pos.y;
            // Check for obstacle collision
            if (checkObstacleCollision(unknown_octree, grid_2d, goal_pose.x, goal_pose.y)) {
                continue;
            }   

            // Calculate yaw based on robot's heading
            double yaw = atan2(-pi_y, -pi_x);
            if (std::isnan(yaw)) {
                yaw = 0.0; // Default value for NaN
            }

            // Create quaternion from yaw
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, yaw);
            quaternion.normalize();

            // Populate the goal message
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position = goal_pose;
            goal.target_pose.pose.orientation.x = quaternion.x();
            goal.target_pose.pose.orientation.y = quaternion.y();
            goal.target_pose.pose.orientation.z = quaternion.z();
            goal.target_pose.pose.orientation.w = quaternion.w();

            // Send the goal and wait for the result
            //action_client.sendGoal(goal);
            //bool success = action_client.waitForResult(ros::Duration(10.0));
            break;
        }
        spin_done = true;
    }

    // ========================================= Check to initialise the tree
    
    if (!kd_tree_initialised) {
        kd_tree_initialised = true;

        geometry_msgs::Point start_rrt_pt;
        start_rrt_pt.x = robot_pos.x;
        start_rrt_pt.y = robot_pos.y;
        start_rrt_pt.z = 0.0;

        KD_RRT::KDNode::KDNodePtr rootNode = std::make_shared<KD_RRT::KDNode>(start_rrt_pt);
        kd_tree_->insert_node(rootNode);
        ROS_INFO("Normal Root node %f, %f", rootNode->position.x, rootNode->position.y);
        root_node = rootNode;
        
    }

    // ========================================= debugging pose array
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "map";


    // =========================================================== Decretisiation of FOV
    float FOV = 40; // degree * 1 rad/degree
    int FOV_resolution = 50;
    float FOV_step = FOV / FOV_resolution;

    build_tree(unknown_octree, map_min_x, map_min_y, map_max_x, map_max_y, FOV, FOV_resolution);
    

    // =================================================================== Calculating the best gain node
    float value(0.0);

    KD_RRT::KDNode::KDNodePtr best_node = kd_tree_->find_highest_gain_node();
    
    if (best_node == nullptr) {
        ROS_INFO("gain be all zeros?");
        return;
    }

    kd_tree_->print_tree();
    kd_tree_->visualise_tree(best_node->gain);
    publish_best_node(best_node->position);
    // =================================================================== Generate a path to the node and execute the first one

    // move to the best node
    std::deque<KD_RRT::KDNode::KDNodePtr> path;

    KD_RRT::KDNode::KDNodePtr node_it = best_node;

    //path.push_front(node_it);
    while (node_it->parent != nullptr) {
        path.push_front(node_it);
        node_it = node_it->parent;
    }

    size_t current_path_index(0);
    size_t max_path_index(5);
    current_search_done = false;
    while (current_path_index < std::min(path.size(), max_path_index)) {
        auto path_node = path[current_path_index];
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

        if (!reconstruction_started) {
        action_client.sendGoal(goal);
        bool success = action_client.waitForResult(ros::Duration(5.0)); // Waits for the result with a timeout of 30 seconds
        }
        current_path_index++;
    }
    current_search_done = true;

    if (best_node->gain < 600) {
        start_3d_search = true;
        retry++;
        //turn();
        ROS_INFO("No more gain, retry is at %d", retry);
        if (retry > 1) {
            ROS_INFO("Completed the scan, going to where I started");

            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = start_pose;
            action_client.sendGoal(goal);
            bool success = action_client.waitForResult(ros::Duration(10.0)); // Waits for the result with a timeout of 30 seconds
            reconstruction_finished = true;
            return;
        }
            
    } 

    ROS_INFO("normal callback clearing the tree?");
    kd_tree_->clear_tree();
    kd_tree_initialised = false;

}



void RayCaster::cuboid_callback(const visualization_msgs::MarkerArray::ConstPtr& search_area_msg, const octomap_msgs::Octomap::ConstPtr& map_msg) {

    reconstruction_started = true; 

    if (reconstruction_finished) {
        ROS_INFO("Mission has finished");

        geometry_msgs::PoseStamped flag;
        pub_recon_finish.publish(flag);
        action_client.cancelAllGoals();
        return;
    } 

    if (!spin_done) {
        return;
    }
    
    // wait for odom to update --
    if (robot_heading.x == 0.0 && robot_heading.y == 0.0) {
        ROS_INFO("waiting for odom to update..");
        return;
    } 

    if (!current_search_done || kd_tree_initialised == true) {
        ROS_INFO("waiting for search to finish..");
        return;
    }

    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*map_msg);
    octomap::ColorOcTree* octree = dynamic_cast<octomap::ColorOcTree*>(tree);

    // get the map bounds
    float map_max_x = grid_2d.info.origin.position.x + grid_2d.info.width * grid_2d.info.resolution;
    float map_max_y = grid_2d.info.origin.position.y + grid_2d.info.height * grid_2d.info.resolution;
    float map_min_x = grid_2d.info.origin.position.x;
    float map_min_y = grid_2d.info.origin.position.y;

    double rec_min_x = search_area_msg->markers[0].points[0].x;
    double rec_max_x = search_area_msg->markers[0].points[2].x;
    double rec_min_y = search_area_msg->markers[0].points[0].y;
    double rec_max_y = search_area_msg->markers[0].points[2].y;

    ROS_INFO("min x : %f, min y: %f, max x: %f, max y: %f", rec_min_x, rec_min_y, rec_max_x, rec_max_y);


    double rec_centre_x = rec_min_x + (rec_max_x - rec_min_x) / 2;
    double rec_centre_y = rec_min_y + (rec_max_y - rec_min_y) / 2;

    Vec3 minBound(rec_min_x, rec_min_y, 0.0);
    Vec3 maxBound(rec_max_x, rec_max_y, 10.0);

    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "map";

    if (!kd_tree_initialised) {
        // different mode of control
        kd_tree_->clear_tree();
        geometry_msgs::Point start_rrt_pt;
        start_rrt_pt.x = robot_pos.x;
        start_rrt_pt.y = robot_pos.y;
        start_rrt_pt.z = 0.0;

        KD_RRT::KDNode::KDNodePtr rootNode = std::make_shared<KD_RRT::KDNode>(start_rrt_pt);
        auto V_facing = kd_tree_->randomSample(map_min_x, map_max_x, map_min_y, map_max_y);
        geometry_msgs::Point heading;
        heading.x = V_facing.x;
        heading.y = V_facing.y;
        rootNode->heading = heading;
        
        kd_tree_->insert_node(rootNode);
        ROS_INFO("Recon Root node %f, %f", rootNode->position.x, rootNode->position.y);
        root_node = rootNode;
        current_robot_node = nullptr;

    } else if (!current_search_done) {
        return;
    }

    if (octree) // can be NULL
    {
        
        // =========================================================== Decretisiation of FOV
        float FOV = 40; // degree * 1 rad/degree
        int FOV_resolution = 50;
        float FOV_step = FOV / FOV_resolution;


        // need to fix what if the robot is too far from the search area??
        float heading_x = rec_centre_x;
        float heading_y = rec_centre_y;
        // mark on the octree?
        build_tree(octree, map_min_x, map_min_y, map_max_x, map_max_y, FOV, FOV_resolution, 1, heading_x, heading_y);

        KD_RRT::KDNode::KDNodePtr best_node = kd_tree_->find_highest_gain_node();
    
        if (best_node == nullptr) {
            ROS_INFO("gain be all zeros?");
            return;
        }

        kd_tree_->print_tree();
        kd_tree_->visualise_tree(best_node->gain);
        publish_best_node(best_node->position);

    // =================================================================== Generating a path to the best node
        // move to the best node
        /**
        move_base_msgs::MoveBaseGoal goal;

        double yaw = atan2(best_node->heading.y - best_node->position.y, best_node->heading.x - best_node->position.x); // Calculate yaw based on direction
        if (std::isnan(yaw)) {
            yaw = 0.0; // Set a default value if yaw is NaN
        }

        // Create quaternion from roll, pitch, and yaw
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        q.normalize(); // Normalize the quaternion

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = best_node->position;
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        action_client.cancelAllGoals();
        action_client.sendGoal(goal);
        bool success = action_client.waitForResult(ros::Duration(50.0)); // Waits for the result with a timeout of 30 seconds
        compute_gain(best_node->position.x, best_node->position.y, heading_x, heading_y, octree, FOV, FOV_resolution, 1, 1);

         */
        std::deque<KD_RRT::KDNode::KDNodePtr> path;

        KD_RRT::KDNode::KDNodePtr node_it = best_node;
        while (node_it->parent != nullptr) {
            path.push_front(node_it);
            node_it = node_it->parent;
        }
        //path.push_front(node_it);        

        ROS_INFO("size of the path %li", path.size());
        int current_path(0);
        for (auto path_node : path) {
            move_base_msgs::MoveBaseGoal goal;

            double yaw = atan2(path_node->heading.y - path_node->position.y, path_node->heading.x - path_node->position.x); // Calculate yaw based on direction
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
            if (path_node == best_node) {
                action_client.sendGoal(goal);
                bool success = action_client.waitForResult(ros::Duration(5.0)); // Waits for the result with a timeout of 30 seconds
                
            } else {
                action_client.sendGoal(goal);
                bool success = action_client.waitForResult(ros::Duration(7.0)); // Waits for the result with a timeout of 30 seconds
            }
            // publish rays so that the map can update
            compute_gain(path_node->position.x, path_node->position.y, heading_x, heading_y, octree, FOV, FOV_resolution, 1, 1);
            
            //pub_sensor_pose.publish(poseStampedToSend);
            //current_path++;

        }


        // =========================================================== Terminating condition
        if (best_node->gain < 600) {
            start_3d_search = true;
            retry++;
            
            ROS_INFO("No more gain, retry is at %d", retry);
            if (retry > 1) {
                ROS_INFO("Completed the scan.");
                
                /** 
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = start_pose;
                action_client.sendGoal(goal);
                bool success = action_client.waitForResult(ros::Duration(30.0)); // Waits for the result with a timeout of 30 seconds*/
                reconstruction_finished = true;
                return;
            }
            turn();
        } 

        kd_tree_->clear_tree();
        kd_tree_initialised = false;

        // add to pose_array to visualise in debug_marker_ray outside the loop
        pub_debug.publish(pose_array);

    }
}


void RayCaster::publish_best_node(const geometry_msgs::Point position) {
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


void RayCaster::occ_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& ogrid)
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
    h.position.z = 0.8;
    match_corners.poses.push_back(h);


    pub_search_cuboid.publish(match_corners);
    

}

void RayCaster::unknown_tree_callback(const octomap_msgs::Octomap& msg) {
    octomap_msg = msg;
    unknown_tree_arrived = true;
}


void RayCaster::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {

    geometry_msgs::PoseStamped ros_pos_odom;
    geometry_msgs::PoseStamped ros_pos_map;
    try {
        geometry_msgs::TransformStamped odom_map_transform = buffer.lookupTransform("map","odom", ros::Time(0));
        ros_pos_odom.header = odom_msg->header;
        ros_pos_odom.pose = odom_msg->pose.pose;
        tf2::doTransform(ros_pos_odom, ros_pos_map, odom_map_transform);
    } catch (tf2::TransformException &ex) { 
        ROS_WARN("Could not transform point: %s", ex.what());
        return;
    }


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
    Vec2 direction(cos(yaw), sin(yaw));
    
    robot_heading.x = direction.x;
    robot_heading.y = direction.y;
    robot_heading.z = 0.0;
}

void RayCaster::look(float yaw) {
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names.push_back("fr3_joint6");
    jt.points.resize(1);
    jt.points[0].positions.resize(1);
    jt.points[0].positions[0] = yaw; // looking down;
    jt.points[0].time_from_start = ros::Duration(3.0);
    pub_franka_control.publish(jt);
}


void RayCaster::turn_around() {
    // move the camera down
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names.push_back("fr3_joint6");
    jt.points.resize(1);
    jt.points[0].positions.resize(1);
    jt.points[0].positions[0] = 2.4; // looking down;
    jt.points[0].time_from_start = ros::Duration(1.0);
    pub_franka_control.publish(jt);

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
    pub_franka_control.publish(jt);

}

void RayCaster::turn() {

    // make the base turn
    geometry_msgs::Twist command_vel;
    float angular_velocity = 0.5;
    ros::Rate rate(10);
    float duration(6.28318530 /  (angular_velocity));
    ros::Time start_time = ros::Time::now();
    while (ros::Time::now() - start_time < ros::Duration(duration)) {
        command_vel.angular.z = angular_velocity;
        pub_cmd_vel.publish(command_vel);
        rate.sleep();
    }
    
    command_vel.angular.z = 0.0;
    pub_cmd_vel.publish(command_vel);

}

void RayCaster::send_velocity(double distance, double velocity, double dir_x, double dir_y) {
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


} // namespace ends

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_casting_node");
    NBV::RayCaster node;
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
