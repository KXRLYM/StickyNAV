        if (current_node_it == nullptr) {
            // create a path all the way to the origin
            while (node_it->parent != nullptr) {
                path.push_front(node_it);
                node_it = node_it->parent;
            }
        } else {
            // starting from the current_robot_node

            KD_RRT::KDNode::KDNodePtr common_parent(nullptr);
            bool parent_found(false);

            while (current_node_it->parent != nullptr && !parent_found) { 
                // start iterating through the current path we are on
                KD_RRT::KDNode::KDNodePtr temp_node_it = best_node;

                while (temp_node_it->parent != nullptr) {
                    // start iterating through the current best path from the origin
                    if (current_node_it == temp_node_it) {
                        common_parent = current_node_it;
                        parent_found = true;
                        break;
                    }
                    if (current_node_it == temp_node_it->parent) {
                        common_parent = current_node_it;
                        parent_found = true;
                        break;
                    }
                    if (current_node_it->parent == temp_node_it->parent) {
                        common_parent = temp_node_it->parent;
                        parent_found = true;
                        break;
                    }
                    temp_node_it = temp_node_it->parent;
                }

                if (parent_found) {
                    // going all the way back to the common parent and to the best node
                    while (node_it != common_parent) {
                        path.push_front(node_it);
                        node_it = node_it -> parent;
                    }
                    std::deque<KD_RRT::KDNode::KDNodePtr>::iterator it = path.begin();
                    if (current_node_it == common_parent) {
                        break;
                    } 

                    while (current_node_it-> parent != common_parent) {
                        path.insert(it, current_node_it->parent);
                        it++;
                        current_node_it = current_node_it->parent;
                    }  
                    break;

                } else {
                    // move one step back in the robot path
                    current_node_it = current_node_it->parent;
                }
            
            }

            if (!parent_found) {
                // going all the way back to the origin and to the best node
                while (node_it->parent != nullptr) {
                    path.push_front(node_it);
                    node_it = node_it->parent;
                }
                path.push_front(current_robot_node); // at the origin now
                std::deque<KD_RRT::KDNode::KDNodePtr>::iterator it = path.begin();
                while (current_robot_node->parent != nullptr) {
                    path.insert(it, current_robot_node->parent);
                    it++;
                    current_robot_node = current_robot_node->parent;
                } 

            }

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
            //pose_array.poses.push_back(ray_dir_map);

            octomap::point3d origin_EE(ray_map.position.x, ray_map.position.y, ray_map.position.z);
            octomap::point3d direction_EE(ray_dir_map.position.x - ray_map.position.x, ray_dir_map.position.y -  ray_map.position.y , ray_dir_map.position.z - ray_map.position.z);
        

            if (octree->castRay(origin_EE, direction_EE, end, true, 5)) {
                geometry_msgs::Pose pose;
                pose.position.x = end.x();
                pose.position.y = end.y();
                pose.position.z = end.z();
                pose.orientation.w = 1.0;        
                pose_array.poses.push_back(pose);

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
                    if (node->getOccupancy() >= 0.5 && node->getOccupancy() < 0.51) {
                        
                        //unexplored
                        gain += 1;
                    }
                }
            }
        } // FOV loop ends */
        pub_debug.publish(pose_array);
        
        if (publish) {
            pub_sensor_poses.publish(pose_array);
        }
        return gain;
    }