#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import numpy as np
import json

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


class Transformer:
    def __init__(self):
        rospy.init_node('transformer')

        rospy.loginfo("starting transformer!")

        
        # Initialize tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Subscribe to the input PoseArray topic
        self.pose_array_sub = Subscriber('/matched_points_camera', PoseArray)
        self.depth_sub = Subscriber('/realsense/depth/color/points', PointCloud2) # depth optical frame
        #self.depth_sub = Subscriber('/octomap_point_cloud_centers', PointCloud2)
        self.camera_info_sub = Subscriber('/realsense/depth/camera_info', CameraInfo)
        self.keymap_sub = Subscriber('/keypoint_map', String)

        self.sync = ApproximateTimeSynchronizer([self.pose_array_sub, self.depth_sub, self.camera_info_sub], queue_size = 1, slop = 0.3)
        self.sync.registerCallback(self.syncCallback)

        self.keymap_sync = ApproximateTimeSynchronizer([self.keymap_sub, self.depth_sub, self.camera_info_sub], queue_size = 1, slop = 0.5, allow_headerless=True)
        self.keymap_sync.registerCallback(self.keymap_callback)

        # Publisher for the transformed poses
        self.matched_pose_pub = rospy.Publisher('matched_pose_array', PoseArray, queue_size=10) # depth frame
        self.transformed_pose_pub = rospy.Publisher('transformed_poses', PoseArray, queue_size=10) # world frame
        self.match_marker_pub = rospy.Publisher('match_centeroid', MarkerArray, queue_size=10)

        self.keymap = {}
        self.matches = []
        self.centeroid = [0,0]
        self.timer = rospy.Timer(rospy.Duration(1), self.pub)
        self.publisher = rospy.Publisher('matches', PoseArray, queue_size=10)
        self.thresh = 100
        self.radius = 1.5

    def string_to_pair(self, string):
        """Convert a string 'x:y' into a tuple (x, y)."""
        return tuple(map(int, map(float, string.split(':'))))

    def pair_to_string(self, pair):
        """Convert a tuple (x, y) into a string 'x:y'."""
        return f"{pair[0]}:{pair[1]}"

    def keymap_callback(self, keymap_msg, depth, cam_info):
        """Process the incoming JSON string and update the keymap."""
        # Convert ROS String message to Python string
        keymap_str = keymap_msg.data
        
        # Parse the JSON string
        try:
            keymap = json.loads(keymap_str)
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing JSON: {e}")
            return

        # Update keymap
        for key_str, value_str in keymap.items():
            key_tuple = self.string_to_pair(key_str)
            value_tuple = self.string_to_pair(value_str)

            if key_tuple in self.keymap:
                # Key exists, update cumulative sum and count
                current_sum, count = self.keymap[key_tuple]
                new_sum = (current_sum[0] + value_tuple[0], current_sum[1] + value_tuple[1])
                new_count = count + 1
                self.keymap[key_tuple] = (new_sum, new_count)
            else:
                # Key does not exist, initialize with current value
                self.keymap[key_tuple] = (value_tuple, 1)

        # Print the map with running averages (optional)
        for key, (cumulative_sum, count) in self.keymap.items():
            average = (cumulative_sum[0] // count, cumulative_sum[1] // count)  # Floating-point division
            rospy.loginfo(f"Key: {self.pair_to_string(key)}, Running Average: {self.pair_to_string(average)}")
        
        fx = cam_info.P[0]  # Focal length along the x-axis (in pixels)
        fy = cam_info.P[5]  # Focal length along the y-axis (in pixels)
        cx = cam_info.P[2]  # Principal point x-coordinate (in pixels)
        cy = cam_info.P[6]  # Principal point y-coordinate (in pixels)

        matched_idx = []

        cloud_points = list(pc2.read_points(depth, skip_nans=True, field_names = ("x", "y", "z")))

        for i, point in enumerate(cloud_points):
            if (point[2] == 0.0):
                continue
            u = int((point[0] * fx / point[2]) + cx)
            v = int((point[1] * fy / point[2]) + cy)

            for key, (cumulative_sum, count) in self.keymap.items():
                if (cumulative_sum[0] // count == u and cumulative_sum[1] // count == v):
                    matched_idx.append(i)
                    print("matched!")
                    break

        
        matched_points = [cloud_points[i] for i in matched_idx]

        pose_array = PoseArray()
        pose_array.header = depth.header    
        poses = []

        for point in matched_points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]
            poses.append(pose)

        pose_array.poses = poses


        # transformation to the world coordinate frame
        rospy.sleep(0.5) #  otherwise transformation error saying information is coming too quickly

        transformed_array = PoseArray()
        transformed_array.header = depth.header
        transformed_poses = []

        if (self.tf_buffer.can_transform('map', 'front_realsense_gazebo', rospy.Time(), rospy.Duration(1.0))):
            #trans = self.tf_buffer.lookup_transform('odom', 'camera_depth_optical_frame', rospy.Time())
            #tx = trans.transform.translation.x
            #ty = trans.transform.translation.y
            #tz = trans.transform.translation.z
            #rx = trans.transform.rotation.x
            #ry = trans.transform.rotation.y
            #rz = trans.transform.rotation.z

            #rospy.loginfo("tx: %f, ty: %f, tz: %f, rx: %f, ry: %f, rz: %f", tx, ty, tz, rx, ry, rz)

            for pose in poses:
                # Convert PoseStamped to target frame
                pose_stamped = PoseStamped()
                pose_stamped.header = depth.header
                pose_stamped.header.frame_id = 'front_realsense_gazebo'
                pose_stamped.pose = pose
                try:
                    # Perform the transform
                    transformed_pose = self.tf_buffer.transform(pose_stamped, 'map')
                    pose_msg = Pose()
                    pose_msg.position = transformed_pose.pose.position
                    pose_msg.orientation = transformed_pose.pose.orientation
                    transformed_poses.append(pose_msg)
                    self.matches.append(pose_msg)
                    #if (len(self.matches) + 1  < self.thresh):
                    #    self.matches.append(pose_msg)
                    #else:
                    #    del self.matches[0]
                    #    self.matches.append(pose_msg)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("Failed to transform pose: {}".format(str(e)))
                    return

        transformed_array.poses = transformed_poses
        transformed_array.header.frame_id = 'map' 
        #self.publisher.publish(transformed_array)


    def pub(self, timer):
        
        pose = PoseArray()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom"
        pose.poses = self.matches
        rospy.loginfo("Currently %i poses in store", len(self.matches))
        if len(self.matches) > 0:
            self.publisher.publish(pose)
            x_avg = 0
            y_avg = 0
            for pose in self.matches:
                x_avg += pose.position.x
                y_avg += pose.position.y
            
            self.centeroid[0] = x_avg/len(self.matches)
            self.centeroid[1] = y_avg/len(self.matches)

            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.id = 99
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.centeroid[0]
            marker.pose.position.y = self.centeroid[1]
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
            for i in range(8):
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = self.radius * np.cos( i * (np.pi/4)) + self.centeroid[0]
                marker.pose.position.y = self.radius * np.sin( i * (np.pi/4)) + self.centeroid[1]
                marker.pose.position.z = 0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)
                self.match_marker_pub.publish(marker_array)
            
            self.matches = []



    def syncCallback(self, points, depth, cam_info):
        '''
        points are pose array of target points
        depth are pointclouds
        we are assuming that the intrinsics of depth and colours are the same.
        

        fx = cam_info.P[0]  # Focal length along the x-axis (in pixels)
        fy = cam_info.P[5]  # Focal length along the y-axis (in pixels)
        cx = cam_info.P[2]  # Principal point x-coordinate (in pixels)
        cy = cam_info.P[6]  # Principal point y-coordinate (in pixels)

        matched_idx = []

        cloud_points = list(pc2.read_points(depth, skip_nans=True, field_names = ("x", "y", "z")))

        for i, point in enumerate(cloud_points):
            if (point[2] == 0.0):
                continue
            u = int((point[0] * fx / point[2]) + cx)
            v = int((point[1] * fy / point[2]) + cy)
            for pose in points.poses:
                if (pose.position.x == u and pose.position.y == v):
                    matched_idx.append(i)
                    break
        
        matched_points = [cloud_points[i] for i in matched_idx]

        pose_array = PoseArray()
        pose_array.header = depth.header    
        poses = []

        for point in matched_points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]
            poses.append(pose)

        pose_array.poses = poses


        self.matched_pose_pub.publish(pose_array)

        # transformation to the world coordinate frame
        rospy.sleep(0.5) #  otherwise transformation error saying information is coming too quickly

        transformed_array = PoseArray()
        transformed_array.header = depth.header
        transformed_poses = []

        if (self.tf_buffer.can_transform('map', 'front_realsense_gazebo', rospy.Time(), rospy.Duration(1.0))):
            #trans = self.tf_buffer.lookup_transform('odom', 'camera_depth_optical_frame', rospy.Time())
            #tx = trans.transform.translation.x
            #ty = trans.transform.translation.y
            #tz = trans.transform.translation.z
            #rx = trans.transform.rotation.x
            #ry = trans.transform.rotation.y
            #rz = trans.transform.rotation.z

            #rospy.loginfo("tx: %f, ty: %f, tz: %f, rx: %f, ry: %f, rz: %f", tx, ty, tz, rx, ry, rz)

            for pose in poses:
                # Convert PoseStamped to target frame
                pose_stamped = PoseStamped()
                pose_stamped.header = depth.header
                pose_stamped.header.frame_id = 'front_realsense_gazebo'
                pose_stamped.pose = pose
                try:
                    # Perform the transform
                    transformed_pose = self.tf_buffer.transform(pose_stamped, 'map')
                    pose_msg = Pose()
                    pose_msg.position = transformed_pose.pose.position
                    pose_msg.orientation = transformed_pose.pose.orientation
                    transformed_poses.append(pose_msg)
                    self.matches.append(pose_msg)
                    #if (len(self.matches) + 1  < self.thresh):
                    #    self.matches.append(pose_msg)
                    #else:
                    #    del self.matches[0]
                    #    self.matches.append(pose_msg)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("Failed to transform pose: {}".format(str(e)))
                    return

        transformed_array.poses = transformed_poses
        transformed_array.header.frame_id = 'map' 
        self.transformed_pose_pub.publish(transformed_array)'''
        
        
if __name__ == '__main__':
    try:
        transformer = Transformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


