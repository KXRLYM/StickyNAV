#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import numpy as np
import json

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from tf2_geometry_msgs import do_transform_pose, do_transform_point
from tf2_ros import Buffer, TransformListener
from message_filters import Subscriber, ApproximateTimeSynchronizer, TimeSynchronizer
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
        self.attn_array_sub = Subscriber('/attn_points_camera', PoseArray)
        self.depth_sub = Subscriber('/realsense/depth/color/points', PointCloud2) # depth optical frame
        #self.depth_sub = Subscriber('/octomap_point_cloud_centers', PointCloud2)

        self.sync = TimeSynchronizer([self.pose_array_sub, self.depth_sub], queue_size = 50)
        self.sync.registerCallback(self.syncCallback)

        # Publisher for the transformed poses
        self.matched_pose_pub = rospy.Publisher('matched_pose_array', PoseArray, queue_size=10) # depth frame
        self.transformed_pose_pub = rospy.Publisher('transformed_poses', PoseArray, queue_size=10) # world frame
        self.transformed_attn_pub = rospy.Publisher('transformed_attn_poses', PoseArray, queue_size=10) # world frame
        self.match_marker_pub = rospy.Publisher('match_centeroid', MarkerArray, queue_size=10)
        self.matched_point_cloud_pub = rospy.Publisher('matched_pt_segments', PointCloud2, queue_size=10)
        self.camera_info_sub = rospy.Subscriber('/realsense/depth/camera_info', CameraInfo, self.cam_info_callback)

        #self.keymap = {}
        self.pc2_msg = None
        self.matches = []
        self.centeroid = [0,0]
        self.timer = rospy.Timer(rospy.Duration(1), self.pub)
        self.publisher = rospy.Publisher('matches', PoseArray, queue_size=10)
        self.thresh = 100
        self.radius = 1.5

        self.cam_info_ = None
        self.cam_info_received_ = False

    def cam_info_callback(self, cam_info):
        self.cam_info_ = cam_info
        self.cam_info_received_ = True

    def pub(self, timer):
        if (self.pc2_msg is not None) :
            self.matched_point_cloud_pub.publish(self.pc2_msg)

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


    def syncCallback(self, points, depth):
        '''
        points are pose array of target points
        depth are pointclouds
        we are assuming that the intrinsics of depth and colours are the same.
        '''
        if self.cam_info_received_ == False:
            rospy.logerr("Waiting for camera info to be received..")
            return
    
        fx = self.cam_info_.P[0]  # Focal length along the x-axis (in pixels)
        fy = self.cam_info_.P[5]  # Focal length along the y-axis (in pixels)
        cx = self.cam_info_.P[2]  # Principal point x-coordinate (in pixels)
        cy = self.cam_info_.P[6]  # Principal point y-coordinate (in pixels)

        matched_idx = []

        cloud_points = list(pc2.read_points(depth, skip_nans=True, field_names = ("x", "y", "z")))
        rospy.loginfo("length of the point cloud is %d", len(cloud_points))

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

        transformed_array = PoseArray()

        if (self.tf_buffer.can_transform('map', 'front_realsense_gazebo', depth.header.stamp, rospy.Duration(1.0))):

            for point in matched_points:
                new_point = self.transform_point_to_map(point, depth.header.stamp)
                if new_point is not None:
                    new_pose = Pose()
                    new_pose.position.x = new_point[0]
                    new_pose.position.y = new_point[1]
                    new_pose.position.z = new_point[2]
                    new_pose.orientation.w = 1.0
                    transformed_array.poses.append(new_pose)

            header = Header()
            header.stamp = depth.header.stamp
            header.frame_id = "map"
            transformed_array.header = header

            self.transformed_pose_pub.publish(transformed_array)


    def transform_point_to_map(self, point, stamp, target_frame='map', source_frame='front_realsense_gazebo'):
        # Create a PointStamped message for the point
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.header.stamp = stamp
        point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = point

        try:
            # Get the transform from source_frame to target_frame
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(0.5))
            # Transform the point
            transformed_point = do_transform_point(point_stamped, transform)
            return [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform failed: %s", e)
            return None
    
if __name__ == '__main__':
    try:
        transformer = Transformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


