#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <map>
#include <tf/transform_listener.h> // For tf
#include <pcl_conversions/pcl_conversions.h>


class ClusterNode {
public:
    ClusterNode() : tf_listener(), received(false) {
        ros::NodeHandle nh;
        sub_real_cloud_ = nh.subscribe("realsense/depth/color/points", 1, &ClusterNode::realCloudCallback, this);
        sub_matched_cloud_ = nh.subscribe("matched_pt_segments", 1, &ClusterNode::matchedCloudCallback, this);
        pub_cluster_ = nh.advertise<sensor_msgs::PointCloud2>("clustered_point_cloud", 1);
    }

    void realCloudCallback(const sensor_msgs::PointCloud2ConstPtr& real_msg) {
        /** 
        pcl::PointXYZ start_point();
        float radius = 5;
        if (!received) {
            return;
        } else {
            start_point = match_clusters_.points[0];
        }
        
        pcl::PCLPointCloud2::Ptr real_cloud(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*real_msg, *real_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*real_cloud,*input_cloud);


        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        try {
            // Create a ROS Transform
            pcl_ros::transformPointCloud ("map", *input_cloud, *transformed_cloud, tf_listener);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Transform error: %s", ex.what());
            return;  // Handle or skip the processing if transform fails
        }        
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : transformed_cloud->points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
                std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z) || point.z < 0.05) {
                continue;  // Skip invalid points
            }
            clean_cloud->points.push_back(point);
        }
        ROS_INFO("coming..");

        pcl::search::KdTree<pcl::PointXYZ> tree;
        tree.setInputCloud(clean_cloud);

        std::vector<int> point_indices;
        std::vector<float> point_distances;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (tree.radiusSearch(start_point, radius, point_indices, point_distances) > 0) {
            // Create a point cloud for the cluster
            
            for (int idx : point_indices) {
                cluster_cloud->points.push_back(input_cloud->points[idx]);
            }
        }

        ROS_INFO("sizes %li", cluster_cloud->points.size());
        if (cluster_cloud->points.size() == 0) {
            return;
        }
        pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cluster_cloud, *output_cloud);
        sensor_msgs::PointCloud2 cluster_msg;
        pcl_conversions::fromPCL(*output_cloud, cluster_msg);
        
        cluster_msg.header.stamp = ros::Time::now();
        cluster_msg.header.frame_id = "map"; // Adjust frame_id accordingly
        pub_cluster_.publish(cluster_msg);*/
    }

    void matchedCloudCallback(const sensor_msgs::PointCloud2ConstPtr& matched_msg) {
        pcl::PCLPointCloud2::Ptr matched_cloud(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*matched_msg, *matched_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*matched_cloud,*input_cloud);

        match_clusters_ = *input_cloud;
        received = true;

        /*
        // Check which cluster contains points from matched_cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr selected_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : input_cloud->points) {
            for (const auto& cluster : clusters_) {
                for (const auto& cluster_point : cluster->points) {
                    if (point.x == cluster_point.x && point.y == cluster_point.y && point.z == cluster_point.z) {
                        *selected_cluster += *cluster;
                        break;
                    }
                }
            }
        }

        // Publish the selected cluster
        pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*selected_cluster, *output_cloud);
        sensor_msgs::PointCloud2 cluster_msg;
        pcl_conversions::fromPCL(*output_cloud, cluster_msg);
        
        cluster_msg.header.stamp = ros::Time::now();
        cluster_msg.header.frame_id = "map"; // Adjust frame_id accordingly
        //pub_cluster_.publish(cluster_msg);*/
    }

private:
    ros::Subscriber sub_real_cloud_;
    ros::Subscriber sub_matched_cloud_;
    ros::Publisher pub_cluster_;
    tf::TransformListener tf_listener;
    pcl::PointCloud<pcl::PointXYZ> match_clusters_;
    bool received;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cluster_node");
    ClusterNode cluster_node;
    ros::spin();
    return 0;
}
