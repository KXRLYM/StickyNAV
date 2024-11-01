#include"ros/ros.h"
#include"sensor_msgs/JointState.h"

#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <fstream>
#include <vector>

#include <octomap_msgs/GetOctomap.h>
#include <std_msgs/Float32MultiArray.h>

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ray.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// #include "quadtree.h"


static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;

static constexpr unsigned char OBJECT_DETECTED = 255;
static constexpr unsigned char CANDIDATES = 125; 

static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

class NBVNode
{
    public:
        NBVNode(int radius, double alpha):
            nh{},
            pub_pgrid(nh.advertise<nav_msgs::OccupancyGrid>("test_map", 5)),
            sub_ogrid(nh.subscribe("/projected_map", 1000, &NBVNode::ogrid_callback, this)),
            pub_marker(nh.advertise<visualization_msgs::MarkerArray>("bounding_box", 5)),
            timer(nh.createTimer(ros::Duration(0.1), &NBVNode::main_loop, this)),
            pub_rectangle(nh.advertise<geometry_msgs::PoseArray>("rectangle", 5)),
            radius_(radius),
            alpha_(alpha)
            {

            }

            void ogrid_callback(const nav_msgs::OccupancyGrid & ogrid)
            {

                // Firstly read in the metadata
                std_msgs::Header header = ogrid.header;
                nav_msgs::MapMetaData info = ogrid.info;
                int32_t height = ogrid.info.height;
                int32_t width = ogrid.info.width;
                nav_msgs::OccupancyGrid* pgrid = new nav_msgs::OccupancyGrid();
                ROS_INFO("Got map %d %d", info.width, info.height);

                pgrid->info = info;
                pgrid->header = header;

                std::vector<int8_t> pgrid_data(info.width*info.height, OCC_GRID_UNKNOWN);

                //char* pgrid_data = new char[info.width*info.height](OCC_GRID_UNKNOWN);
                
                /* Insert custom code ------------------------------------------------------------ */

                double min_x = std::numeric_limits<double>::max();
                double max_x = -std::numeric_limits<double>::max();
                double min_y = std::numeric_limits<double>::max();
                double max_y = -std::numeric_limits<double>::max();

                int target_cell_count(0);

                for (unsigned int i = 0; i < info.height * info.width; i++) {
                    int x = i % info.width;
                    int y = i / info.width;

                    if (ogrid.data[i] == OCC_GRID_OCCUPIED) {
                        pgrid_data[i] = OCC_GRID_OCCUPIED;
                        for (int dy = std::max(0, y - radius_); dy <= std::min( height-1, y + radius_); ++dy) {
                            for (int dx = std::max(0, x - radius_); dx <= std::min(width-1, x + radius_); ++dx) {

                                int dxx = dx - x;
                                int dyy = dy - y;
                                int index = dy * info.width + dx;
                                int distance = std::sqrt(dxx*dxx + dyy*dyy);

                                if (distance <= radius_ && ogrid.data[index] == OCC_GRID_UNKNOWN) {
                                    int index = dy * info.width + dx;
                                    pgrid_data[index] = CANDIDATES;
                                    //double prob = OBJECT_DETECTED*std::exp(-0.01*distance);
                                    //pgrid_data[index] = static_cast<int8_t>(std::round(prob));


                                    // centeroid calculation
                                    double cell_x = ogrid.info.origin.position.x + dx * ogrid.info.resolution + ogrid.info.resolution / 2;
                                    double cell_y = ogrid.info.origin.position.y + dy * ogrid.info.resolution + ogrid.info.resolution / 2;
                                    // Update bounding box extremes
                                    min_x = std::min(min_x, cell_x);
                                    max_x = std::max(max_x, cell_x);
                                    min_y = std::min(min_y, cell_y);
                                    max_y = std::max(max_y, cell_y);
                                }
                            }
                        }
                    }
                }

                // Expand bounding box by half_size
                double half_size = 5.0 / 2.0;
                min_x -= half_size;
                max_x += half_size;
                min_y -= half_size;
                max_y += half_size;

                // Calculate centroid of the bounding box
                geometry_msgs::Point centroid;
                centroid.x = (min_x + max_x) / 2.0;
                centroid.y = (min_y + max_y) / 2.0;
                centroid.z = 0;

                geometry_msgs::PoseArray bounding_rectangle;
                bounding_rectangle.header.stamp = ros::Time::now();
                bounding_rectangle.header.frame_id = "map";
                geometry_msgs::Pose min_corner, max_corner;
                min_corner.position.x = min_x;
                min_corner.position.y = min_y;
                max_corner.position.x = max_x;
                max_corner.position.y = max_y;
                bounding_rectangle.poses.push_back(min_corner);
                bounding_rectangle.poses.push_back(max_corner);

                pub_rectangle.publish(bounding_rectangle);

                // Visualize bounding box
                visualizeBoundingBox(min_x, max_x, min_y, max_y);



                /* No Need to Change ------------------------------------------------------------ */
                pgrid->data = pgrid_data;
                pub_pgrid.publish(*pgrid);
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
                bounding_box.color.g = 0.0;
                bounding_box.color.b = 1.0;
                bounding_box.color.a = 1.0;

                // Define points of the bounding box
                geometry_msgs::Point p1, p2, p3, p4;

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
                pub_marker.publish(markers);
            }
        

            bool bound_check(int32_t location, int32_t width, int32_t height) {
                if (location < 0 || location > width * height - 1) {
                    //ROS_INFO("false %d", location);
                    return false;
                }
                return true;
            }

            void main_loop(const ros::TimerEvent &) const
            {
            }

         

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_pgrid;
        ros::Publisher pub_marker;
        ros::Subscriber sub_ogrid;
        ros::Publisher pub_rectangle;
        ros::Timer timer;
        int radius_;
        double alpha_;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nbv_node");
    int radius = 10; 
    double alpha = 0.03;
    NBVNode node(radius, alpha);
    ros::spin();
    return 0;
}

