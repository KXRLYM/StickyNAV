#include <chrono>
#include <thread>
#include "active_3d_planning_ros/planner/ros_planner.h"
#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_voxblox/initialization/voxblox_package.h"

int main(int argc, char **argv)
{
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // init ros
    ros::init(argc, argv, "test_stickynav_node");
    active_3d_planning::initialize::voxblox_package();

    // node handles
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // Setup
    active_3d_planning::ros::ModuleFactoryROS factory;
    active_3d_planning::Module::ParamMap param_map;
    active_3d_planning::ros::RosPlanner::setupFactoryAndParams(
        &factory, &param_map, nh_private);

    // Create and launch the planner
    active_3d_planning::ros::RosPlanner node(nh, nh_private, &factory,
                                             &param_map);
    node.planningLoop();
}
