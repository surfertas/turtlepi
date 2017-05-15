/*
 * Author: Tasuku Miura 
 */
#ifndef TARGET_GENERATOR_H
#define TARGET_GENERATOR_H

#include <random>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>    
#include <costmap_2d/cost_values.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <turtlepi_navigate/GenerateTarget.h>

namespace turtlepi_navigate
{

class TargetGenerator
{
public:
    TargetGenerator(ros::NodeHandle& nh);
    virtual ~TargetGenerator();

    void setParams();
    void registerSubscriber();
    void registerPublisher();
    void registerService();
    void costMapUpdateCB(const map_msgs::OccupancyGridUpdate::ConstPtr& grid_msg);
    void costMapInitCB(const nav_msgs::OccupancyGrid::ConstPtr& grid_msg);
    bool generateTargetService(turtlepi_navigate::GenerateTarget::Request &req, 
                               turtlepi_navigate::GenerateTarget::Response &res);

    void mapToWorld(uint32_t mx, uint32_t my, double& wx, double& wy);
  
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_cost_map_;
    ros::Subscriber sub_cost_map_update_;
    ros::ServiceServer srv_generate_target_;
    tf::TransformListener tf_listener_;
    costmap_2d::Costmap2D cost_map_;
    std::vector<int8_t> map_data_;

    uint32_t map_size_x_;
    uint32_t map_size_y_;
    float map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    double theta_;

    double PI_;
};
}
#endif // TARGET_GENERATOR_H

