/*
 * Author: Tasuku Miura
 */
#ifndef TARGET_GENERATOR_H
#define TARGET_GENERATOR_H

#include <random>
#include <unordered_set>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <turtlepi_interfaces/GenerateTarget.h>

namespace turtlepi_navigate
{
class TargetGenerator
{
public:
  TargetGenerator(ros::NodeHandle& nh);
  virtual ~TargetGenerator();

  /// Register subscribers related to navigation.
  void
  registerSubscriber();

  /// Register publisher used for visualization.
  void
  registerPublisher();

  /// Register service that generates navigation target.
  void
  registerService();

  /// Call back that updates the current position.
  void
  currentPositionCB(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& location);

  /// Service that generates a random target.
  bool
  generateTargetService(
    turtlepi_interfaces::GenerateTarget::Request& req,
    turtlepi_interfaces::GenerateTarget::Response& res);

  /// Set target orientation of robot.
  void
  setParams();

  /// Initialize the cost map.
  bool
  costMapInit();

  /// Map to world conversion.
  void
  mapToWorld(
    const uint32_t map_x,
    const uint32_t map_y,
    double& world_x,
    double& world_y) const;

  /// World to map conversion.
  void
  worldToMap(
    uint32_t& map_x,
    uint32_t& map_y,
    const double world_x,
    const double world_y) const;

  /// Creates and publishes a marker.
  void
  targetMarker(const double x, const double y) const;

  /// WIP: TODO: not working, need to debug.
  void
  generateMapFill();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_visualization_marker_;
  ros::Subscriber sub_turtlepi_location_;
  ros::ServiceServer srv_generate_target_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener listener_{ tfBuffer_ };
  std::vector<int8_t> map_data_;
  std::unordered_set<uint32_t> free_space_;
  geometry_msgs::PoseWithCovarianceStamped current_position_;

  uint32_t map_size_x_;
  uint32_t map_size_y_;
  float map_resolution_;
  double map_origin_x_;
  double map_origin_y_;
  double theta_;
  bool init_;

  double DISTANCE_THRESHOLD_;
  double PI_;
};
}
#endif  // TARGET_GENERATOR_H
