/*
 * Author: Tasuku Miura
 */
#ifndef TURTLEPI_NAV_H
#define TURTLEPI_NAV_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/stream.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <turtlepi_interfaces/GenerateTarget.h>

namespace turtlepi_navigate
{
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TurtlepiNavigate
{
public:
  TurtlepiNavigate(ros::NodeHandle& nh, std::string action_server);
  virtual ~TurtlepiNavigate();

  /// Register publisher to publish episode results.
  void
  registerPublisher();

  /// Service to send target to robot.
  void
  sendTarget(turtlepi_interfaces::GenerateTarget srv);

private:
  ros::NodeHandle nh_;
  MoveBaseClient ac_;

  ros::Publisher pub_episode_result_;
};
}
#endif  // TURTLEPI_NAV_H
