/*
 * Author: Tasuku Miura
 */
#ifndef TURTLEPI_NAV_H
#define TURTLEPI_NAV_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <turtlepi_navigate/GenerateTarget.h>

namespace turtlepi_navigate
{
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TurtlepiNavigate
{
public:
  TurtlepiNavigate(ros::NodeHandle& nh, std::string action_server);
  virtual ~TurtlepiNavigate();

  void registerPublisher();
  void sendTarget(turtlepi_navigate::GenerateTarget srv);

private:
  ros::NodeHandle nh_;
  MoveBaseClient ac_;
};
}
#endif  // TURTLEPI_NAV_H
