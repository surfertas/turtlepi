#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "turtlepi_navigate/turtlepi_nav.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlepi_navigation");
  ros::NodeHandle nh;

  turtlepi_navigate::TurtlepiNavigate navigator(nh, "move_base");

  turtlepi_navigate::GenerateTarget srv;

  while (true)
  {
    if (ros::service::waitForService("/turtlepi_navigate/generate_nav_target", 2000))
    {
      auto call_success = ros::service::call("/turtlepi_navigate/generate_nav_target", srv);

      navigator.sendTarget(srv);
    }
    else
    {
      std::cout << "Timed out on waiting for service." << std::endl;
    }
  }
  ros::spin();
  return 0;
}
