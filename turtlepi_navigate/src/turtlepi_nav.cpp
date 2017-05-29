#include <turtlepi_navigate/turtlepi_nav.h>

namespace turtlepi_navigate
{
TurtlepiNavigate::TurtlepiNavigate(ros::NodeHandle &nh, std::string action_server)
  : nh_(nh), ac_(action_server.c_str(), true)
{
  while (!ac_.waitForServer(ros::Duration(5.0)))
    std::cout << "Waiting: move_base action server" << std::endl;

  registerPublisher();
}

TurtlepiNavigate::~TurtlepiNavigate()
{
}

void TurtlepiNavigate::registerPublisher()
{
  pub_episode_result_ = nh_.advertise<std_msgs::String>("/turtlepi_navigate/episode_result", 0);
}

void TurtlepiNavigate::sendTarget(turtlepi_navigate::GenerateTarget srv)
{
  std_srvs::SetBool toggle;
  toggle.request.data = true;
  ros::service::call("turtlepi_recorder/recorder_control", toggle);
  ac_.sendGoal(srv.response.goal);
  std::cout << "sent goal" << std::endl;

  while (!ac_.waitForResult())
  {
    // recordDataToBag();
  }

  actionlib::SimpleClientGoalState result = ac_.getState();
  std_msgs::String msg;
  msg.data = result.toString();
  pub_episode_result_.publish(msg);

  if (result == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::cout << "Target successfully reached." << std::endl;
  }
  else
  {
    std::cout << "Failed: " << result.toString() << std::endl;
  }
  toggle.request.data = false;
  ros::service::call("turtlepi_recorder/recorder_control", toggle);
}
}  // namespace turtlepi_navigate
