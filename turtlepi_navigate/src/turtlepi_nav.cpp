#include <turtlepi_navigate/turtlepi_nav.h>

namespace turtlepi_navigate
{

TurtlepiNavigate::TurtlepiNavigate(ros::NodeHandle& nh,
                                   std::string action_server) :
    nh_(nh),
    ac_(action_server.c_str(), true)
{
    while (!ac_.waitForServer(ros::Duration(5.0)))
        std::cout << "Waiting: move_base action server" << std::endl;
}

TurtlepiNavigate::~TurtlepiNavigate()
{
}

void TurtlepiNavigate::registerPublisher()
{
}
                                       
void TurtlepiNavigate::sendTarget(turtlepi_navigate::GenerateTarget srv)
{
    ac_.sendGoal(srv.response.goal);
    std::cout << "sent goal" << std::endl;

    ac_.waitForResult();

    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        std::cout << "Target successfully reached." << std::endl;
    std::cout << "Failed" << std::endl;
}

}  // namespace turtlepi_navigate

