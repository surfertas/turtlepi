#include <turtlepi_recorder/turtlepi_recorder.h>

namespace turtlepi_recorder
{
TurtlepiRecorder::TurtlepiRecorder(ros::NodeHandle& nh, rosbag::RecorderOptions options) : nh_(nh), recorder_(options)
{
  registerService();
  std::cout << "Initialized recorder." << std::endl;
  recorder_on_ = false;

  ros::Rate r(100);
  while (!recorder_on_ && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  recorder_.run();
}

TurtlepiRecorder::~TurtlepiRecorder()
{
}

void TurtlepiRecorder::registerService()
{
  srv_recorder_control_ =
      nh_.advertiseService("turtlepi_recorder/recorder_control", &TurtlepiRecorder::recorderControlService, this);
}

bool TurtlepiRecorder::recorderControlService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (!recorder_on_)
  {
    recorder_on_ = req.data;
    std::cout << "Starting recording." << std::endl;
  }
  else
  {
    ros::shutdown();
    std::cout << "Stopping recording." << std::endl;
  }
  res.success = true;

  return res.success;
}
}
