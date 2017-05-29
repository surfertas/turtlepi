#ifndef TURTLEPI_RECORDER_H
#define TURTLEPI_RECORDER_H

#include <ros/ros.h>
#include <rosbag/recorder.h>
//#include <turtlepi_recorder/RecordTopics.h>
#include <std_srvs/SetBool.h>

namespace turtlepi_recorder
{
class TurtlepiRecorder
{
public:
  TurtlepiRecorder(ros::NodeHandle& nh, rosbag::RecorderOptions options);
  virtual ~TurtlepiRecorder();

private:
  void registerService();
  bool recorderControlService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  ros::NodeHandle nh_;
  rosbag::Recorder recorder_;
  ros::ServiceServer srv_recorder_control_;

  bool recorder_on_;
};
}
#endif  // TURTLEPI_RECORDER_H
