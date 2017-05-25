#ifndef TURTLEPI_RECORDER_H
#define TURTLEPI_RECORDER_H

#include <ros/ros.h>
#include <rosbag/recorder.h>
//#include <turtlepi_recorder/RecordTopics.h>

namespace turtlepi_recorder
{
class TurtlepiRecorder
{
public:
  TurtlepiRecorder(ros::NodeHandle& nh, rosbag::RecorderOptions options);
  virtual ~TurtlepiRecorder();

private:
  ros::NodeHandle nh_;
  rosbag::Recorder recorder_;
};
}
#endif //TURTLEPI_RECORDER_H
