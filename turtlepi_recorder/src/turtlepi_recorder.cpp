#include <turtlepi_recorder/turtlepi_recorder.h>

namespace turtlepi_recorder
{
  TurtlepiRecorder::TurtlepiRecorder(ros::NodeHandle& nh,
                                     rosbag::RecorderOptions options)
    : nh_(nh),
      recorder_(options)
  {
    std::cout << "Initialized recorder." << std::endl;
    recorder_.run();
  }
  
  TurtlepiRecorder::~TurtlepiRecorder()
  {
  }
} 
