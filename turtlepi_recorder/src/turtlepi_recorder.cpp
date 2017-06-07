#include <turtlepi_recorder/turtlepi_recorder.h>

namespace turtlepi_recorder
{
TurtlepiRecorder::TurtlepiRecorder(ros::NodeHandle& nh, rosbag::RecorderOptions options) : 
    nh_(nh), 
    recorder_(options),
    sub_amcl_pose_(nh_, "amcl_pose", 1),
    sub_laser_scan_(nh_, "laser_scan", 1),
    sync_(SyncPolicy(10), sub_amcl_pose_, sub_laser_scan_)
{
  registerService();
  registerSubscriber();

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

void TurtlepiRecorder::registerSubscriber()
{
  sync_.registerCallback(boost::bind(&TurtlepiRecorder::timeSyncCallback, _1, _2));
}

void TurtlepiRecorder::registerPublisher()
{
  pub_sync_amcl_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/synced/amcl_pose", 1000);
  pub_sync_scan_ =
    nh_.advertise<sensor_msgs::LaserScan>("/synced/laser_scan", 1000);
}

void TurtlepiRecorder::registerService()
{
  srv_recorder_control_ =
      nh_.advertiseService("turtlepi_recorder/recorder_control", &TurtlepiRecorder::recorderControlService, this);
}

void
TurtlepiRecorder::timeSyncCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg, 
                                   const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  pub_sync_amcl_.publish(pose_msg);
  pub_sync_scan_.publish(scan_msg); 
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
