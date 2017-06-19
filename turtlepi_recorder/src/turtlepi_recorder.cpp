#include <turtlepi_recorder/turtlepi_recorder.h>


// TODO: Figure out how to time stamp cmd_vel
// TODO: Figure out why we ccnt include tf2

namespace turtlepi_recorder
{
TurtlepiRecorder::TurtlepiRecorder(ros::NodeHandle& nh, rosbag::RecorderOptions options) : 
    nh_(nh), 
    recorder_(options),
    sub_amcl_pose_(nh_, "amcl_pose", 1),
    sub_laser_scan_(nh_, "scan", 1),
//    sub_cmd_vel_(nh_, "navigation_velocity_smoother/raw_cmd_vel", 1),
    sub_odom_(nh_, "odom", 1),
//    sub_tf_(nh_, "tf", 1),
    sub_depth_image_(nh_, "camera/depth/image_raw", 1),
    sync_(SyncPolicy(10), sub_amcl_pose_, sub_laser_scan_, sub_odom_, sub_depth_image_)
{
  registerService();
  registerSubscriber();
  registerPublisher();

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
  sync_.registerCallback(boost::bind(&TurtlepiRecorder::timeSyncCallback, this, _1, _2, _3, _4));
}

void TurtlepiRecorder::registerPublisher()
{
  pub_sync_amcl_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/synced/amcl_pose", 1000);
  pub_sync_scan_ =
    nh_.advertise<sensor_msgs::LaserScan>("/synced/scan", 1000);
//  pub_sync_cmd_vel_ =
//    nh_.advertise<geometry_msgs::Twist>("/synced/cmd_vel", 1000);
  pub_sync_odom_ =
    nh_.advertise<nav_msgs::Odometry>("/synced/odom", 1000);
//  pub_sync_tf_ =
//    nh_.advertise<tf2_msgs::TFMessage>("/synced/tf", 1000);
  pub_sync_depth_image_ =
    nh_.advertise<sensor_msgs::Image>("/synced/depth_image", 1000);
}

void TurtlepiRecorder::registerService()
{
  srv_recorder_control_ =
      nh_.advertiseService("turtlepi_recorder/recorder_control", &TurtlepiRecorder::recorderControlService, this);
}

void
TurtlepiRecorder::timeSyncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, 
                                   const sensor_msgs::LaserScanConstPtr& scan_msg,
 //                                  const geometry_msgs::TwistConstPtr& cmd_msg)//,
                                   const nav_msgs::OdometryConstPtr& odom_msg,
 //                                  const tf2_msgs::TFMessageConstPtr& tf_msg,
                                   const sensor_msgs::ImageConstPtr& depth_msg)
{
  pub_sync_amcl_.publish(pose_msg);
  pub_sync_scan_.publish(scan_msg); 
  //pub_sync_cmd_vel_.publish(cmd_msg);
  pub_sync_odom_.publish(odom_msg);
//  pub_sync_tf_.publish(tf_msg);
  pub_sync_depth_image_.publish(depth_msg);
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
