#ifndef TURTLEPI_RECORDER_H
#define TURTLEPI_RECORDER_H

#include <ros/ros.h>
#include <rosbag/recorder.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//TODO: time stamp cmd_vel message
//TODO: Figure out why cant we include tf2
namespace turtlepi_recorder
{
typedef message_filters::sync_policies::ApproximateTime<
  geometry_msgs::PoseWithCovarianceStamped, 
  sensor_msgs::LaserScan,
//  geometry_msgs::Twist
  nav_msgs::Odometry,
  //tf2_msgs::TFMessage,
  sensor_msgs::Image
> SyncPolicy;

class TurtlepiRecorder
{
public:
  TurtlepiRecorder(ros::NodeHandle& nh, rosbag::RecorderOptions options);
  virtual ~TurtlepiRecorder();

private:
  void registerSubscriber();
  void registerPublisher();
  void registerService();
  void timeSyncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, 
                        const sensor_msgs::LaserScanConstPtr& scan_msg,
                        //const geometry_msgs::TwistConstPtr& cmd_msg);//,
                        const nav_msgs::OdometryConstPtr& odom_msg,
  //                      const tf2_msgs::TFMessageConstPtr& tf_msg,
                        const sensor_msgs::ImageConstPtr& depth_msg);
  bool recorderControlService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  ros::NodeHandle nh_;
  rosbag::Recorder recorder_;
  ros::Publisher pub_sync_amcl_;
  ros::Publisher pub_sync_scan_;
//  ros::Publisher pub_sync_cmd_vel_;
  ros::Publisher pub_sync_odom_;
//  ros::Publisher pub_sync_tf_;
  ros::Publisher pub_sync_depth_image_;

  ros::ServiceServer srv_recorder_control_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_amcl_pose_;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser_scan_;
//  message_filters::Subscriber<geometry_msgs::Twist> sub_cmd_vel_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
//  message_filters::Subscriber<tf2_msgs::TFMessage> sub_tf_;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_image_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  bool recorder_on_;
};
}
#endif  // TURTLEPI_RECORDER_H
