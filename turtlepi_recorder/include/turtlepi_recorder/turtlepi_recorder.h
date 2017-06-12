#ifndef TURTLEPI_RECORDER_H
#define TURTLEPI_RECORDER_H

#include <ros/ros.h>
#include <rosbag/recorder.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace turtlepi_recorder
{
typedef message_filters::sync_policies::ApproximateTime<
  geometry_msgs::PoseWithCovarianceStamped, 
  sensor_msgs::LaserScan
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
  void timeSyncCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg, 
                        const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  bool recorderControlService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  ros::NodeHandle nh_;
  rosbag::Recorder recorder_;
  ros::Publisher pub_sync_amcl_;
  ros::Publisher pub_sync_scan_;
  ros::ServiceServer srv_recorder_control_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_amcl_pose_;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser_scan_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  bool recorder_on_;
};
}
#endif  // TURTLEPI_RECORDER_H
