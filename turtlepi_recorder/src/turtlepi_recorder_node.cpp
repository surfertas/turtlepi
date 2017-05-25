#include <ros/ros.h>
#include <rosbag/recorder.h>
#include <regex>
#include <turtlepi_recorder/turtlepi_recorder.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_logger_test_node");
  ros::NodeHandle nh;
  std::string path_name, file_name, topics;
  rosbag::RecorderOptions options;

  if (!nh.getParam("turtlepi_recorder/topics", topics))
    std::cout << "No topics param found." << std::endl;

  if (!nh.getParam("turtlepi_recorder/path_to_bag", path_name))
    std::cout << "No path to bag specified." << std::endl;

  if (topics.empty()) {
    options.record_all = true;
  } else {
    std::regex re_ws("\\s+");
    std::sregex_token_iterator iter(topics.begin(), topics.end(), re_ws, -1);
    std::sregex_token_iterator end;
    std::vector<std::string> topics_record;
    for (; iter != end; ++iter)
      topics_record.push_back(*iter);

    options.topics = topics_record;
  }
  options.append_date = false;
  options.trigger = false;
  options.min_space = 0;
  options.verbose = true;
  options.prefix = path_name + "/" + "test";
  options.split = false;
  options.regex = false;

  std::cout << "about to initialize recorder." << std::endl;

  turtlepi_recorder::TurtlepiRecorder recorder(nh, options);

  //TODO: currently runs, but runs indefinitly need to implement controls around
  //the recorder. 

  ros::spin();
  return 0;
}
