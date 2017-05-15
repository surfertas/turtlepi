#include <ros/ros.h>
#include "turtlepi_navigate/target_generator.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "target_generator");
    ros::NodeHandle nh;
        
    turtlepi_navigate::TargetGenerator generator(nh);
    
    ros::spin();
    return 0;
}

    
