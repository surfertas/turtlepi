#include <turtlepi_navigate/target_generator.h>

namespace turtlepi_navigate
{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

    TargetGenerator::TargetGenerator(ros::NodeHandle& nh) :
        nh_(nh)
    {
        if (costMapInit()) {
            std::cout << "Cost map properly initialized." << std::endl;
        } else {
            std::cout << "Issues initializing cost map." << std::endl;
        }
        setParams();
        registerService();
        registerPublisher();
        registerSubscriber();
    }

    TargetGenerator::~TargetGenerator()
    {
        srv_generate_target_.shutdown();
    }

    void TargetGenerator::registerSubscriber()
    {
        sub_turtlepi_location_ = 
            nh_.subscribe("/amcl_pose", 10, &TargetGenerator::currentPositionCB, this);
        std::cout << "Registered subscriber." << std::endl;
    }

    void TargetGenerator::registerPublisher()
    {
        pub_visualization_marker_ = 
            nh_.advertise<visualization_msgs::Marker>("/turtlepi_navigate/visualization_marker", 0);
    }

    void TargetGenerator::registerService()
    {
        srv_generate_target_ =
            nh_.advertiseService("/turtlepi_navigate/generate_nav_target", &TargetGenerator::generateTargetService, this);
            std::cout << "Registered generate target server." << std::endl;
    }

    void TargetGenerator::currentPositionCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& location)
    {
        current_position_ = *location;
    }

    void TargetGenerator::mapToWorld(uint32_t mx, uint32_t my, double& wx, double& wy)
    {
        wx = map_origin_x_ + (mx + 0.5) * map_resolution_;
        wy = map_origin_y_ + (my + 0.5) * map_resolution_;
    }

    bool TargetGenerator::generateTargetService(turtlepi_navigate::GenerateTarget::Request &req,
                                                turtlepi_navigate::GenerateTarget::Response &res)
    {
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> grid_x(0, map_size_x_);
        std::uniform_int_distribution<> grid_y(0, map_size_y_);

        double world_x, world_y;
        uint32_t idx; 
 
        do {
            uint32_t map_x = grid_x(gen);
            uint32_t map_y = grid_y(gen);

            mapToWorld(map_x, map_y, world_x, world_y);

            idx = map_x + map_y * map_size_x_;
            std::cout << "idx: " << idx << std::endl;
            std::cout << "map_x: " << map_x << std::endl;
            std::cout << "map_y:" << map_y << std::endl;
            std::cout << "map_size_x_:" << map_size_x_ << std::endl;
            std::cout << "current_x: " << current_position_.pose.pose.position.x << std::endl;
            std::cout << "current_y: " << current_position_.pose.pose.position.y << std::endl;
            std::cout << std::endl;
            ROS_INFO("map data[idx]: %d\n", map_data_[idx]);
        } while (map_data_[idx] != 0);

        double radians = theta_ * (PI_/180.0);
        
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);
        geometry_msgs::Quaternion q_msg;
        tf::quaternionTFToMsg(quaternion, q_msg);

        res.goal.target_pose.header.frame_id = "base_link";
        res.goal.target_pose.header.stamp = ros::Time::now();

        res.goal.target_pose.pose.position.x = world_x;
        res.goal.target_pose.pose.position.y = world_y;
        res.goal.target_pose.pose.orientation =  q_msg;
        res.success = true;

        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer_.lookupTransform("map", "base_link",
                                                         ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }


        geometry_msgs::PoseStamped pose_in;
        geometry_msgs::PoseStamped pose_out;

        pose_in.pose.position.x = world_x;
        pose_in.pose.position.y = world_y;
        pose_in.header.stamp = ros::Time(0);
        pose_in.header.frame_id = "base_link";

        tfBuffer_.transform(pose_in, pose_out, "map");
        // TODO: Need to figure correct transform. Should it be base_link to
        // map? or something else.
        // TODO: Need to create threshold (if target position is x distance away
        // from current position.
        targetMarker(pose_out.pose.position.x, pose_out.pose.position.y);
        return res.success;
    }

    void TargetGenerator::setParams()
    {
        // set target orientation of robot.
        theta_ = 90.0;
        PI_ = 3.14159265358;
    }

    bool TargetGenerator::costMapInit()
    {
        while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
            std::cout << "Waiting for static_map" << std::endl;
        }
    
        ros::ServiceClient map_service_client = nh_.serviceClient<nav_msgs::GetMap>("static_map");
        nav_msgs::GetMap srv_map;

        if (map_service_client.call(srv_map)) {
            map_origin_x_ = srv_map.response.map.info.origin.position.x;
            map_origin_y_ = srv_map.response.map.info.origin.position.y;
            map_resolution_ = srv_map.response.map.info.resolution;
            map_size_x_ = srv_map.response.map.info.width;
            map_size_y_ = srv_map.response.map.info.height;
            map_data_ = srv_map.response.map.data;
            std::cout << "Cost Map updated!" << std::endl;
            return true;
        }
        return false;
        
    }

    void TargetGenerator::targetMarker(uint32_t x, uint32_t y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "turtlepi_navigate";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (double)x;
        marker.pose.position.y = (double)y;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        pub_visualization_marker_.publish(marker);
    }

}  // namespace turtlepi_navigate

