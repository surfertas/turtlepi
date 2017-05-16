#include <turtlepi_navigate/target_generator.h>

namespace turtlepi_navigate
{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

    TargetGenerator::TargetGenerator(ros::NodeHandle& nh) :
        nh_(nh)
    {
        setParams();
        registerService();
        registerPublisher();
        registerSubscriber();
    }

    TargetGenerator::~TargetGenerator()
    {
        srv_generate_target_.shutdown();
    }

    void TargetGenerator::setParams()
    {
        // set target orientation of robot.
        theta_ = 90.0;
        PI_ = 3.14159265358;
    }


    void TargetGenerator::registerSubscriber()
    {
        sub_cost_map_ =
            nh_.subscribe("/move_base/global_costmap/costmap", 1, &TargetGenerator::costMapInitCB, this);

        sub_cost_map_update_ =
            nh_.subscribe( "/move_base/global_costmap/costmap_updates", 10, &TargetGenerator::costMapUpdateCB, this );

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

    void TargetGenerator::mapToWorld(uint32_t mx, uint32_t my, double& wx, double& wy)
    {
        wx = map_origin_x_ + (mx + 0.5) * map_resolution_;
        wy = map_origin_y_ + (my + 0.5) * map_resolution_;
    }

    void TargetGenerator::costMapInitCB(const nav_msgs::OccupancyGrid::ConstPtr& grid_msg)
    {
        map_size_x_ = grid_msg->info.width;
        map_size_y_ = grid_msg->info.height;

        map_origin_x_ = grid_msg->info.origin.position.x;
        map_origin_y_ = grid_msg->info.origin.position.y;

        map_resolution_ = grid_msg->info.resolution;
        map_data_ = grid_msg->data;

        std::cout << "Cost Map created!" << std::endl;
    }

    void TargetGenerator::costMapUpdateCB(const map_msgs::OccupancyGridUpdate::ConstPtr& grid_msg)
    {
        map_size_x_ = grid_msg->width;
        map_size_y_ = grid_msg->height;

        map_origin_x_ = (double)grid_msg->x;
        map_origin_y_ = (double)grid_msg->y;

        map_data_ = grid_msg->data;
        std::cout << "Cost Map updated!" << std::endl;

    }

    void TargetGenerator::currentPositionCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& location)
    {
        current_position_ = *location;
    }

    bool TargetGenerator::generateTargetService(turtlepi_navigate::GenerateTarget::Request &req,
                                                 turtlepi_navigate::GenerateTarget::Response &res)
    {
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> grid_x(0, map_size_x_);
        std::uniform_int_distribution<> grid_y(0, map_size_y_);

        uint32_t map_x, map_y, idx;
        double world_x, world_y;

        do {
            map_x = grid_x(gen);
            map_y = grid_y(gen);
            std::cout << "COORD: " << map_x<< ":" << map_y << std::endl;

            mapToWorld(map_x, map_y, world_x, world_y);

            idx = map_x + map_y * map_size_x_;
            std::cout << "idx: " << idx;
            std::cout << "map_x:" << map_x;
            std::cout << "map_y:" << map_y;
            std::cout << "map_size_x_:" << map_size_x_;
            std::cout << "current_x: " << current_position_.pose.pose.position.x;
            std::cout << "current_y: " << current_position_.pose.pose.position.y;

            std::cout << std::endl;
            ROS_INFO("map data[idx]: %d\n", map_data_[idx]);

        //TODO:Check /amcl_pose to get location of turtlebot, use threshold to see if
        //ok to accept target.

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

        // TODO: move to separate function 
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

