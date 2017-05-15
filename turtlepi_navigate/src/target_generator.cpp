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
        registerSubscriber();
    }

    TargetGenerator::~TargetGenerator()
    {
        srv_generate_target_.shutdown();
    }

    void TargetGenerator::registerSubscriber()
    {
        sub_cost_map_ =
            nh_.subscribe("/move_base/global_costmap/costmap", 1, &TargetGenerator::costMapInitCB, this);

        sub_cost_map_update_ =
            nh_.subscribe( "/move_base/global_costmap/costmap_updates", 10, &TargetGenerator::costMapUpdateCB, this );

        std::cout << "Registered subscriber." << std::endl;
    }

    void TargetGenerator::setParams()
    {
        // set target orientation of robot.
        theta_ = 90.0;
        PI_ = 3.141592653589793238463;
    }

    void TargetGenerator::registerPublisher()
    {
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

    bool TargetGenerator::generateTargetService(turtlepi_navigate::GenerateTarget::Request &req,
                                                 turtlepi_navigate::GenerateTarget::Response &res)
    {
        std::cout << "CALLED SERV" << std::endl;
        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> grid_x(0, map_size_x_);
        std::uniform_int_distribution<> grid_y(0, map_size_y_);


        uint32_t map_x, map_y, idx;
        double world_x, world_y;
        std::cout <<"Entering loop" << std::endl;

        do {
            map_x = grid_x(gen);
            map_y = grid_y(gen);
            std::cout << "COORD: " << map_x<< ":" << map_y << std::endl;

            mapToWorld(map_x, map_y, world_x, world_y);

            idx = map_x + map_y * map_size_x_;
            std::cout << "idx: " << idx << std::endl;
            std::cout << "idx: " << map_data_[idx] << std::endl;

        } while (map_data_[idx] != 0);
        // TODO: Why doesnt this print here?????? NEED TO FOLLOW UP
        std::cout <<"exit loop" << std::endl;

        std::cout << "before radians\n";
        double radians = theta_ * (PI_/180.0);
        std::cout << "radians: " << radians << std::endl;
        
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

        return res.success;
    }

}  // namespace turtlepi_navigate

