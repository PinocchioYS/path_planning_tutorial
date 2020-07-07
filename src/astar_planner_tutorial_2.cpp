#include <ros/ros.h>

#include <astar_planner/in_3d_configuration_space/astar_planner_3d.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <path_planning_tutorial/utils.h>
#include <path_planning_tutorial/visualization.h>

class AstarPlanningServer {
protected:
    const double CONFIGURATION_XY_RESOLUTION = 0.1;         // [m]
    const int    NUMBER_OF_ROTATION_BINS     = 8;

    const double ROBOT_WIDTH  = 0.55;
    const double ROBOT_HEIGHT = 0.45;
    const quadmap::point2d robot_size = quadmap::point2d((float)ROBOT_WIDTH, (float)ROBOT_HEIGHT);

public:
    explicit AstarPlanningServer(const std::string& _map_filename) : planner(CONFIGURATION_XY_RESOLUTION, NUMBER_OF_ROTATION_BINS, robot_size)
    {
        map = utils::load_map_from_file(_map_filename);
        if(map != nullptr) {
            msg_map = visualization::map_to_occupancy_grid(map);
        }
        planner.set_environment(map);

        map_publisher               = nh.advertise<nav_msgs::OccupancyGrid>("/tutorial_2/map", 1);
        start_robot_model_publisher = nh.advertise<visualization_msgs::Marker>("/tutorial_2/start_robot_model", 1);
        goal_robot_model_publisher  = nh.advertise<visualization_msgs::Marker>("/tutorial_2/goal_robot_model", 1);
        path_publisher              = nh.advertise<visualization_msgs::Marker>("/tutorial_2/path", 1);
        motion_publisher            = nh.advertise<visualization_msgs::MarkerArray>("/tutorial_2/motion", 1);

        start_pose_subscriber = nh.subscribe("/initialpose", 1, &AstarPlanningServer::set_start_pose, this);
        goal_pose_subscriber  = nh.subscribe("/move_base_simple/goal", 1, &AstarPlanningServer::set_goal_pose, this);
    }
    ~AstarPlanningServer() {
        delete map;
    }

    void set_start_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg) {
        quadmap::point2d center(_msg->pose.pose.position.x, _msg->pose.pose.position.y);
        double yaw = utils::get_yaw(_msg->pose.pose.orientation);
        start_robot_model = OBB(center, robot_size, yaw);

        std_msgs::ColorRGBA color;
        color.r = 0.0;  color.g = 1.0;  color.b = 0.0;  color.a = 1.0;
        msg_start_robot_model = visualization::obb_to_marker(start_robot_model, color);

        ROS_INFO("Set start pose: (%f, %f, %f)", start_robot_model.center.x(), start_robot_model.center.y(), start_robot_model.rotation);

        // Planning
        if(planning()) {
            msg_path = visualization::path_to_marker(planner.get_path());
            msg_motion = visualization::path_to_markerarray(planner.get_path(), robot_size);
            ROS_INFO("Number of nodes in the path: %u", (unsigned int)planner.get_path().size());
        }
        else if(msg_start_robot_model != nullptr && msg_goal_robot_model != nullptr) {
            ROS_INFO("Fail to find the path");
        }
    }

    void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& _msg) {
        quadmap::point2d center(_msg->pose.position.x, _msg->pose.position.y);
        double yaw = utils::get_yaw(_msg->pose.orientation);
        goal_robot_model = OBB(center, robot_size, yaw);

        std_msgs::ColorRGBA color;
        color.r = 0.0;  color.g = 0.0;  color.b = 1.0;  color.a = 1.0;
        msg_goal_robot_model = visualization::obb_to_marker(goal_robot_model, color);

        ROS_INFO("Set goal pose: (%f, %f, %f)", goal_robot_model.center.x(), goal_robot_model.center.y(), goal_robot_model.rotation);

        // Planning
        if(planning()) {
            msg_path = visualization::path_to_marker(planner.get_path());
            msg_motion = visualization::path_to_markerarray(planner.get_path(), robot_size);
            ROS_INFO("Number of nodes in the path: %u", (unsigned int)planner.get_path().size());
        }
        else if(msg_start_robot_model != nullptr && msg_goal_robot_model != nullptr) {
            ROS_INFO("Fail to find the path");
        }
    }

    bool planning() {
        if(msg_start_robot_model == nullptr || msg_goal_robot_model == nullptr)
            return false;

        Configuration3D start_conf(start_robot_model.center.x(), start_robot_model.center.y(), start_robot_model.rotation);
        Configuration3D goal_conf(goal_robot_model.center.x(), goal_robot_model.center.y(), goal_robot_model.rotation);
        return planner.planning(start_conf, goal_conf);
    }

    void publish_messages() {
        if(msg_map != nullptr && map_publisher.getNumSubscribers() > 0)
            map_publisher.publish(msg_map);
        if(msg_start_robot_model != nullptr && start_robot_model_publisher.getNumSubscribers() > 0)
            start_robot_model_publisher.publish(msg_start_robot_model);
        if(msg_goal_robot_model != nullptr && goal_robot_model_publisher.getNumSubscribers() > 0)
            goal_robot_model_publisher.publish(msg_goal_robot_model);
        if(msg_path != nullptr && path_publisher.getNumSubscribers() > 0)
            path_publisher.publish(msg_path);
        if(msg_motion != nullptr && motion_publisher.getNumSubscribers() > 0)
            motion_publisher.publish(msg_motion);
    }

protected:
    ros::NodeHandle nh;
    ros::Publisher  map_publisher;
    ros::Publisher  start_robot_model_publisher;
    ros::Publisher  goal_robot_model_publisher;
    ros::Publisher  path_publisher;
    ros::Publisher  motion_publisher;
    ros::Subscriber start_pose_subscriber;
    ros::Subscriber goal_pose_subscriber;

    nav_msgs::OccupancyGridPtr          msg_map;
    visualization_msgs::MarkerPtr       msg_start_robot_model;
    visualization_msgs::MarkerPtr       msg_goal_robot_model;
    visualization_msgs::MarkerPtr       msg_path;
    visualization_msgs::MarkerArrayPtr  msg_motion;

    AstarPlanner3D planner;

    quadmap::QuadTree* map;

    // NOTE: Tutorial 2 uses an oriented bounding box type for robot model
    OBB start_robot_model;
    OBB goal_robot_model;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "astar_planner_tutorial_2");
    ros::NodeHandle nh;

    // Load parameters
    std::string MAP_FILENAME = "/home/user/catkin_ws/src/path_planning_tutorial/map/simple_scene.ot2";
    // std::string MAP_FILENAME = "/home/user/catkin_ws/src/path_planning_tutorial/map/quadtree.ot2";

    AstarPlanningServer astar_planning_server(MAP_FILENAME);

    try {
        while (nh.ok()) {
            astar_planning_server.publish_messages();

            ros::spinOnce();
        }
    }
    catch (std::runtime_error& e){
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    return 0;
}