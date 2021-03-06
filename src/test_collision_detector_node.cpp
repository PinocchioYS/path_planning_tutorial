#include <ros/ros.h>

#include <collision_detector/collision_detector.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <path_planning_tutorial/utils.h>
#include <path_planning_tutorial/visualization.h>

#define USE_OBB_MODEL

class CollisionDetectionServer {
protected:
#ifdef USE_OBB_MODEL
    const double ROBOT_WIDTH  = 0.55;
    const double ROBOT_HEIGHT = 0.45;
    const quadmap::point2d robot_size = quadmap::point2d((float)ROBOT_WIDTH, (float)ROBOT_HEIGHT);
#else
    const double ROBOT_RADIUS = 0.3; // [m]
    const double robot_size = ROBOT_RADIUS * 2.0;
#endif

public:
    explicit CollisionDetectionServer(const std::string& _map_filename) {
        map = utils::load_map_from_file(_map_filename);
        if(map != nullptr) {
            msg_map = visualization::map_to_occupancy_grid(map);
        }

        map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/test_collision_detector/map", 1);
        robot_model_publisher = nh.advertise<visualization_msgs::Marker>("/test_collision_detector/robot_model", 1);

        robot_pose_subscriber = nh.subscribe("/initialpose", 1, &CollisionDetectionServer::detect_collision, this);
    }
    ~CollisionDetectionServer() {
        delete map;
    }

    void detect_collision(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg) {
        // Create a new robot model with the given pose
        quadmap::point2d center(_msg->pose.pose.position.x, _msg->pose.pose.position.y);
        std_msgs::ColorRGBA color;
        color.r = 0.0;  color.g = 1.0;  color.b = 0.0;  color.a = 1.0;

#ifdef USE_OBB_MODEL
        double yaw = utils::get_yaw(_msg->pose.pose.orientation);
        robot_model = OBB(center, robot_size, yaw);
        msg_robot_model = visualization::obb_to_marker(robot_model, color);
#else
        robot_model = Circle(center, (float)robot_size * 0.5f);
        msg_robot_model = visualization::circle_to_marker(robot_model, color);
#endif

        bool collision = CollisionChecker::doCollide(map, robot_model);
        if(collision) {
            msg_robot_model->color.r = 1.0;
            msg_robot_model->color.g = 0.0;
        }
        ROS_INFO("Result: %s", (collision ? "collision" : "no collision"));
    }

    void publish_messages() {
        if(msg_map != nullptr && map_publisher.getNumSubscribers() > 0)
            map_publisher.publish(msg_map);
        if(msg_robot_model != nullptr && robot_model_publisher.getNumSubscribers() > 0)
            robot_model_publisher.publish(msg_robot_model);
    }

protected:
    ros::NodeHandle nh;
    ros::Publisher  map_publisher;
    ros::Publisher  robot_model_publisher;
    ros::Subscriber robot_pose_subscriber;

    quadmap::QuadTree*          map;
    nav_msgs::OccupancyGridPtr  msg_map;

#ifdef USE_OBB_MODEL
    OBB     robot_model;
#else
    Circle  robot_model;
#endif

    visualization_msgs::MarkerPtr   msg_robot_model;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "test_collision_detector_node");
    ros::NodeHandle nh;

    // Load parameters
    std::string MAP_FILENAME = "/home/user/catkin_ws/src/path_planning_tutorial/map/simple_scene.ot2";
    // std::string MAP_FILENAME = "/home/user/catkin_ws/src/path_planning_tutorial/map/fr079.ot2";

    CollisionDetectionServer collision_detection_server(MAP_FILENAME);

    try {
        while (nh.ok()) {
            collision_detection_server.publish_messages();

            ros::spinOnce();
        }
    }
    catch (std::runtime_error& e){
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    return 0;
}