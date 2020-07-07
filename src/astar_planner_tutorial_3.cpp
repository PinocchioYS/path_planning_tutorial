#include <ros/ros.h>

#include <astar_planner/in_3d_configuration_space/astar_planner_3d.h>

#include <tf/transform_listener.h>

#include <path_planning_tutorial/utils.h>
#include <path_planning_tutorial/visualization.h>

class AstarPlanningServer {
protected:
    const double CONFIGURATION_XY_RESOLUTION = 0.1;     // [m]
    const int    NUMBER_OF_ROTATION_BINS     = 8;

    const double ROBOT_WIDTH  = 0.55;
    const double ROBOT_HEIGHT = 0.45;
    const quadmap::point2d robot_size = quadmap::point2d((float)ROBOT_WIDTH, (float)ROBOT_HEIGHT);

public:
    explicit AstarPlanningServer() : planner(CONFIGURATION_XY_RESOLUTION, NUMBER_OF_ROTATION_BINS, robot_size)
    {
        path_publisher        = nh.advertise<visualization_msgs::Marker>("/tutorial_3/path", 1);
        motion_publisher      = nh.advertise<visualization_msgs::MarkerArray>("/tutorial_3/motion", 1);
        start_robot_model_publisher = nh.advertise<visualization_msgs::Marker>("/tutorial_3/start_robot_model", 1);
        goal_robot_model_publisher  = nh.advertise<visualization_msgs::Marker>("/tutorial_3/goal_robot_model", 1);
        next_pose_publisher         = nh.advertise<geometry_msgs::PoseStamped>("/tutorial_3/next_pose", 1);

        map_subscriber        = nh.subscribe("/environment", 1, &AstarPlanningServer::set_map, this);

        map = nullptr;
    }
    ~AstarPlanningServer() {
        delete map;
    }

    void set_map(const nav_msgs::OccupancyGrid::ConstPtr& _msg) {
        if(map != nullptr)
            return;

        // OccupancyGrid to map
        map = new quadmap::QuadTree(_msg->info.resolution);
        quadmap::point2d map_offset(_msg->info.origin.position.x + _msg->info.resolution, _msg->info.origin.position.y + _msg->info.resolution);
        quadmap::QuadTreeKey min_key = map->coordToKey(map_offset);

        for(int i = 0; i < _msg->info.width; i++) {
            for(int j = 0; j < _msg->info.height; j++) {
                int idx = j * (int)_msg->info.height + i;
                quadmap::QuadTreeKey key(min_key[0] + i, min_key[1] + j);
                int occupancy = _msg->data[idx];

                if(occupancy > 50)
                    map->updateNode(key, true);
                else if(occupancy < 50)
                    map->updateNode(key, false);
            }
        }

        planner.set_environment(map);

        ROS_INFO("Set an environment.");
    }

    void planning() {
        tf::StampedTransform start_pose;
        tf::StampedTransform goal_pose;
        try{
            ros::Time time = ros::Time(0);
            tf_listener.lookupTransform("/map", "/start", time, start_pose);
            tf_listener.lookupTransform("/map", "/goal", time, goal_pose);
        }
        catch (tf::TransformException& ex){
            return;
        }

        {
            quadmap::point2d center(start_pose.getOrigin().x(), start_pose.getOrigin().y());
            double yaw = utils::get_yaw(start_pose.getRotation());
            start_robot_model = OBB(center, robot_size, yaw);

            std_msgs::ColorRGBA color;
            color.r = 0.0;  color.g = 1.0;  color.b = 0.0;  color.a = 1.0;
            msg_start_robot_model = visualization::obb_to_marker(start_robot_model, color);

            ROS_INFO("Set start pose: (%f, %f, %f)", start_robot_model.center.x(), start_robot_model.center.y(), start_robot_model.rotation);
        }

        {
            quadmap::point2d center(goal_pose.getOrigin().x(), goal_pose.getOrigin().y());
            double yaw = utils::get_yaw(goal_pose.getRotation());
            goal_robot_model = OBB(center, robot_size, yaw);

            std_msgs::ColorRGBA color;
            color.r = 0.0;  color.g = 0.0;  color.b = 1.0;  color.a = 1.0;
            msg_goal_robot_model = visualization::obb_to_marker(goal_robot_model, color);

            ROS_INFO("Set goal pose: (%f, %f, %f)", goal_robot_model.center.x(), goal_robot_model.center.y(), goal_robot_model.rotation);
        }

        Configuration3D start_conf(start_robot_model.center.x(), start_robot_model.center.y(), start_robot_model.rotation);
        Configuration3D goal_conf(goal_robot_model.center.x(), goal_robot_model.center.y(), goal_robot_model.rotation);

        if(planner.planning(start_conf, goal_conf)) {
            msg_path = visualization::path_to_marker(planner.get_path());
            msg_motion = visualization::path_to_markerarray(planner.get_path(), robot_size);
            msg_next_pose = utils::next_pose_to_pose_stamped(planner.get_path());

//            ROS_INFO("Number of nodes in the path: %u", (unsigned int)planner.get_path().size());
            ROS_INFO("Next pose: (%f, %f, %f)", msg_next_pose->pose.position.x, msg_next_pose->pose.position.y, utils::get_yaw(msg_next_pose->pose.orientation));
        }
//        else if(msg_start_robot_model != nullptr && msg_goal_robot_model != nullptr) {
//            ROS_INFO("Fail to find the path");
//        }
    }

    void publish_messages() {
        if(msg_start_robot_model != nullptr && start_robot_model_publisher.getNumSubscribers() > 0)
            start_robot_model_publisher.publish(msg_start_robot_model);
        if(msg_goal_robot_model != nullptr && goal_robot_model_publisher.getNumSubscribers() > 0)
            goal_robot_model_publisher.publish(msg_goal_robot_model);
        if(msg_path != nullptr && path_publisher.getNumSubscribers() > 0)
            path_publisher.publish(msg_path);
        if(msg_motion != nullptr && motion_publisher.getNumSubscribers() > 0)
            motion_publisher.publish(msg_motion);
        if(msg_next_pose != nullptr && next_pose_publisher.getNumSubscribers() > 0)
            next_pose_publisher.publish(msg_next_pose);
    }

protected:
    ros::NodeHandle nh;
    ros::Publisher  start_robot_model_publisher;
    ros::Publisher  goal_robot_model_publisher;
    ros::Publisher  path_publisher;
    ros::Publisher  motion_publisher;
    ros::Publisher  next_pose_publisher;
    ros::Subscriber map_subscriber;
    tf::TransformListener tf_listener;

    ros::Subscriber start_pose_subscriber;
    ros::Subscriber goal_pose_subscriber;

    visualization_msgs::MarkerPtr       msg_start_robot_model;
    visualization_msgs::MarkerPtr       msg_goal_robot_model;
    visualization_msgs::MarkerPtr       msg_path;
    visualization_msgs::MarkerArrayPtr  msg_motion;
    geometry_msgs::PoseStampedPtr       msg_next_pose;

    AstarPlanner3D  planner;

    quadmap::QuadTree* map;

    OBB start_robot_model;
    OBB goal_robot_model;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "astar_planner_tutorial_3");
    ros::NodeHandle nh;

    AstarPlanningServer astar_planning_server;

    try {
        while (nh.ok()) {
            astar_planning_server.planning();

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