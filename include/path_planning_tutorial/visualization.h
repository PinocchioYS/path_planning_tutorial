#ifndef PATH_PLANNING_TUTORIAL_VISUALIZATION_H
#define PATH_PLANNING_TUTORIAL_VISUALIZATION_H

#include <quadmap/QuadTree.h>

#include <astar_planner/in_2d_configuration_space/configuration_2d.h>
#include <collision_detector/primitive_circle.h>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf2/LinearMath/Quaternion.h>

namespace visualization {
    // OPTIONS =========================================================================================================
    const int    MAX_NUM_OF_MARKERS = 10;
    const double VIS_HEIGHT_MARKER  = 0.01;

    /*
     *  This function generates a visualization message using the occupancy probabilities of all cells of QuadMap.
     *  RViz visualizes this message as a set of the gray-scale squares, black(occupied, 1.0) -- white(free, 0.0).
     */
    nav_msgs::OccupancyGridPtr map_to_occupancy_grid(quadmap::QuadTree* _map,
                                                     const std::string& _fixed_frame_id = "map",
                                                     const ros::Time& _time = ros::Time::now()) {
        // Convert a quad-tree to a uniform-grid
        _map->expand();

        // Compute a region of quadtree visualization
        double min_x, min_y, max_x, max_y;
        _map->getMetricMin(min_x, min_y);
        _map->getMetricMax(max_x, max_y);
        quadmap::QuadTreeKey min_key = _map->coordToKey(min_x, min_y);
        quadmap::QuadTreeKey max_key = _map->coordToKey(max_x, max_y);

        // Initialize a visualization message (nav_msgs::OccupancyGrid)
        nav_msgs::OccupancyGridPtr message(new nav_msgs::OccupancyGrid);
        message->header.frame_id = _fixed_frame_id;
        message->header.stamp = _time;

        message->info.resolution = (float)_map->getResolution();

        message->info.origin.position.x = _map->keyToCoord(min_key[0]) - (_map->getResolution() / 2.0);
        message->info.origin.position.y = _map->keyToCoord(min_key[1]) - (_map->getResolution() / 2.0);
        message->info.origin.position.z = 0.0;
        message->info.origin.orientation.x = 0.0;
        message->info.origin.orientation.y = 0.0;
        message->info.origin.orientation.z = 0.0;
        message->info.origin.orientation.w = 1.0;

        message->info.width = (unsigned int)(max_key[0] - min_key[0] + 1);
        message->info.height = (unsigned int)(max_key[1] - min_key[1] + 1);
        message->data.resize(message->info.width * message->info.height, -1);

        // Insert occupancy data to the message
        for(auto it = _map->begin_leafs(); it != _map->end_leafs(); it++){
            const quadmap::QuadTreeKey& node_key = it.getKey();
            int cell_x = node_key[0] - min_key[0];
            int cell_y = node_key[1] - min_key[1];
            int cell_idx = (int)(cell_y * message->info.width + cell_x);

            int occupancy_state = it->getOccupancy() > 0.5 ? 100 : 0;
            message->data[cell_idx] = occupancy_state;
        }

        // Convert the uniform-grid to the quad-tree
        _map->prune();

        return message;
    }

    /*
     * This function generates a visualization message for a circle.
     */
    visualization_msgs::MarkerPtr circle_to_marker(const Circle& _circle, std_msgs::ColorRGBA& _color,
                                                   const std::string& _FIXED_FRAME_ID = "map",
                                                   const ros::Time& _time = ros::Time::now(),
                                                   const int& _id = 0) {
        visualization_msgs::MarkerPtr message(new visualization_msgs::Marker);
        // Message header
        message->header.frame_id = _FIXED_FRAME_ID;
        message->header.stamp = _time;

        message->id = _id;
        message->type = visualization_msgs::Marker::CYLINDER;
        message->action = visualization_msgs::Marker::ADD;

        message->scale.x = message->scale.y = _circle.radius * 2.0;
        message->scale.z = VIS_HEIGHT_MARKER;

        message->color = _color;

        message->pose.position.x = _circle.center.x();
        message->pose.position.y = _circle.center.y();
        message->pose.position.z = VIS_HEIGHT_MARKER * 0.5;
        message->pose.orientation.x = 0.0;   message->pose.orientation.y = 0.0;   message->pose.orientation.z = 0.0;   message->pose.orientation.w = 1.0;

        return message;
    }

    /*
     *
     */
    visualization_msgs::MarkerArrayPtr path_to_markerarray(const std::vector<Configuration2D>& _path, const double& _robot_size,
                                                           const std::string& _FIXED_FRAME_ID = "map",
                                                           const ros::Time& _time = ros::Time::now()) {
        if(_path.empty())
            return nullptr;

        visualization_msgs::MarkerArrayPtr message(new visualization_msgs::MarkerArray);
        message->markers.resize(MAX_NUM_OF_MARKERS);

        int path_idx = 0;
        int path_idx_step = ((int)(_path.size() - 1) / MAX_NUM_OF_MARKERS) + 1;
        for(int i = 0; i < MAX_NUM_OF_MARKERS; i++) {
            if(path_idx < _path.size()) {
                double v = std::min(std::max(0.0, (double)path_idx / _path.size()), 1.0);
                std_msgs::ColorRGBA color;
                color.r = 0.0;  color.g = v;    color.b = (1.0-v);  color.a = 1.0;

                Circle robot_model(quadmap::point2d(_path[path_idx].x(), _path[path_idx].y()), (float)_robot_size);

                message->markers[i] = *(circle_to_marker(robot_model, color, _FIXED_FRAME_ID, _time, i));
            }
            else {
                message->markers[i].id = i;
                message->markers[i].action = visualization_msgs::Marker::DELETE;
            }

            path_idx += path_idx_step;
        }

        return message;
    }

    /*
     *
     */
    visualization_msgs::MarkerPtr path_to_marker(const std::vector<Configuration2D>& _path,
                                                 const std::string& _FIXED_FRAME_ID = "map",
                                                 const ros::Time& _time = ros::Time::now()) {
        if(_path.empty())
            return nullptr;

        visualization_msgs::MarkerPtr message(new visualization_msgs::Marker);

        // Message header
        message->header.frame_id = _FIXED_FRAME_ID;
        message->header.stamp = _time;

        message->id = 0;
        message->type = visualization_msgs::Marker::LINE_STRIP;
        message->action = visualization_msgs::Marker::ADD;

        message->scale.x = 0.01;

        message->color.r = 1.0;
        message->color.g = 0.0;
        message->color.b = 0.0;
        message->color.a = 1.0;

        message->pose.position.x = 0.0;      message->pose.position.y = 0.0;      message->pose.position.z = 0.0;
        message->pose.orientation.x = 0.0;   message->pose.orientation.y = 0.0;   message->pose.orientation.z = 0.0;   message->pose.orientation.w = 1.0;

        for(const auto& conf : _path) {
            geometry_msgs::Point point;
            point.x = conf.x();
            point.y = conf.y();
            point.z = VIS_HEIGHT_MARKER;
            message->points.push_back(point);
        }

        return message;
    }
}

#endif //PATH_PLANNING_TUTORIAL_VISUALIZATION_H
