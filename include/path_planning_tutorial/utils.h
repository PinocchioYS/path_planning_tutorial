#ifndef PATH_PLANNING_TUTORIAL_UTILS_H
#define PATH_PLANNING_TUTORIAL_UTILS_H

#include <quadmap/QuadTree.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

namespace utils {
    /*
     *
     */
    quadmap::QuadTree* load_map_from_file(const std::string& _filename) {
        std::string file_extension = _filename.substr(_filename.length() - 4, _filename.length());
        if(file_extension != ".ot2") {
            ROS_ERROR("Incorrect file extension");
            ROS_INFO("File extension of map file should be \".ot2\"");
            return nullptr;
        }

        return (quadmap::QuadTree*)(quadmap::AbstractQuadTree::read(_filename));
    }

    /*
     *
     */
    double get_yaw(const geometry_msgs::Quaternion& _q) {
        double m1 = 2.0 * (_q.w * _q.z + _q.x * _q.y);
        double m2 = 1.0 - 2.0 * (_q.y * _q.y + _q.z * _q.z);
        return std::atan2(m1, m2);
    }

    /*
     *
     */
    double get_yaw(const tf::Quaternion& _q) {
        double m1 = 2.0 * (_q.w() * _q.z() + _q.x() * _q.y());
        double m2 = 1.0 - 2.0 * (_q.y() * _q.y() + _q.z() * _q.z());
        return std::atan2(m1, m2);
    }
}



#endif //PATH_PLANNING_TUTORIAL_UTILS_H
