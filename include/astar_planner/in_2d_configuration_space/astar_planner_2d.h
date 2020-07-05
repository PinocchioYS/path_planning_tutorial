#ifndef PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_2D_H
#define PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_2D_H

#include <astar_planner/in_2d_configuration_space/astar_planner_2d_node.h>
#include <collision_detector/collision_detector.h>
#include <unordered_map>
#include <queue>
#include <nav_msgs/Path.h>

// #define USE_8_MOVEMENT

typedef std::unordered_map<Configuration2DKey, bool, Configuration2DKey::KeyHash> CollisionHashMap;

class AstarPlanner2D {
public:
    /*
     *
     */
    AstarPlanner2D(double _conf_xy_resolution, double ROBOT_RADIUS);

    /*
     *
     */
    ~AstarPlanner2D();

    /*
     *
     */
    bool planning(const Configuration2D& _start_conf, const Configuration2D& _goal_conf);

    /*
     *
     */
    void set_environment(const quadmap::QuadTree* _environment) { this->environment = _environment; }

    /*
     *
     */
    const std::vector<Configuration2D>& get_path() const { return path; }

protected:
    const quadmap::QuadTree* environment;
    const double             robot_size;

#ifdef USE_8_MOVEMENT
    const unsigned int       NUMBER_OF_MOVEMENT = 8;
    const std::array<int, 8> MOVEMENT_X_DIR = { 1, 1, 0, -1, -1, -1,  0,  1};
    const std::array<int, 8> MOVEMENT_Y_DIR = { 0, 1, 1,  1,  0, -1, -1, -1};
#else
    const unsigned int       NUMBER_OF_MOVEMENT = 4;
    const std::array<int, 4> MOVEMENT_X_DIR = { 1,  0, -1,  0 };
    const std::array<int, 4> MOVEMENT_Y_DIR = { 0,  1,  0, -1 };
#endif

    std::vector<Configuration2D> path;

    Configuration2DConverter* converter;

    /*
     *
     */
    static inline double cost(const Configuration2DKey& _from, const Configuration2DKey& _to) {
        int delta[2] = {(int) _to[0] - _from[0], (int) _to[1] - _from[1] };
        return std::sqrt((double) (delta[0] * delta[0] + delta[1] * delta[1]));
    }

    /*
     *
     */
    class OpenSet : public std::priority_queue<AstarNode2D*, std::vector<AstarNode2D*>, AstarNode2D::compare> {
    public:
        AstarNode2D* find(const Configuration2DKey& _key) const {
            for(const auto& node : this->c) {
                if (node->key == _key)
                    return node;
            }
            return nullptr;
        }
    };

    /*
     *
     */
    typedef std::unordered_map<Configuration2DKey, AstarNode2D*, Configuration2DKey::KeyHash> ClosedSet;

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        The below members and functions are related to collision detection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    CollisionHashMap collision_map;

    /*
     *
     */
    inline bool is_collision_at(const Configuration2DKey& _conf_key) {
        // Had checked the collision at the given key (configuration)
        if(collision_map.find(_conf_key) != collision_map.end())
            return collision_map.find(_conf_key)->second;

        const Configuration2D& conf = converter->keyToConf(_conf_key);

        bool collision_result = false;
        if(environment != nullptr && environment->size() > 0) {
            Circle robot_model(quadmap::point2d(conf.x(), conf.y()), (float)robot_size);
            collision_result = CollisionChecker::doCollide(environment, robot_model);
        }
        collision_map.insert(std::pair<Configuration2DKey, bool>(_conf_key, collision_result));

        return collision_result;
    }
};

#endif //PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_2D_H
