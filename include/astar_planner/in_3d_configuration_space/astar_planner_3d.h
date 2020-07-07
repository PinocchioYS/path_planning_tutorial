#ifndef PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_3D_H
#define PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_3D_H

#include <astar_planner/in_3d_configuration_space/astar_planner_3d_node.h>
#include <collision_detector/collision_detector.h>
#include <unordered_map>
#include <queue>
#include <nav_msgs/Path.h>

// #define USE_26_MOVEMENT
// #define USE_NON_HOLONOMIC_CONSTRAINT

typedef std::unordered_map<Configuration3DKey, bool, Configuration3DKey::KeyHash> CollisionHashMap;

class AstarPlanner3D {
public:
    /*
     *
     */
    AstarPlanner3D(double _conf_xy_resolution, int _num_of_rotation_bins, const quadmap::point2d& _robot_size);

    /*
     *
     */
    ~AstarPlanner3D();

    /*
     *
     */
    bool planning(const Configuration3D& _start_conf, const Configuration3D& _goal_conf);

    /*
     *
     */
    void set_environment(const quadmap::QuadTree* _environment) { this->environment = _environment; }

    /*
     *
     */
    const std::vector<Configuration3D>& get_path() const { return path; }

#ifdef USE_NON_HOLONOMIC_CONSTRAINT
    // TODO: this rule-based can be an example of the non-holonomic constraints, but is not a general solution.
    inline bool is_constrained(const unsigned int _from_r_key, const unsigned int _movement_idx) {
        if(NUM_OF_ROT_BINS != 8 || NUMBER_OF_MOVEMENT != 26)
            return false;

        if (_from_r_key % 8 == 4) {
            if(_movement_idx == 19 || _movement_idx == 20 || _movement_idx == 21 || _movement_idx == 22 || _movement_idx == 23)
                return false;
        }
        else if (_from_r_key % 8 == 5) {
            if(_movement_idx == 14 || _movement_idx == 23 || _movement_idx == 24 || _movement_idx == 25 || _movement_idx == 22)
                return false;
        }
        else if (_from_r_key % 8 == 6) {
            if(_movement_idx == 6 || _movement_idx == 14 || _movement_idx == 15 || _movement_idx == 16 || _movement_idx == 25)
                return false;
        }
        else if (_from_r_key % 8 == 7) {
            if(_movement_idx == 3 || _movement_idx == 6 || _movement_idx == 7 || _movement_idx == 8 || _movement_idx == 16)
                return false;
        }
        else if (_from_r_key % 8 == 0) {
            if(_movement_idx == 0 || _movement_idx == 3 || _movement_idx == 4 || _movement_idx == 5 || _movement_idx == 8)
                return false;
        }
        else if (_from_r_key % 8 == 1) {
            if(_movement_idx == 0 || _movement_idx == 1 || _movement_idx == 2 || _movement_idx == 5 || _movement_idx == 9)
                return false;
        }
        else if (_from_r_key % 8 == 2) {
            if(_movement_idx == 2 || _movement_idx == 9 || _movement_idx == 10 || _movement_idx == 11 || _movement_idx == 17)
                return false;
        }
        else if (_from_r_key % 8 == 3) {
            if(_movement_idx == 11 || _movement_idx == 17 || _movement_idx == 18 || _movement_idx == 19 || _movement_idx == 20)
                return false;
        }

        return true;
    }
#endif

protected:
    const quadmap::QuadTree* environment;
    const quadmap::point2d   robot_size;
    const int                NUM_OF_ROT_BINS;

#if defined(USE_26_MOVEMENT)
    const unsigned int        NUMBER_OF_MOVEMENT = 26;
    const std::array<int, 26> MOVEMENT_X_DIR = {  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    const std::array<int, 26> MOVEMENT_Y_DIR = {  1,  1,  1,  0,  0,  0, -1, -1, -1,  1,  1,  1,  0,  0, -1, -1, -1,  1,  1,  1,  0,  0,  0, -1, -1, -1 };
    const std::array<int, 26> MOVEMENT_R_DIR = {  1,  0, -1,  1,  0, -1,  1,  0, -1,  1,  0, -1,  1, -1,  1,  0, -1,  1,  0, -1,  1,  0, -1,  1,  0, -1 };
    //                                            0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 }
#else
    const unsigned int       NUMBER_OF_MOVEMENT = 6;
    const std::array<int, 6> MOVEMENT_X_DIR = { 0,  0,  1, -1,  0,  0};
    const std::array<int, 6> MOVEMENT_Y_DIR = { 0,  0,  0,  0,  1, -1};
    const std::array<int, 6> MOVEMENT_R_DIR = { 1, -1,  0,  0,  0,  0};
#endif

    std::vector<Configuration3D> path;

    Configuration3DConverter* converter;

    inline double trans_squared_cost(const Configuration3DKey& _from, const Configuration3DKey& _to) {
        int delta[2] = {(int) _to[0] - _from[0], (int) _to[1] - _from[1]};
        return (double) (delta[0]*delta[0] + delta[1]*delta[1]);
    }
    inline double rot_squared_cost(const Configuration3DKey& _from, const Configuration3DKey& _to) {
        int delta = std::abs((int)_to[2] - _from[2]);
        delta = std::min(std::min(delta, std::abs(delta+NUM_OF_ROT_BINS)), std::abs(delta-NUM_OF_ROT_BINS));
        return (double) (delta*delta);
    }
    inline double cost(const Configuration3DKey& _from, const Configuration3DKey& _to) {
        return std::sqrt(trans_squared_cost(_from, _to) + rot_squared_cost(_from, _to));
    }

    /*
     *
     */
    class OpenSet : public std::priority_queue<AstarNode3D*, std::vector<AstarNode3D*>, AstarNode3D::compare> {
    public:
        AstarNode3D* find(const Configuration3DKey& _key) const {
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
    typedef std::unordered_map<Configuration3DKey, AstarNode3D*, Configuration3DKey::KeyHash> ClosedSet;

    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        The below implementations are related to collision detection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    CollisionHashMap collision_map;

    /*
     *
     */
    inline bool is_collision_at(const Configuration3DKey& _conf_key) {
        // Had checked the collision at the given key (configuration)
        if(collision_map.find(_conf_key) != collision_map.end())
            return collision_map.find(_conf_key)->second;

        const Configuration3D& conf = converter->keyToConf(_conf_key);

        bool collision_result = false;
        if(environment != nullptr && environment->size() > 0) {
            OBB robot_model(quadmap::point2d(conf.x(), conf.y()), robot_size, conf.r());
            collision_result = CollisionChecker::doCollide(environment, robot_model);
        }
        collision_map.insert(std::pair<Configuration3DKey, bool>(_conf_key, collision_result));

        return collision_result;
    }
};

#endif //PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_3D_H
