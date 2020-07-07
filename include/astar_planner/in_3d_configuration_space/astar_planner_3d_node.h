#ifndef PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_3D_NODE_H
#define PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_3D_NODE_H

#include <astar_planner/in_3d_configuration_space/configuration_3d.h>

struct AstarNode3D {
    // Constructor
    AstarNode3D(const Configuration3DKey& _key, double _g, double _h, AstarNode3D* _parent) {
        key = _key;
        g = _g;
        h = _h;
        parent = _parent;
    }

    // Node information
    double              g;      // cost to come
    double              h;      // cost to go
    Configuration3DKey  key;    // indexing key
    AstarNode3D*        parent;

    // Score
    inline double get_score() { return g + h; }

    // compare operator for priority queue
    struct compare {
        bool operator()(AstarNode3D* _a, AstarNode3D* _b) {
            return _a->get_score() > _b->get_score();
        };
    };
};

#endif //PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_3D_NODE_H
