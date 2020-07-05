#ifndef PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_2D_NODE_H
#define PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_2D_NODE_H

#include <astar_planner/in_2d_configuration_space/configuration_2d.h>

struct AstarNode2D {
    // Constructor
    AstarNode2D(const Configuration2DKey& _key, double _g, double _h, AstarNode2D* _parent) {
        key = _key;
        g = _g;
        h = _h;
        parent = _parent;
    }

    // Node information
    double              g;      // cost to come
    double              h;      // cost to go
    Configuration2DKey  key;    // indexing key
    AstarNode2D*        parent; // link to parent

    // Score
    inline double get_score() { return g + h; }

    // compare operator for priority queue
    struct compare {
        bool operator()(AstarNode2D* _a, AstarNode2D* _b) {
            return _a->get_score() > _b->get_score();
        };
    };
};

#endif //PATH_PLANNING_TUTORIAL_ASTAR_PLANNER_2D_NODE_H
