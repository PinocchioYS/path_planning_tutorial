#ifndef HUBO_PLANNER_COLLISION_DETECTOR_H
#define HUBO_PLANNER_COLLISION_DETECTOR_H

// Environment
#include <collision_detector/primitive_box.h>

// Robot model
#include <collision_detector/primitive_circle.h>

#define SQUARED_ROOT_2 1.41421356237

class CollisionChecker{
public:
    static bool doCollide(const quadmap::QuadTree* _environment, const Circle& _circle);

protected:
    static bool doCollideRecursive(const quadmap::QuadTree* _environment, const Circle& _circle,
                                   const quadmap::QuadTreeNode* _node, const Box& _node_box);

    static bool doCollidePrimitives(const Circle& _circle, const Box& _box);
};

#endif //HUBO_PLANNER_COLLISION_DETECTOR_H
