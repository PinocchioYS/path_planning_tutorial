#ifndef PATH_PLANNING_TUTORIAL_COLLISION_DETECTOR_H
#define PATH_PLANNING_TUTORIAL_COLLISION_DETECTOR_H

// Environment
#include <collision_detector/primitive_box.h>

// Robot model
#include <collision_detector/primitive_circle.h>
#include <collision_detector/primitive_obb.h>

#define SQUARED_ROOT_2 1.41421356237

class CollisionChecker{
public:
    static bool doCollide(const quadmap::QuadTree* _environment, const Circle& _circle);
    static bool doCollide(const quadmap::QuadTree* _environment, const OBB& _obb);

protected:
    static bool doCollideRecursive(const quadmap::QuadTree* _environment, const Circle& _circle,
                                   const quadmap::QuadTreeNode* _node, const Box& _node_box);
    static bool doCollideRecursive(const quadmap::QuadTree* _environment, const OBB& _obb,
                                   const quadmap::QuadTreeNode* _node, const Box& _node_box);

    static bool doCollidePrimitives(const Circle& _circle, const Box& _box);
    static bool doCollidePrimitives(const OBB& _obb, const Box& _box);
    static bool doCollidePrimitives(const OBB& _obb, const AABB& _aabb);
};

#endif //PATH_PLANNING_TUTORIAL_COLLISION_DETECTOR_H
