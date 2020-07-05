#include <collision_detector/collision_detector.h>

bool CollisionChecker::doCollide(const quadmap::QuadTree* _environment, const Circle& _circle)
{
    if(_environment == nullptr || _environment->size() <= 0)
        return false;

    double root_size = _environment->getNodeSize(0);
    Box root_box(quadmap::point2d(0.0f, 0.0f), (float)root_size);
    return doCollideRecursive(_environment, _circle, _environment->getRoot(), root_box);
}

bool CollisionChecker::doCollideRecursive(const quadmap::QuadTree* _environment, const Circle& _circle, const quadmap::QuadTreeNode* _node, const Box& _node_box)
{
    for(unsigned int i = 0; i < 4; i++){
        if(!_environment->nodeChildExists(_node, i))
            continue;   // Unknown space

        const quadmap::QuadTreeNode* childNode = _environment->getNodeChild(_node, i);
        if(!_environment->isNodeOccupied(childNode))
            continue;   // Free space

        // Collision check
        Box child_node_box(_node_box.center, _node_box.size * 0.5f);
        float center_offset = _node_box.size * 0.25f;
        if(i & 1U) child_node_box.center.x() += center_offset;
        else       child_node_box.center.x() -= center_offset;
        if(i & 2U) child_node_box.center.y() += center_offset;
        else       child_node_box.center.y() -= center_offset;

        // Collision check between circle and bounding box
        if(doCollidePrimitives(_circle, child_node_box)) {
            // Collision at leaf node (child node)
            if(!_environment->nodeHasChildren(childNode))
                return true;
            // Recursive collision check using a child node
            if(doCollideRecursive(_environment, _circle, childNode, child_node_box)) {
                return true;
            }
        }
    }

    return false;
}

bool CollisionChecker::doCollidePrimitives(const Circle& _circle, const Box& _box)
{
    double box_half_size = _box.size * 0.5;
    double distX = fabs(_circle.center.x() - _box.center.x());
    double distY = fabs(_circle.center.y() - _box.center.y());

    if(distX > (box_half_size + _circle.radius))
        return false;
    if(distY > (box_half_size + _circle.radius))
        return false;

    if(distX <= box_half_size){
        return true;
    }
    if(distY <= box_half_size){
        return true;
    }

    const double r = SQUARED_ROOT_2 * box_half_size + _circle.radius;
    double cornerDist = distX*distX + distY*distY;

    return (cornerDist < (r * r));
}