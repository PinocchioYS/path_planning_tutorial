#include <astar_planner/in_3d_configuration_space/astar_planner_3d.h>
#include <ros/console.h>

AstarPlanner3D::AstarPlanner3D(double _conf_xy_resolution, int _num_of_rotation_bins, const quadmap::point2d& _robot_size)
: converter(new Configuration3DConverter(_conf_xy_resolution, TWO_PI / _num_of_rotation_bins)), NUM_OF_ROT_BINS(_num_of_rotation_bins), environment(nullptr), robot_size(_robot_size)
{

}

AstarPlanner3D::~AstarPlanner3D()
{
    delete converter;
}

bool AstarPlanner3D::planning(const Configuration3D& _start_conf, const Configuration3D& _goal_conf)
{
    // Check correctness of input
    Configuration3DKey startKey = converter->confToKey(_start_conf);
    Configuration3DKey goalKey = converter->confToKey(_goal_conf);
    if(is_collision_at(startKey)) {
        ROS_ERROR("Collision at the start configuration.");
        return false;
    }
    if(is_collision_at(goalKey)) {
        ROS_ERROR("Collision at the goal configuration.");
        return false;
    }
    if(startKey == goalKey) {
        ROS_WARN("No path because of start == goal");
        return true;
    }

    // Initialize
    AstarNode3D* current = nullptr;
    OpenSet      openSet;
    ClosedSet    closedSet;

    openSet.push(new AstarNode3D(startKey, 0, cost(startKey, goalKey), nullptr));
    closedSet.clear();

    // Path planning
    bool found = false;
    while(!openSet.empty()) {
        // Get current node
        current = openSet.top();

        // Found the goal
        if(cost(current->key, goalKey) < 10e-10) {
            found = true;
            break;
        }

        // Save the visited node
        closedSet.insert(std::pair<Configuration3DKey, AstarNode3D*>(current->key, current));
        openSet.pop();

        // Check new candidates
        for(unsigned int i = 0; i < NUMBER_OF_MOVEMENT; ++i){
            Configuration3DKey newKey = current->key;
            newKey[0] += MOVEMENT_X_DIR[i];
            newKey[1] += MOVEMENT_Y_DIR[i];
            newKey[2] += MOVEMENT_R_DIR[i];

            // newKey is obstacle or not the end of closedset(previous node)
            if(is_collision_at(newKey) || closedSet.find(newKey) != closedSet.end())
                continue;

            // Non-holonomic constraint
#ifdef USE_NON_HOLONOMIC_CONSTRAINT
            if(is_constrained(current->key[2], i))
                continue;
#endif

            AstarNode3D* neighbor = openSet.find(newKey);

            // Update G
            double g_score = current->g + cost(current->key, newKey);
            // Act as "F_score = G_score + H_score"
            double h_score = cost(newKey, goalKey);

            if(neighbor == nullptr) {  // openSet doesn`t include neighbor node
                neighbor = new AstarNode3D(newKey, g_score, h_score, current);
                openSet.push(neighbor);
            }
            else if(g_score < neighbor->g) { // neighbor node already exists in openSet but New g_score is smaller than before
                neighbor->parent = current;
                neighbor->g = g_score;
                neighbor->h = h_score;
            }
        }
    }

    // Save the planned path
    path.clear();
    if(found){
        while(current != nullptr){
            path.push_back(converter->keyToConf(current->key));
            current = current->parent;
        }
    }

    // Release memories of all the nodes in open set
    while(!openSet.empty()){
        delete openSet.top();
        openSet.pop();
    }
    // Release memories of all the nodes in closed set
    for(auto& node : closedSet){
        delete node.second;
    }
    // Clear the collision hash information
    collision_map.clear();

    return found;
}