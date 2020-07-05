#include <astar_planner/in_2d_configuration_space/astar_planner_2d.h>
#include <ros/console.h>

AstarPlanner2D::AstarPlanner2D(double _conf_xy_resolution, double ROBOT_RADIUS)
: converter(new Configuration2DConverter(_conf_xy_resolution)), environment(nullptr), robot_size(ROBOT_RADIUS)
{
}

AstarPlanner2D::~AstarPlanner2D()
{
    delete converter;
}

bool AstarPlanner2D::planning(const Configuration2D& _start_conf, const Configuration2D& _goal_conf)
{
    // Check correctness of input
    Configuration2DKey startKey = converter->confToKey(_start_conf);
    Configuration2DKey goalKey = converter->confToKey(_goal_conf);
    if(is_collision_at(startKey)) {
        ROS_WARN("Collision at the start configuration.");
        return false;
    }
    if(is_collision_at(goalKey)) {
        ROS_WARN("Collision at the goal configuration.");
        return false;
    }
    if(startKey == goalKey) {
        ROS_WARN("No path because of start == goal");
        return true;
    }

    // Initialize
    AstarNode2D* current = nullptr;
    OpenSet      openSet;
    ClosedSet    closedSet;

    openSet.push(new AstarNode2D(startKey, 0, cost(startKey, goalKey), nullptr));
    closedSet.clear();

    // Path planning
    bool found = false;
    while(!openSet.empty()) {
        // Get current node
        current = openSet.top();

        // Found the goal
        if(current->key == goalKey){
            found = true;
            break;
        }

        // Save the visited node
        closedSet.insert(std::pair<Configuration2DKey, AstarNode2D*>(current->key, current));
        openSet.pop();

        // Check new candidates
        for(unsigned int i = 0; i < NUMBER_OF_MOVEMENT; ++i){
            Configuration2DKey newKey = current->key;
            newKey[0] += MOVEMENT_X_DIR[i];
            newKey[1] += MOVEMENT_Y_DIR[i];

            // newKey is obstacle or not the end of closedset(previous node)
            if(is_collision_at(newKey) || closedSet.find(newKey) != closedSet.end())
                continue;

            AstarNode2D* neighbor = openSet.find(newKey);

            // Compute G cost
            double g_score = current->g + cost(current->key, newKey);
            // Compute H cost
            double h_score = cost(newKey, goalKey);

            if(neighbor == nullptr) {  // openSet doesn`t include neighbor node
                neighbor = new AstarNode2D(newKey, g_score, h_score, current);
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
    if(found) {
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

