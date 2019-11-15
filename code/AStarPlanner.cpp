#include "AStarPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <string>

using namespace std;

ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const {
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    return path;
}

// Note: I added an int to say whether we are using CBS or PP to make sure the correct cost and paths are outputed
Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints, int CBSorPP) {
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];
    bool fail_constraint;   // initialize a checker to see if a constraint is failed

    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;

    // check to see the maximum timestep of the constraint on the current agent
    int largest_timestep_CBS = 0;
    for(auto constraint : constraints){
        int ai = get<0>(constraint);
        int t = get<3>(constraint);

        if(agent_id == ai && t > largest_timestep_CBS){
            largest_timestep_CBS = t;
        }
    }

    int largest_timestep_PP = 0;
    for(auto constraint : constraints){
        int ai = get<0>(constraint);
        int x = get<1>(constraint);
        int y = get<2>(constraint);
        int t = get<3>(constraint);

        if(ai == agent_id  && t > largest_timestep_PP && (goal_location == x || goal_location == y)){
            largest_timestep_PP = t;
        }
    }

    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.

    unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes; // new nodes mapping for

    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node
    auto root = new AStarNode(start_location, 0, h, 0, nullptr); // root node starts at time step 0
    open.push(root);

    Path path; // vector <int>
    while (!open.empty()) {
        auto curr = open.top();
        open.pop();

        // goal test
        // if we are using Conflict Based Search we need to check the current agents largest timestep
        if(CBSorPP == 0){
            if (curr->location == goal_location && curr->timestep > largest_timestep_CBS){
                path = make_path(curr);
                break;
            }
            
        }
        // if we are using Prioritized Planning we need to check if there are constraints from an earlier agent past the current timestep
        else if(CBSorPP == 1){
            if(curr->location == goal_location && curr->timestep > largest_timestep_PP){
                path = make_path(curr);
                break;
            }
        }

        // generate child nodes
        for (auto next_location : ins.get_adjacent_locations(curr->location)) {

            //auto it = all_nodes.find(next_location); // original
            pair <int, int> curr_pair;   // create a pair variable for the current node and timestep pair
            curr_pair.first = next_location;     // first value of pair is the loction
            curr_pair.second = curr->timestep;   // second value of pair is the timestep
            auto it = all_nodes.find(curr_pair);     // modify it from task0 to lookup currPair as its key

            if (it == all_nodes.end()) {// the location has not been visited before
                int next_g = curr->g + 1;
                int next_h = ins.get_Manhattan_distance(next_location, goal_location);
                int next_t = curr->timestep + 1; // calculate timestep of child node
                auto next = new AStarNode(next_location, next_g, next_h, next_t, curr); // generate the next AStarNode
                
                /*
                 see if the next node satisfies the given constraints
                    we are only concerned with two types of constraints for this example (edge and vertex)
                */
               /*
                    If the constraint is a vertex constraint, then it is in the format < ai, x, −1, t >
                    which prohibits agent ai
                    from being at location x at timestep t. 
               */
               /*
                    If the constraint is an edge constraint, then it is in the format < ai, x, y, t >  
                    which prohibits agent ai from moving from location x to location y from timestep
                    t−1 to timestep t
               */

                fail_constraint = false;    //  reset the fail constraint

                // if our next time step is too large, we are in an infinite loop and should exit without finding a path
                if(next_t > 3000){
                    Path empty_path;
                    return empty_path;
                }

                for(auto constraint : constraints){
                    // get the values from the constraints
                    int ai = get<0>(constraint);
                    int x = get<1>(constraint);
                    int y = get<2>(constraint);
                    int t = get<3>(constraint);

                    // vertex constraint
                    if(y == -1){
                        // prohibits agent ai from being at location x at time t
                        if (agent_id == ai && next_location == x && next_t == t){
                            fail_constraint = true;
                        }

                        // if we have a negative timestep, then we know we have reached a higher priority agents goal
                        // node and should avoid it (only applicable to prioritized planner)
                        if(t < 0){
                            if(agent_id == ai && next_location == x){
                                fail_constraint = true;
                            }
                        }
                    }
                    // edge constraint
                    else{
                        // prohibits agent ai from moving from location x to location y
                        // from timestep t-1(current) to timestep t(next)
                        if(agent_id == ai && curr->location == x && next_location == y && next_t == t){
                            fail_constraint = true;
                        }

                        if(t <= 0){
                            if(agent_id == ai && curr->location == x && next_location == y){
                                fail_constraint = true;
                            }
                        }
                    }

                }
                
                // if constraints are met, push the AstarNode and add it to all_nodes
                if(fail_constraint == false){
                    open.push(next);
                    all_nodes[curr_pair] = next;
                }
            }
            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }
    }

    // release memory
    for (auto n : all_nodes){
        delete n.second;
    }
    
    return path;
}