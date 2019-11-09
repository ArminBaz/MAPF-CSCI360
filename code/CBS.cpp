#include "CBS.h"
#include <iostream>
#include <queue>

vector<Path> CBS::find_solution() {
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list

    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);  // whenever generating a new node, we need to
                                 // put it into all_nodes
                                 // so that we can release the memory properly later in ~CBS()

    // find paths for the root node
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) {
        // TODO: if you change the input format of function find_path()
        //  you also need to change the following line to something like
        //  root->paths[i] = a_star.find_path(i, list<Constraint>());
        root->paths[i] = a_star.find_path(i, list<Constraint>(), 0);
        if (root->paths[i].empty()) {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }
    // compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // put the root node into open list
    open.push(root);

    int time = 0;
    // high-level implementation of CBS
    while (!open.empty()) {
        // get the node with the smallest cost
        CBSNode *P = open.top();

        // find collision (if there are any) from P
        collision find_col = find_collision(P->paths);

        // if there are no collisions, then we return the path
        if(find_col.collision_type == "none"){
            return(P->paths);
        }

        list<Constraint> new_constraints;   // list to store our two new constraints

        // if its a vertex collision then we generate two vertex constraints
        if(find_col.collision_type == "vertex"){
            // prohibit the first agent from being in the loc at the timestep
            Constraint v1 = vert_constraint(find_col.agent_a, find_col.loc_a, find_col.timestep);
            // prhibit the second agent from being in the loc at the timestep
            Constraint v2 = vert_constraint(find_col.agent_b, find_col.loc_b, find_col.timestep);
            new_constraints.push_back(v1);
            new_constraints.push_back(v2);
        }
        // if its an edge collision then we generate two edge constraints
        else if(find_col.collision_type == "edge"){
            // prohibit the first agent from moving from x to y at timestep t
            Constraint e1 = edge_constraint(find_col.agent_a, find_col.agent_b, find_col.loc_a, find_col.loc_b, find_col.timestep, 0);
            Constraint e2 = edge_constraint(find_col.agent_a, find_col.agent_b, find_col.loc_a, find_col.loc_b, find_col.timestep, 1);

            new_constraints.push_back(e1);
            new_constraints.push_back(e2);
        }

        // loop through the new_constraints and create new nodes
        for(auto constraint : new_constraints){
            // construnct a new Node for our conflict tree
            auto Q = new CBSNode(*P);

            // add the new constraint to the new nodes list of constraints
            Q->constraints.push_back(constraint);

            // get the agent for which the constraint takes place on
            int ai = get<0>(constraint);
            int x = get<1>(constraint);
            int y = get<2>(constraint);
            int t = get<3>(constraint);

            // generate a new path for the agent based on the new constraints
            Path ai_path = a_star.find_path(ai, Q->constraints, 0);

            // if the path isn't empty
            if(!ai_path.empty()){
                // replace the current path for the agent
                Q->paths[ai] = ai_path;
                
                // compute the cost of the root node
                for (const auto& path : Q->paths)
                    Q->cost += (int)path.size() - 1;
                
                // insert Q into open
                open.push(Q);
            }
            //return(Q->paths);
        }
        // pop off the current node and then we choose the next node with the lowest cost
        open.pop();

        time++;
    }
    return vector<Path>(); // return "No solution"
}

Constraint CBS::vert_constraint(int agent, int loc, int timestep){
    Constraint vert = make_tuple(agent,loc, -1, timestep);
    return vert;
}

Constraint CBS::edge_constraint(int agent_a, int agent_b, int x, int y, int timestep, int num){
    // edge constraint for agent a
    if(num == 0){
        Constraint edge_a = make_tuple(agent_a, x, y, timestep);
        return edge_a;
    }
    // edge constraint for agent b
    else {
        Constraint edge_b = make_tuple(agent_b, y, x , timestep);
        return edge_b;
    }
}

collision CBS::find_collision(vector<Path> paths){
    collision is_collision;

    int num_paths = paths.size();
    // find the longest path size
    int longest_path = 0;
    for(int i = 0; i < num_paths; i++){
        if(paths[i].size() > longest_path){
            longest_path = paths[i].size();
        }
    }

    // resize all of the paths to be the same length
    vector<Path> paths_same;
    paths_same.resize(num_paths);
    for(int i = 0; i < num_paths; i++){
        paths_same[i].resize(longest_path);
    }
    
    // loop through the paths and if we are past the last value, contine filling in
    // the resized paths holder with the values
    for(int i = 0; i < num_paths; i++){
        int last_value = paths[i].size()-1;
        for(int j = 0; j < longest_path; j++){
            if(j >= last_value){
               paths_same[i][j] = paths[i][last_value]; 
            }
            else{
                paths_same[i][j] = paths[i][j];
            }
        }
    }

    // Search for vertex collisions
    for(int agent = 0; agent < num_paths-1; agent++){
        for(int next_agent = agent+1; next_agent < num_paths; next_agent++){
            for(int time = 0; time < longest_path; time++){
            // VERTEX COLLISION: they are the same at the same time
                if(paths_same[agent][time] == paths_same[next_agent][time]){
                    collision vert_col;
                    int loc = paths_same[agent][time];
                    vert_col.collision_type = "vertex";
                    vert_col.agent_a = agent;
                    vert_col.agent_b = next_agent;
                    vert_col.loc_a = loc;
                    vert_col.loc_b = loc;
                    vert_col.timestep = time;
                    return vert_col;
                }
            }
        }
    }
    /*
        where agent a moves from cell x to cell y and
        agent b moves from cell y to cell x at time step t
    */
   // Search for edge collisions
    for(int agent = 0; agent < num_paths-1; agent++){
        for(int next_agent = agent+1; next_agent < num_paths; next_agent++){
            for(int time = 0; time < longest_path - 1; time++){
            int next_time = time + 1;
            int posA_1 = paths_same[agent][time];
            int posA_2 = paths_same[agent][next_time];
            int posB_1 = paths_same[next_agent][time];
            int posB_2 = paths_same[next_agent][next_time];

                // EDGE COLLISION: if they move to opposite locations on the same timestep
                if(posA_1 == posB_2 && posA_2 == posB_1){
                    collision edge_col;
                    edge_col.collision_type = "edge";
                    edge_col.agent_a = agent;
                    edge_col.agent_b = next_agent;
                    edge_col.loc_a = posA_1;
                    edge_col.loc_b = posB_1;
                    edge_col.timestep = next_time;
                    return edge_col;
                }
            }
        }
    }

    // if the function makes it this far then there are no collisions
    is_collision.collision_type = "none";
    return is_collision;
}


CBS::~CBS() {
    // release the memory
    for (auto n : all_nodes)
        delete n;
}