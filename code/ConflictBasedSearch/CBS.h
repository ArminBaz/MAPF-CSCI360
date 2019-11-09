#pragma once
#include "AStarPlanner.h"

struct CBSNode {
    list<Constraint> constraints;
    vector<Path> paths;
    int cost;

    CBSNode(): cost(0) {}

    // this constructor helps to generate child nodes
    CBSNode(const CBSNode& parent):
            constraints(parent.constraints), paths(parent.paths), cost(0) {}
};

// This function is used by priority_queue to prioritize CBS nodes
struct CompareCBSNode {
    bool operator()(const CBSNode* n1, const CBSNode* n2) {
        return n1->cost > n2->cost; // prefer smaller cost
    }
};

// collision struct to return from find_collision
struct collision{
    int agent_a;    // first agent involved
    int loc_a;      // location of the first agent
    int agent_b;    // second agent involved
    int loc_b;      // location of the second agent
    int timestep;   // timestep of the collision
    string collision_type; // can be "vertex", "edge", or "none"
};

class CBS {
public:
    vector<Path> find_solution();
    explicit CBS(const MAPFInstance& ins): a_star(ins) {}
    ~CBS();
    Constraint vert_constraint(int agent, int loc, int timestep);
    Constraint edge_constraint(int agent_a, int agent_b, int x, int y, int timestep, int num);
    //collision find_collision(vector<Path> paths);
    collision find_collision(vector<Path> paths);

private:
    AStarPlanner a_star;
    /*
        function to generate a new CBSNode every time there is a collision
        inputs: agent_0 -> first agents id
                agent_1 -> second agents id
                VorE -> vertex or edge collision vertex = 0, edge = 1
                num -> first or second constraint to generate; 0 = first form, 1 = second form
        outputs: a new CBSNode
    */

    // all_nodes stores the pointers to CBS nodes
    // so that we can release the memory properly when
    // calling the destructor ~CBS()
    list<CBSNode*> all_nodes;

};
