
#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>

int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file = argv[1];
    string output_file = argv[2];
    if (ins.load_instance(input_file)) {
        ins.print_instance();
    } else {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }

    AStarPlanner a_star(ins);
    vector<Path> paths(ins.num_of_agents);
    list<Constraint> constraints;
    
    // < ai, x, y, t >

    // Question 1 constraints
    /*
    Constraint Q1_1;    // prohibits agent 0 from being at its goal at timestep 4
    Q1_1 = make_tuple(0, 12, -1, 4);
    Constraint Q1_2;    // prohibits agent 1 from moving from start location to neighbor location from 0 to 1 from t0 to t1
    Q1_2 = make_tuple(1, 9, 10, 1);

    constraints.push_back(Q1_1);
    constraints.push_back(Q1_1);
    */

    // Question 2 constraints
    /*
    Constraint Q2_1;    // prohibits agent 0 from being at its goal location at timestep 10
    Q2_1 = make_tuple(0, 12, -1, 10);

    constraints.push_back(Q2_1);
    */
    
    // Question 3 constraints
    /*
    Constraint Q3_1;    // prohibits agent 1 from being at 11 at timestep 2
    Q3_1 = make_tuple(1, 11, -1, 2);
    Constraint Q3_2;    // prohibits agent 1 from being at 9 at timestep 2
    Q3_2 = make_tuple(1, 9, -1, 2);      
    Constraint Q3_3;    // prohibits agent 1 from staying still at timestep 2
    Q3_3 = make_tuple(1, 10, -1, 2);

    constraints.push_back(Q3_1);
    constraints.push_back(Q3_2);
    constraints.push_back(Q3_3);
    */

    // Question 3 constraints
    
    Constraint Q3_1;    // prohibits agent 1 from being at 11 at timestep 2
    Q3_1 = make_tuple(1, 11, -1, 2);
    Constraint Q3_2;    // prohibits agent 1 from being at 9 at timestep 2
    Q3_2 = make_tuple(1, 9, -1, 2);      
    Constraint Q3_3;    // prohibits agent 1 from staying still at timestep 2
    Q3_3 = make_tuple(1, 10, -1, 2);
    

    constraints.push_back(Q3_1);
    constraints.push_back(Q3_2);
    constraints.push_back(Q3_3);

    for (int i = 0; i < ins.num_of_agents; i++) {
        paths[i] = a_star.find_path(i, constraints);

        if (paths[i].empty()) {
            cout << "Fail to find any solutions for agent " << i << endl;
            return 0;
        }
    }

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;

    // save paths
    ofstream myfile (output_file.c_str(), ios_base::out);
    if (myfile.is_open()) {
        for (int i = 0; i < ins.num_of_agents; i++) {
            myfile << paths[i] << endl;
        }
        myfile.close();
    } else {
        cout << "Fail to save the paths to " << output_file << endl;
        exit(-1);
    }

    return 0;
}