#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>
#include <map>  // adding for mapping constraints
#include <chrono>

using namespace std;

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
    list<Constraint> constraints;   // initialize the list of constraints
    int timestep;

    // assign priority ordering to agents
    // By default, we use the index ordering of the agents where
    // the first always has the highest priority.
    vector<int> priorities;
    for (int i = 0; i < ins.num_of_agents; i++) {
        priorities.push_back(i);
    }

    //auto start_time = std::chrono::high_resolution_clock::now();
    ::std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    // loop through all of the agents based on priority
    for(int i = 0; i < priorities.size()-1; i++){
        int agent = priorities[i];  // generate first agent from priority
        paths[agent] = a_star.find_path(agent, constraints, 1); // fill in the path of the higher priority agent

        //  loop through the current highest priority agent's path
        for(int j = 1; j < paths[agent].size(); j++){

            // loop through the rest of the agents
            for(int k = i+1; k < priorities.size(); k++){
                int next_agent = priorities[k]; // generate the next agent from priority

                // if we are at the last location
                if(j == paths[agent].size()-1){
                    constraints.push_back(make_tuple(next_agent, paths[agent][j], -1, -j));
                    //cout << "Negative Constraint: " << paths[agent][j] << " At time: " << j << endl;
                }
                else{
                    constraints.push_back(make_tuple(next_agent, paths[agent][j], -1, j));  // vertex constraint for next agent
                    //cout << "Vertex Constraint: " << paths[agent][j] << " At time: " << j << endl;
                    constraints.push_back(make_tuple(next_agent, paths[agent][j+1], paths[agent][j], j+1));   // edge constraint for next agent
                }
                //cout << "Constraint Made for agent: " << k << " At timestep: " << j << endl;
            }
        }
        if(paths[agent].empty()){
            cout << "Failed to find a path for agent: " << agent << endl;
            ::std::chrono::steady_clock::duration elapsedTime = ::std::chrono::steady_clock::now() - startTime;
            double duration = ::std::chrono::duration_cast< ::std::chrono::duration< double > >(elapsedTime).count();
            std::cout << "Milliseconds: " << duration * 1000 << std::endl;
            return 0;
        }

        // if we are at the second to last agent
        if(i == priorities.size()-2){
            int final_agent = priorities[i+1];
            paths[final_agent] = a_star.find_path(final_agent, constraints, 1);

            if(paths[final_agent].empty()){
                //auto end_time = std::chrono::high_resolution_clock::now();
                //auto time = end_time - start_time;
                cout << "Failed to find a path for agent: " << final_agent << endl;
                ::std::chrono::steady_clock::duration elapsedTime = ::std::chrono::steady_clock::now() - startTime;
                double duration = ::std::chrono::duration_cast< ::std::chrono::duration< double > >(elapsedTime).count();
                std::cout << "Milliseconds: " << duration * 1000 << std::endl;
                return 0;
            }
        }
    }
    ::std::chrono::steady_clock::duration elapsedTime = ::std::chrono::steady_clock::now() - startTime;
    double duration = ::std::chrono::duration_cast< ::std::chrono::duration< double > >(elapsedTime).count();
    //auto time = end_time - start_time;


    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;
    std::cout << "Milliseconds: " << duration * 1000 << std::endl;
    //cout << time/std::chrono::milliseconds(1) << " ms to run." << endl;

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