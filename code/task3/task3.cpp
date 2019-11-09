#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "CBS.h"
#include <tuple>
#include <chrono>

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

    CBS cbs(ins);
    ::std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    vector<Path> paths = cbs.find_solution();
    ::std::chrono::steady_clock::duration elapsedTime = ::std::chrono::steady_clock::now() - startTime;
    double duration = ::std::chrono::duration_cast< ::std::chrono::duration< double > >(elapsedTime).count();
    if (paths.empty()) { // Fail to find solutions
        cout << "No solutions!" << endl;
        return 0;
    }

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;
    std::cout << "Milliseconds: " << duration * 1000 << std::endl;

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