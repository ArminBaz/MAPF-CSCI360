# Multi Agent Path Finding

#### Preparation:
Before running the code please make sure that you have *Python 3* with **matplotlib** as well as **numpy** packages installed.
<br/><br/>

#### Compiling the code:
To compile the code first navigate into either the PrioritizedPlanner or ConflictBasedSearch directories
```linux
cmake .
make
```
To execute the code after making the executable
```linux
PrioritizedPlanner exp0.txt exp0_paths.txt
```
Here, file exp0.txt is the input file that contains the information of the map and the
start and goal locations of the agents. File exp0 paths.txt is the output file that
contains the paths.

*Note*: You can change the input file name to any valid input file and the output file can be whatever you desire as long as it ends in .txt


For example, if I wanted ro run it with one of the paths in the test directory:
```linux
ConflictBasedSearch ../test/test_1.txt test_1_CBS_paths.txt
```

&nbsp;
