# Multi Agent Path Finding
### Overview
This is a project that I had to do for my csci-360 class Introduction to Artificial Intelligence.
This project uses time-space A* to implement two different solvers: Prioritized Planning and ConflictBased Search (CBS).

<br/>

Just as a *note* when attempting to find the paths using the Prioritized Planner, sometimes an optimal path may not be found. This is because the goal state of a higher priority agent may block a lower priority agent from reaching it's own goal state. 

**Example:** <br/>
@ @ @ @ @ @ @ @ @ @ @<br/>
@ A0 A1 . . .  . . . . . G1 G0 @ <br/>
@ @ @ @ @ .  . @ @ @ @ @<br/>
@ @ @ @ @ @ @ @ @ @ @<br/>


As you can see if Agent A1 has the highest priority it will prevent agent A0 from ever reaching its goal state. However if A0 is the higher prioity agent it will force A1 to move out of the way into the little divit in the middle as the lower priority agent, A1, cannot be on a collision path with the higher priority agent, A0.

<br/><br/>

### Preparation:
Before running the code please make sure that you have *Python 3* with **matplotlib** as well as **numpy** packages installed.
<br/><br/>

### Compiling and Running the Code:
To compile the code first navigate into either the PrioritizedPlanner or ConflictBasedSearch directories.
* Prioritized Planner
   * ../code/PrioritizedPlanner/
* CBS
   * ../code/ConflictBasedSearch/

**To compile:**
```linux
cmake .
make
```
<br/>

**To execute the code after making the executable:**
```linux
PrioritizedPlanner exp0.txt exp0_paths.txt
```

Here, file exp0.txt is the input file that contains the information of the map and the
start and goal locations of the agents. File exp0 paths.txt is the output file that
contains the paths.

<br/>
*Note*: You can change the input file name to any valid input file and the output file 
can be whatever you desire as long as it ends in .txt.

<br/>

**To run code with a path in a different directory its the same process:**
```linux
ConflictBasedSearch ../test/test_1.txt test_1_CBS_paths.txt
```

<br/>


To use the python visualizer make sure that you pass in the paths for the same map.

**To run the python visualizer:**
```linux
python3 ../visualize.py exp0.txt exp0_paths.txt
```


<br/>

And **thats it**, you should be able to compile and run the programs, as well as use the visualization tool. :neckbeard:

If you would like to continue reading, I will go into more detail about the A* search algorithm as well as both the Conflict Based Search and Prioritized Planning algorithms.


<br/><br/>
## A* Search
