# RRTPlanner

Function Description:

**rrt**

Our rrt algorithm is an implementation of RRT connect. We designed it based on the pseudocode provided by Dr. Hsieh in the slides; essentially, it expands on the classic RRT concept by having two trees, rather than one. A classic RRT algorithm grows a tree from the start node to the end node– in every iteration, it generates a random point in the configuration space, creates an expansion node inching the closest node in the tree to the random point towards the random point, checks the expansion link for a collision, and adds it to the tree. By repeatedly doing this, an RRT can effectively explore a configuration space and eventually, given enough iterations, will with high likelihood, find the goal. 
	Our RRT expands on this concept by instead of having one tree, grows two trees, one from the start node, and one from the goal node. During every iteration, the RRT has one cycle where the start node expands to a random point, and the end node targets the start node by expanding to the closest node in the start node. Then, the reverse happens: the end node expands to a random point, and the start node targets the end node. The idea is that with the double-tree method, the planner, on average, should need to explore less of the configuration space to reach the goal.
	Noting that RRT’s have faster runtimes as the dimensionality of the configuration space drops, and noting that joints 5 and 6 do not impact the position of the robot’s end effector, we have our RRT plan only on the first four joint variables. To make joint 5 and 6 move from the start configuration to the end configuration, after the RRT is done running, we append two linearly spaced vectors to the path the RRT’s generated, one for each joint. The starting value of each vector is the start configuration of its respective joint, and the ending value of each vector is the goal configuration of its respective joint. The length of each linearly spaced vector is simply the length of the original path array. After implementing this, we saw a dramatic decrease in the runtime of our RRT.
	The RRT has a number of helper functions and objects:

* Node (object)
* init_node_array (function)
* check_valid_startgoal (function)
* gen_rand_q (function)
* get_closest_node (function)
* expand (function)
* isRobotCollided (function)
* give_parent (function)
* get_node_distance (function)
* traceback (function)


**Node**

Node is an object that has 5 properties; 4 properties for the first 4 joints, and a parent property. By assigning node as an Object, we can easily create a list of linked entities by linking them via the “Parent” property. Node also has one built in method; getQ, which allows the user to easily grab all the node configuration variables into one array.

**init_node_array**

This function transforms the start and goal arrays into nodes. Since nodes can be stacked (they can be turned into node arrays), the start and goal nodes themselves are used to initialize the start and goal arrays. The start and goal arrays contain all the nodes in the start and goal trees; they’re essentially tree data structures.


**check_valid_startgoal**

This function checks to make sure that the start and goals configurations are within the robot’s joint limits. The function returns a boolean.

**gen_rand_q**

This function generates a random configuration of the first four joint variables within the robot’s joint limits.

**get_closest_node**

This function takes in two node arrays (where a node array can contain one or more nodes), and finds the node pair between the two arrays that are closest in distance in the configuration space. For example, if the configuration space was one dimensional, one node array was A = {1,  5, 10}, and the other was B = {3, 4, 15, 20}, the function would select the node with configuration 5 from array A, and the node with configuration 15 from array B. It calculates the distances between every node in the first input to every node in the second input via pdist2, part of the statistics package. We chose to use pdist2 over a nested for loop to calculate our distances to reduce runtime; we found that we were able to significantly speed up the algorithm by doing this.

**expand**

This function takes in a pair of nodes, and generates a new node that lies on a vector between the first and second input nodes, at a distance of a pre-specified step_distance or less from the first node. If the distance between the two input nodes is less than the step distance, the new node is simply placed on top of the second node; if not, the new node is placed at the step_distance away from the first node. Importantly, expand does not assign a parent to the new node; that is done in the give_parent function after the RRT has checked to see if the connection between the new node and its potential parent (the first node input) will result in a collision.


**isRobotCollided**

In order to detect collisions, we used the detectCollisions.m file that was provided to us to check the collision between the line connecting two specified points and the specified obstacle.  The FK solution code was used to determine the coordinates of all of the joint locations on the robot for a specified configuration.  By taking the first 4 joint locations as the first array and the last 4 joint locations as the second array, we can vectorize this function to take the 4 lines that make up the segments of the robot.  We employed an additional loop to account for the possibility of multiple obstacles (detectCollisions.m only takes one block argument).  Finally, in order to account for the thickness of the robot arm, an additional 20mm was added onto the obstacle geometry (~half the width of the robot arm) to serve as a crude approximation.  This should serve as a conservative approximation, since at the corner of the obstacle box, the robot would actually be able to contact the obstacle at 20mm [mm] whereas the approximation allows for it to only contact the obstacle at 20mm*sqrt(2) [mm]:
		

**give_parent**

This function, which only runs if isRobotCollided returns false, assigns the parent property of the second node input to be the first node input. This prepares the second node to be added to the node array.

**get_node_distance**

This function gets the euclidean distance between its two inputs (simply the sum of the square difference between their configuration values).

**traceback**

Once the start and goal trees have been connected, traceback runs through the input tree to populate an array of q’s (the path). To do this, it pulls the configuration values of the parent of each node in the input tree, stopping when it cannot find a parent (the only nodes in the tree that do not have parents are the start and goal nodes). This function is run on the start array and the goal array.
