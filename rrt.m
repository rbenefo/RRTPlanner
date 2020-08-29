function [path] = rrt(map, start, goal)
% RRT Find the shortest path from start to goal.
%   PATH = rrt(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The
%   first row is start and the last row is goal. If no path is found, PATH
%   is a 0x6 matrix.
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration


%% Prep Code

path = []; %Initialize path variable
robotmat = load("robot.mat"); %load in robot data
iterations = 10000; %Set the number of RRT iterations (set to a high number)
startnodearray =init_node_array(start); %Transforms the start array into a node, uses it to initialize the start tree.
%The start and end arrays are the start and goal trees; they're essentially tree data structures 
endnodearray = init_node_array(goal); %Transforms the goal array into a node, uses it to initialize the goal tree
validitycheck = check_valid_startgoal(robotmat, start, goal); %Boolean to check if the start or end configs are outside the robot range 
step_dist = 0.1; %Sets the step distance (how much the RRT expands on each iteration)
whotarget = 1; %Boolean that switches every iteration, switching from the start tree being the target and the end tree being the target
if validitycheck == 1 %Only run the loop if the start and goal nodes are within the robot's range 
    for i=1:iterations
        if whotarget == 1 %Start tree expands to random point; end tree expands towards closest start tree node
            rand_q = gen_rand_q(robotmat); %Generates random configuration
            [q_a, ~] = get_closest_node(startnodearray, rand_q); %Finds closest node in start tree to random configuration
            new = expand(q_a, rand_q, step_dist); %Creates expansion node where closest node in start tree expands towards random configuration 
            
            if not(isRobotCollided(getq(new), map, robotmat)) %Checks expansion node for collision. 
                new = give_parent(q_a, new); %If no collision, then assign parent to the expansion node (add it to the tree)
                startnodearray = [startnodearray new]; %Append new expansion node to the list tracking the nodes in the start tree
                [new, q_trgt] = get_closest_node(new, endnodearray); %Get element in end tree that's closest to any element in the start tree 

                distance = get_node_distance(new, q_trgt); %Get the distance between the pair of closest nodes between the end tree and start tree
                if distance < step_dist %If the distance is closer than the step distance, then connect the trees
                    startpath = traceback(new); %Get the start path by tracing the start tree by following the "parent" links
                    endpath = traceback(q_trgt); %Get the end path by tracing the end tree by following the "parent" links
                    path = [flip(startpath, 1);endpath]; %Combine the start path and end path into one path
                    break
                end
            end
            [q_b,q_trgt] = get_closest_node(endnodearray, startnodearray); %Now, the end tree targets the start tree.
            %Get the closest pair of nodes between the end tree and start
            %tree.

            new = expand(q_b, q_trgt, step_dist); %Prepare the expansion node from the end tree to the selected closest node in the start tree

            if not(isRobotCollided(getq(new), map, robotmat)) %If no collision
                new = give_parent(q_b, new); %then give a parent to the expansion node (add it to the end tree)
                endnodearray = [endnodearray new]; %Append the new node to the end node array

                distance = get_node_distance(new, q_trgt); %Check to see if we're close enough to the start tree
                if distance < step_dist %If the two trees are closer than
                    %the step distance, connect the trees
                    endpath = traceback(new); %Get end path by tracing the end tree by following the parent links
                    startpath = traceback(q_trgt); %Same for start tree
                    path = [flip(startpath, 1);endpath]; %Combine the two paths into one path
                    break
                end 
            end
            whotarget = whotarget*-1; %Flip the targeting; now, the end tree expands to a random point, and the start tree targets the end tree
        else
            rand_q = gen_rand_q(robotmat);
            [q_a, ~] = get_closest_node(endnodearray, rand_q);
            new = expand(q_a, rand_q, step_dist);
            if not(isRobotCollided(getq(new), map, robotmat)) %expand randomly from end
               new = give_parent(q_a, new);
                %tree connects q_a and rand_q
               [new, q_trgt] = get_closest_node(new, startnodearray);

                distance = get_node_distance(new, q_trgt);
                if distance < step_dist
                    endpath = traceback(new);
                    startpath = traceback(q_trgt);
                    path = [flip(startpath,1);endpath];
                    break
                end    

                endnodearray = [endnodearray new];
            end

            [q_b,q_trgt] = get_closest_node(startnodearray, endnodearray);
            %Targeter
            new = expand(q_b, q_trgt, step_dist);
            if not(isRobotCollided(getq(new), map, robotmat)) %track target from start
%                 new = expand(q_b, q_trgt, step_dist, 1);
                new = give_parent(q_b, new);
                startnodearray = [startnodearray new];
                distance = get_node_distance(new, q_trgt);
                if distance < step_dist
                    startpath = traceback(new);
                    endpath = traceback(q_trgt);
                    path = [flip(startpath,1);endpath];
                    break
                end    
            end
            whotarget = whotarget*-1;
        end
    
    end

joint_5_6_path = [linspace(start(5), goal(5),size(path,1))', linspace(start(6), goal(6),size(path,1))']; %Add in joint 5 and 6;
%they are a linear vector between their starting and ending configurations,
%spaced out by the length of the path
path = [path joint_5_6_path]; %Combine the path of joints 1-4 with the path of joints 5,6
else %if validity check returns false, don't run the for loop and return an error message
    fprintf("Either the start or goal positions were outside the robot's range.")
end
if isempty(path) == 1 %If the simulation does not converge (either it takes too long, or the validity check returned false)
    path = double.empty(0, 6); %return empty 0x6 matrix
    fprintf("\nRRT did not converge.\n")   
end

end
