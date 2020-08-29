function [path] = traceback(node)
    path = [node.q0 node.q1 node.q2 node.q3]; %Initialize path with the input nod
    error = 0;
    while error == 0 %Continue adding to the path until we find a node that doesn't have a parent (either the start or goal nodes) 
        try
            addpath = [node.parent.q0 node.parent.q1 node.parent.q2 node.parent.q3];%Add the node's parent's configs to the path
            path = [path;addpath];
            node = node.parent; %Trace back in the tree; set the current node to the previous node's parent
        catch
            error = 1; %If no parent exists (we've hit the start or goal nodes), flag an error and stop the traceback
        end
    end
    
end
