function [nodearray] = init_node_array(qvec)
    nodearray = Node; %Create new node object
    nodearray(1).q0 = qvec(1); %assign configuration to node properties
    nodearray(1).q1 = qvec(2);
    nodearray(1).q2 = qvec(3);
    nodearray(1).q3 = qvec(4);
end
