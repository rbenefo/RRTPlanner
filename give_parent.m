function [expanded_node] = give_parent(node1, node2)
    node2.parent = node1; %Assign parent to node 
    expanded_node = node2;
end
