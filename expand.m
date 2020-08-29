function [expanded_node, expand_vec] = expand(node1, node2, step_dist)
    vec_between(1) = node2.q0-node1.q0; %Get vector between nodes
    vec_between(2) = node2.q1-node1.q1;
    vec_between(3) = node2.q2-node1.q2;
    vec_between(4) = node2.q3-node1.q3;
    mag = norm(vec_between); %Get magnitude of vector
    scale = step_dist/mag; %Calculate scaling factor to scale down difference vector to step_distance
    if scale >= 1 %prevents overshoot of target (if scale is greater than 1, just expand by the 
        %vector between
        expand_vec = vec_between;
    else %else, scale down the vector
        expand_vec = vec_between*scale;
    end
    expanded_node = Node; %create new node object
    expanded_node.q0 = node1.q0 + expand_vec(1); %assign new, expanded configuration
    expanded_node.q1 = node1.q1 + expand_vec(2);
    expanded_node.q2 = node1.q2 + expand_vec(3);
    expanded_node.q3 = node1.q3 + expand_vec(4);
end
