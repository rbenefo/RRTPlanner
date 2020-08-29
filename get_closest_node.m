function [closest_node1, closest_node2] = get_closest_node(nodearray1, nodearray2)
    check_node1 = getq(nodearray1); %get all the configuration variables from the node array
    check_node2 = getq(nodearray2);
    dist_matrix = pdist2(check_node1,check_node2).^2; %pdist2, from the stats 
    %toolbox, is an efficient algorithm that calculates the distance
    %between every node in one list to every node in another list. It's
    %much faster than the naive solution of a nested for loop to generate
    %each distance. pdist2 populates an mxn matrix with the distances,
    %where m is the length of check_node1 and n is the length of
    %check_node2
    minarray = min(dist_matrix(:)); %get the minimum value from the distance matrix
    [row,col] = find(dist_matrix==minarray); %get the indices for the minimum value
    closest_node1 = nodearray1(row); %Pull out the closest node pair
    closest_node2 = nodearray2(col);
end
