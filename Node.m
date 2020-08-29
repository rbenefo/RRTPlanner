classdef Node %Create new node object, with properties
    properties
        q0  
        q1
        q2
        q3
        parent %Parent property links nodes
    end
    methods
        function q = getq(obj) %make it easy to grab all the joint configs from a node at once
            q(:,1) = [obj.q0];
            q(:,2) = [obj.q1];
            q(:,3) = [obj.q2];
            q(:,4) = [obj.q3];
        end
    end
end
