function [distance] = get_node_distance(q, q_trgt)
    distance = sqrt((q.q0-q_trgt.q0)^2+(q.q1-q_trgt.q1)^2+(q.q2-q_trgt.q2)^2+...
        (q.q3-q_trgt.q3)^2);
end

