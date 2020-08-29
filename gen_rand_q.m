function [rand_q] = gen_rand_q(robotmat)
    min_1boundary = robotmat.robot.lowerLim(1)+0.01; %gets the robot's joint limits, squeezes them in a bit to avoid hitting them 
    min_2boundary = robotmat.robot.lowerLim(2)+0.01;
    min_3boundary = robotmat.robot.lowerLim(3)+0.01;
    min_4boundary = robotmat.robot.lowerLim(4)+0.01;
    max_1boundary = robotmat.robot.upperLim(1)-0.01;
    max_2boundary = robotmat.robot.upperLim(2)-0.01;
    max_3boundary = robotmat.robot.upperLim(3)-0.01;
    max_4boundary = robotmat.robot.upperLim(4)-0.01;

    boundary1_dx = max_1boundary - min_1boundary; %Gets distance between joint limits
    boundary2_dx = max_2boundary - min_2boundary;
    boundary3_dx = max_3boundary - min_3boundary;
    boundary4_dx = max_4boundary - min_4boundary;   
    randarr = rand(4,1)'; %Generate 4x1 vector of random numbers
    rand_q = Node; %Create new node object
    rand_q.q0 = randarr(1)*boundary1_dx+min_1boundary; %Assign node joint configurations to random numbers within the robot's joint boundaries
    rand_q.q1 = randarr(2)*boundary2_dx+min_2boundary;
    rand_q.q2 = randarr(3)*boundary3_dx+min_3boundary;
    rand_q.q3 = randarr(4)*boundary4_dx+min_4boundary;
end
