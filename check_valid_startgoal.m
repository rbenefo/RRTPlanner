function [valid] = check_valid_startgoal(robotmat, start, goal)
    valid = 1;
    if any(start > robotmat.robot.upperLim) %Check to make sure start config is not outside of robot joint range
        valid = 0;
    elseif any(start<robotmat.robot.lowerLim)
        valid = 0;
    elseif any(goal > robotmat.robot.upperLim) %check to make sure end config is not outside of robot range
        valid = 0;
    elseif any(goal<robotmat.robot.lowerLim)
        valid = 0;
    end
end
