function [isCollided] = isRobotCollided(q, map, robot)

% ISROBOTCOLLIDED Detect if a configuration of the Lynx is in collision
%   with any obstacles on the map.
%
% INPUTS:
%   q   - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q = calculateFK_sol(q);

[row,~] = size(map.obstacles);

isCollided = 0;
for i = 1:row
    linePt1 = Q(1:length(q)-1,:);
    linePt2 = Q(2:length(q),:);
    box = map.obstacles(i,:);
    boxMod = box + [-20, -20 , -20, 20, 20, 20];  % conservative estimate of robotic arm geometry [mm]
    
    collision = detectCollision(linePt1, linePt2, boxMod);
    if (any(collision))
        isCollided = 1;      
        break
    end
        
end    
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
