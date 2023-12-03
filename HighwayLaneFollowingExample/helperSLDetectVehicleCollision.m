function collision = helperSLDetectVehicleCollision(targetPoses, vehicleProfiles)
% helperSLDetectVehicleCollision detects collision of vehicles in ego coordinates
%
% targetPoses:     Array of poses of target vehicles
% vehicleProfiles: Array of vehicle profiles, first vehicle is ego
% collision:       Returns true if collision detected
%
% This is a helper function for example purposes and may be removed or
% modified in the future.

% Copyright 2019 The MathWorks, Inc.

% Default no collision
collision = false;

% Ego polyshape
egoProfile = vehicleProfiles(1);
p1 = vehicle2polyshape(...
    [0 0],... % Position [x y] (m)
    0,...     % Yaw (deg)
    egoProfile.Width,...
    egoProfile.Length,...
    egoProfile.Length/2 + egoProfile.OriginOffset(1));

% Compare overlap of ego and target polyshapes
for n = 1:numel(targetPoses)

    targetProfile = vehicleProfiles(n+1);
    p2 = vehicle2polyshape(...
        targetPoses(n).Position,...
        targetPoses(n).Yaw,...
        targetProfile.Width,...
        targetProfile.Length,...
        targetProfile.Length/2 + targetProfile.OriginOffset(1));
    
    overlapRatio = area(intersect(p1,p2))/area(union(p1,p2));
    
    if overlapRatio > 0
        collision = true; % collision detected
        return;
    end
end

end

function pshape = vehicle2polyshape(position,yaw,width,length,rearOverhang)

x = position(1);
y = position(2);

ycoord = [y-width/2, y+width/2, y+width/2, y-width/2];
xcoord = [x-rearOverhang, x-rearOverhang, x+length-rearOverhang, x+length-rearOverhang];
pshape = rotate(polyshape(xcoord, ycoord), yaw, [x, y]);

end
