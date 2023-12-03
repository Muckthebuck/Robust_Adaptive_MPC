function [scenario,assessment,laneInfo] = helperGetLaneFollowingScenario(roadType)
%helperGetLaneFollowingScenario gets the driving scenario synchronized with
%HighwayLaneFollowingTestBench.slx
%
%   'roadType'      Type of Sim 3D road scene. Valid values:
%                   "Straight road" synchronize with "Straight road"
%                   Sim 3D scene
%                   "Curved road segment" synchronize with segment of
%                   "Curved road" Sim 3D scene
%
%   'scenario'      Driving scenario with vehicles matching with
%                   HighwayLaneFollowingTestBench.slx
%
%   'assessment'    Assessment structure used by
%                   HighwayLaneFollowingTestBench.slx
%
%   'laneInfo'      Lane Information of the scene. It is a structure that
%                   contains lane width, lane direction and lane centers
%                   which are helpful to specify vehicle trajectories.
%

%   This is a helper function for example purposes and
%   may be removed or modified in the future.

%   Copyright 2019 The MathWorks, Inc.

%% Construct driving scenario
scenario = drivingScenario;
scenario.SampleTime = 0.1;
scenario.StopTime = 30;
%% Add road to scenario
switch roadType
    case "Straight road"
        laneInfo = addStraightRoad(scenario);
        defaultVehiclePosition = [-122 50 0];
    case "Curved road segment"
        laneInfo = addCurvedRoadSegment(scenario);
        defaultVehiclePosition = [0 0 0];
    otherwise
        error("Unsupported road type. Road type must be 'Straight road' or 'Curved road segment'");
end

%% Add vehicles to scenario
% - Vehicle profiles must match vehicles in Simulink model
% - First vehicle added is ego (ActorID = 1)
addVehicle(scenario, "Sedan",                 defaultVehiclePosition);
addVehicle(scenario, "Sedan",                 defaultVehiclePosition);
addVehicle(scenario, "Muscle car",            defaultVehiclePosition);
addVehicle(scenario, "Hatchback",             defaultVehiclePosition);
addVehicle(scenario, "Small pickup truck",    defaultVehiclePosition);
addVehicle(scenario, "Sport utility vehicle", defaultVehiclePosition);

%% Default assessments
assessment.TimeGap = 0.8;
assessment.LongitudinalAccelMin = -3;
assessment.LongitudinalAccelMax = 3;
assessment.LateralDeviationMax = 0.45;

end

function laneInfo = addStraightRoad(scenario)
% Add road representing "Straight road" Sim 3D scene
rc = load('laneFollowingRoadCenters','roadCentersStraightRoad');
roadCenters = rc.roadCentersStraightRoad;

marking = [...
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Unmarked')];
laneSpecification = lanespec([3 3], 'Width', [1.15 3.85 4.05 4.05 3.85 1.15], 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

[n,~] = size(roadCenters);

laneCenters = {zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3)};
numLanes = numel(laneSpecification.Width);
offset = estimateRoadCenterOffset(laneSpecification);
minLaneWidth = 2;
laneInfo = struct();
% Shifting of road centers to get lane centers
for j = 1:numLanes
    for i = 1:n
        laneCenters{j}(i,1) = roadCenters(i,1) + 0;
        laneCenters{j}(i,2) = roadCenters(i,2) + offset{j};
        laneCenters{j}(i,3) = roadCenters(i,3) + 0;
    end
    laneInfo(j).LaneWidth     = laneSpecification.Width(j);
    
    if laneSpecification.Width(j) <= minLaneWidth
        laneInfo(j).LaneDirection = 0;
        laneInfo(j).LaneCenters   = laneCenters{j};
    else
        % Lanes with indices 2,3 are considered in positive direction
        if j/(numLanes/2) > 1
            laneInfo(j).LaneDirection = 1;
            laneInfo(j).LaneCenters   = laneCenters{j};
        else
            % Lanes with indices 4,5 are considered in negative direction
            laneInfo(j).LaneDirection = -1;
            laneInfo(j).LaneCenters   = flip(laneCenters{j});
        end
    end
end
end


function laneInfo = addCurvedRoadSegment(scenario)
% Add road representing a segment of the "Curved road" Sim 3D scene
rc = load('laneFollowingRoadCenters','roadCentersCurvedRoadSegment');
roadCenters = rc.roadCentersCurvedRoadSegment;

marking = [...
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed', 'Width', 0.13, 'Length', 1.5, 'Space', 3)
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Unmarked')];

laneSpecification = lanespec(6, 'Width', [1.15 3.85 4.05 4.05 3.85 1.15], 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

laneInfo = estimateLaneCenters(roadCenters,laneSpecification);

end


function addVehicle(scenario, passVehMesh, defaultVehiclePosition)
% Add vehicle to scenario using Sim 3D profiles
switch passVehMesh
    case "Hatchback"
        % https://www.mathworks.com/help/vdynblks/ref/hatchback.html
        vehicleProfile.Length = 0.589 + 1.343 + 1.104 + 0.828;
        vehicleProfile.Width = 1.653;
        vehicleProfile.Height = 1.286 + 0.227;
        vehicleProfile.FrontOverhang = 0.828;
        vehicleProfile.RearOverhang = 0.589;
    case "Muscle car"
        % https://www.mathworks.com/help/vdynblks/ref/musclecar.html
        vehicleProfile.Length = 0.945 + 1.529 + 1.491 + 0.983;
        vehicleProfile.Width = 2.009;
        vehicleProfile.Height = 1.129 + 0.241;
        vehicleProfile.FrontOverhang = 0.983;
        vehicleProfile.RearOverhang = 0.945;
    case "Small pickup truck"
        % https://www.mathworks.com/help/vdynblks/ref/smallpickuptruck.html
        vehicleProfile.Length = 1.321 + 1.750 + 1.947 + 1.124;
        vehicleProfile.Width = 2.073;
        vehicleProfile.Height = 1.502 + 0.488;
        vehicleProfile.FrontOverhang = 1.124;
        vehicleProfile.RearOverhang = 1.321;
    case "Sport utility vehicle"
        % https://www.mathworks.com/help/vdynblks/ref/sportutilityvehicle.html
        vehicleProfile.Length = 0.939 + 1.474 + 1.422 + 0.991;
        vehicleProfile.Width = 1.935;
        vehicleProfile.Height = 1.426 + 0.348;
        vehicleProfile.FrontOverhang = 0.991;
        vehicleProfile.RearOverhang = 0.939;
    otherwise % case "Sedan"
        % https://www.mathworks.com/help/vdynblks/ref/sedan.html
        vehicleProfile.Length = 1.119 + 1.305 + 1.513 + 0.911;
        vehicleProfile.Width = 1.842;
        vehicleProfile.Height = 1.246 + 0.271;
        vehicleProfile.FrontOverhang = 0.911;
        vehicleProfile.RearOverhang = 1.119;
end

% add vehicle to scenario
vehicle(scenario,...
    'ClassID',  1, ...
    'Position', defaultVehiclePosition, ...
    'Roll',     0, ...
    'Pitch',    0, ...
    'Yaw',      0, ...
    'Length',   vehicleProfile.Length, ...
    'Width',    vehicleProfile.Width, ...
    'Height',   vehicleProfile.Height, ...
    'RearOverhang',  vehicleProfile.RearOverhang,...
    'FrontOverhang', vehicleProfile.FrontOverhang);
end


function laneInfo = estimateLaneCenters(roadCenters,laneSpecification)
% estimateLaneCenters computes laneCenters from roadCenters based on
% laneSpecification for all the lanes in the scenario.
% RoadCenterDirection is defined as the positive increment of index in
% roadCenters array.
% laneInfo   : Structure that has below fields 
%              LaneWidth : Width of the lane.
%              LaneDirection: Direction of the lane. 
%              If the lane is in RoadCenterDirection, it is 1.
%              If the lane is opposite to the RoadCenterDirection, it is
%              -1.
%              If the lane iS border lane, it is 0.
%              LaneCenters: Center of the lanes. Lane centers are flipped
%              in case of  LaneDirection opposite to the RoadCenterDirection
%           
% laneInfo{1}: Left border  lane.
% laneInfo{2}: Leftmost lane from road center in RoadCenterDirection.
% laneInfo{3}: Immediate left lane from road center in RoadCenterDirection.
% laneInfo{4}: Immediate right lane from road center in RoadCenterDirection.
% laneInfo{5}: Rightmost lane from road center in RoadCenterDirection.
% laneInfo{6}: Right border lane.

offset = estimateRoadCenterOffset(laneSpecification);
[n,~] = size(roadCenters);
laneCenters = {zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3),zeros(n,3)};
minLaneWidth = 2;
laneInfo = struct();
for j = 1:numel(laneCenters)
    for i = 1:n-1
        span = hypot(roadCenters(i+1,1)-roadCenters(i,1),roadCenters(i+1,2)-roadCenters(i,2));
        theta = asin((roadCenters(i+1,2)-roadCenters(i,2))/span);
        
        if (roadCenters(i+1,1)-roadCenters(i,1)) > 0
            delX = -offset{j} * sin(theta);
            delY = offset{j} * cos(theta);
        else
            delX = -offset{j} * sin(theta);
            delY = -offset{j} * cos(theta);
        end
        
        laneCenters{j}(i,1) = roadCenters(i,1) + delX;
        laneCenters{j}(i,2) = roadCenters(i,2) + delY;
        laneCenters{j}(i,3) = 0;
    end
    
    laneCenters{j}(n,1) = roadCenters(n,1) + delX;
    laneCenters{j}(n,2) = roadCenters(n,2) + delY;
    laneCenters{j}(n,3) = 0;
    
    if j/(numel(laneCenters)/2)>1
        laneInfo(j).LaneWidth     = laneSpecification.Width(j);
        if laneSpecification.Width(j) > minLaneWidth
            laneInfo(j).LaneDirection = 1;
        else
            laneInfo(j).LaneDirection = 0;
        end
        laneInfo(j).LaneCenters   = laneCenters{j}(2:end-1,:);
    else
        laneInfo(j).LaneWidth     = laneSpecification.Width(j);
        if laneSpecification.Width(j) > minLaneWidth
            laneInfo(j).LaneDirection = -1;
            laneInfo(j).LaneCenters   = flip(laneCenters{j}(2:end-1,:));
        else
            laneInfo(j).LaneDirection = 0;
            laneInfo(j).LaneCenters   = laneCenters{j}(2:end-1,:);
        end
    end
end
end


function offset = estimateRoadCenterOffset(laneSpecification)
% estimateRoadCenterOffset takes laneSpecification as input and gives
% offsets for each lane from road center.
% 'offset' means distance from the road center.
% Output : offset as a cell array.

numLanes = sum(laneSpecification.NumLanes);
% Lane width less than minLaneWith is not considered as lane for offset
% calculation
minLaneWidth = 2; % Distance in meter
offset = cell(1,numLanes);
i = 1;

% If number of Lanes are even
    if mod(numLanes,2) == 0
        leftWidths = laneSpecification.Width(1:numLanes/2);
        rightWidths = laneSpecification.Width((numLanes/2)+1:end);
    
        % offset is calculated based on the Lane width of each lane
        for j = 1:length(leftWidths)
            if j < length(leftWidths)
                offset{i} = (leftWidths(j)/2) + leftWidths(j+1);
                i = i+1;
            else
                offset{i} = leftWidths(j)/2;
                i = i+1;
            end
        end
    
        % Negative sign is used based on convention that Lane offset towards
        % left of road centeres are positive and towards right are negative
        for j = 1:length(rightWidths)
            if j == 1
                offset{i} = -rightWidths(j)/2;
                i = i+1;
            else
                offset{i} = -((rightWidths(j)/2) + rightWidths(j-1));
                i = i+1;
            end
        end
    % If number of Lanes are odd
    else
        leftWidths = laneSpecification.Width(1:floor(numLanes/2));
        rightWidths = laneSpecification.Width(ceil(numLanes/2)+1:end);
        centerLaneWidth = laneSpecification.Width(ceil(numLanes/2));
        for j = 1:length(leftWidths)
            if leftWidths(j) > minLaneWidth
                if j < length(leftWidths)
                    offset{i} = (centerLaneWidth/2) + (leftWidths(j)/2) + leftWidths(j+1);
                    i = i+1;
                else
                    offset{i} = leftWidths(j)/2 + (centerLaneWidth/2);
                    i = i+1;
                end
            end
        end
        offset{ceil(numLanes/2)} = 0;
        i = i+1;
        for j = 1:length(rightWidths)
            if rightWidths(j) > minLaneWidth
                if j == 1
                    offset{i} = -((rightWidths(j)/2) + (centerLaneWidth/2));
                    i = i+1;
                else
                    offset{i} = -((centerLaneWidth/2) + (rightWidths(j)/2) + rightWidths(j-1));
                    i = i+1;
                end
            end
        end
    end
end
