classdef HelperLaneMarkerDetector < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    %HelperLaneMarkerDetector Provides lane marker detections on image frame.
    % HelperLaneMarkerDetector estimates lane markers on the image frame
    % provided by monoCamera sensor.
    % NOTE: The name of this System Object and it's functionality may 
    % change without notice in a future release, 
    % or the System Object itself may be removed.

    % Copyright 2019-2020 The MathWorks, Inc.
    
    properties(Nontunable, Logical)
        % Enabling lane tracker
        EnableLaneTracker = true;
        
         % Display debug visualization windows
        EnableDisplays = true;
    end
    
    properties
        % Camera sensor parameters
    Camera = struct('ImageSize',[768 1024],'PrincipalPoint',[512 384],...
        'FocalLength',[512 512],'Position',[1.8750 0 1.2000],...
        'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0],...
        'LaneDetectionRanges',[6 30],'DetectionRanges',[6 50],...
        'MeasurementNoise',diag([6,1,1]));
    end
    
    properties (SetAccess='private', GetAccess='private', Hidden)
        Sensor
        LastValidLaneLeft
        LastValidLaneRight
        KalmanLeftFilter
        KalmanRightFilter
        IsLeftTrackInitialized
        IsRightTrackInitialized
        LeftPredict
        RightPredict
        % Flag to discard invalid lanes at the staritng of the simulation
        % in lane tracker
        FirstInstance   
        LaneXExtentThreshold
        LaneSegmentationSensitivity  % increased as compared to original demo
        ApproximateLaneMarkerWidth   % width specified in meters
        LaneStrengthThreshold
        MaxNumLaneMarkersToDetect
        LaneDetectionRanges
        BirdsEyeConfig
        BirdsEyeImage
        BirdsEyeBW
        VehicleROI
        FrameCount
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        % System object methods for Simulink integration
        %------------------------------------------------------------------
        function setupImpl(obj)
            % Camera setup
            %-------------
            camera = obj.Camera;
            
            focalLength    = camera.FocalLength;
            principalPoint = camera.PrincipalPoint;
            imageSize      = camera.ImageSize;
            height         = camera.Position(3);  % mounting height in meters from the ground
            pitch          = camera.Rotation(2);  % pitch of the camera in degrees
            
            camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
            sensor        = monoCamera(camIntrinsics, height, 'Pitch', pitch);
            
            % Lane marker detection parameters
            %---------------------------------
            % The percentage extent of the ROI a lane needs to cover. It can remove
            % noisy detections
            obj.LaneXExtentThreshold = 0.4;
            
            obj.LaneSegmentationSensitivity = 0.5; % increased as compared to original demo
            obj.ApproximateLaneMarkerWidth = 0.3;  % width specified in meters
            obj.LaneStrengthThreshold = 0.2;
            
            obj.MaxNumLaneMarkersToDetect = 3;
            obj.LaneDetectionRanges = camera.LaneDetectionRanges;
            
            % Assign mono camera sensor property
            obj.Sensor = sensor;     
            
            % Assign Lane Tracker parameters
            obj.LastValidLaneLeft = parabolicLaneBoundary(zeros(1,3));
            obj.LastValidLaneRight = parabolicLaneBoundary(zeros(1,3));
            obj.LeftPredict = zeros(1,3);
            obj.RightPredict = zeros(1,3);
            obj.IsLeftTrackInitialized = false;
            obj.IsRightTrackInitialized = false;
            % To initialize LeftPredict and RightPredict properties with
            % valid lane boundary parameters at the starting of
            % the simulation
            obj.FirstInstance = true;
            obj.KalmanLeftFilter = configKalman(obj,zeros(1,3));
            obj.KalmanRightFilter = configKalman(obj,zeros(1,3));
        end
        
        function [lanes] = stepImpl(obj,frame)
            
            % Detect lane boundaries
            [leftEgoBoundary,rightEgoBoundary] = laneDetector(obj, frame); 
            
            % Reject invalid lanes 
            if obj.EnableLaneTracker
                [leftEgoBoundary,rightEgoBoundary] = rejectInvalidLanes(obj,leftEgoBoundary,rightEgoBoundary);
            end
            
            % Display debugging windows
            if(isempty(coder.target))
                if obj.EnableDisplays
                    displaySensorOutputs(obj, frame, leftEgoBoundary,rightEgoBoundary, false);
                end
            end
            
            % Pack lane boundaries to LaneSensor bus
            lanes = packLaneBoundaryDetections(obj,leftEgoBoundary,rightEgoBoundary);

        end
        %------------------------------------------------------------------
        % laneDetector detects lane boundaries in a frame of video sequence.
        %  [leftEgoBoundary,rightEgoBoundary] = laneDetector(frame) returns
        %  detected left and right ego-lane boundaries
        function [leftEgoBoundary,rightEgoBoundary] =  laneDetector(obj,frame)
            
            sensor = obj.Sensor;
            WidthOfBirdsEyeView = 16;
            
            % Define area to transform
            distAheadOfSensor = obj.LaneDetectionRanges(2); % in meters, as previously specified in monoCamera height input
            spaceToOneSide    = WidthOfBirdsEyeView/2;  % all other distance quantities are also in meters
            bottomOffset      = obj.LaneDetectionRanges(1);
            outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
            outImageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio
            
            % Compute birdsEyeView image
            birdsEyeConfig = birdsEyeView(sensor, outView, outImageSize);
            obj.BirdsEyeConfig = birdsEyeConfig;
            birdsEyeImage = transformImage(birdsEyeConfig, frame);           
            birdsEyeImage = rgb2gray(birdsEyeImage);
            obj.BirdsEyeImage = birdsEyeImage;
            
            
            % Lane marker segmentation ROI in world units
            vehicleROI = outView - [-1, 2, -3, 3]; % look 3 meters to left and right, and 4 meters ahead of the sensor
            obj.VehicleROI = vehicleROI;
            approxLaneMarkerWidthVehicle = obj.ApproximateLaneMarkerWidth; % 25 centimeters
            
            % Detect lane boundary features
            laneSensitivity = obj.LaneSegmentationSensitivity;
            birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, ...
                approxLaneMarkerWidthVehicle, 'ROI', vehicleROI, ...
                'Sensitivity', laneSensitivity);
            obj.BirdsEyeBW = birdsEyeViewBW;
            % Obtain lane candidate points in vehicle coordinates
            [imageX, imageY] = find(birdsEyeViewBW);
            xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
            
            maxLanes      = obj.MaxNumLaneMarkersToDetect; % look for maximum of two lane markers
            boundaryWidth = 3*approxLaneMarkerWidthVehicle; % expand boundary width
            
            % Find lane boundary candidates
            [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
                'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);
            
            % Establish criteria for rejecting boundaries based on their length
            maxPossibleXLength = diff(vehicleROI(1:2));
            minXLength         = maxPossibleXLength * obj.LaneXExtentThreshold; % establish a threshold
            
            % To compute the maximum strength, assume all image pixels within the ROI
            % are lane candidate points
            birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
            [laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));
            
            % Convert the image points to vehicle points
            vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);
            
            % Find the maximum number of unique x-axis locations possible for any lane
            % boundary
            maxPointsInOneLane = numel(unique(single((vehiclePoints(:,1)))));
            
            % Set the maximum length of a lane boundary to the ROI length
            maxLaneLength = diff(vehicleROI(1:2));
            
            % Compute the maximum possible lane strength for this image size/ROI size
            % specification
            maxStrength   = maxPointsInOneLane/maxLaneLength;

            if( numel(boundaries) > 0 )
                isOfMinLength = false(1, numel(boundaries));
                for i = 1 : numel(boundaries)
                    if(diff(boundaries(i).XExtent) > minXLength)
                        isOfMinLength(i) = true;
                    end
                end
            else
                isOfMinLength = false;
            end
            % Reject weak boundaries
            idx = 0;
            boundariesIsStrong = parabolicLaneBoundary(zeros(nnz(isOfMinLength), 3));
            for i = 1 : size(isOfMinLength,2)
                if( isOfMinLength(i) == 1 )
                    if( boundaries(i).Strength > obj.LaneStrengthThreshold*maxStrength )
                        idx = idx + 1;
                        boundariesIsStrong(idx) = boundaries(i);
                    end
                end
            end
            % Classify lane marker type when boundaryPoints are not empty
            if isempty(boundaryPoints)
                boundariesIsStrong = repmat(boundariesIsStrong,1,2);
                boundariesIsStrong(1) = parabolicLaneBoundary(zeros(1,3));
                boundariesIsStrong(2) = parabolicLaneBoundary(zeros(1,3));
            else
                boundariesIsStrong = classifyLaneTypes(boundariesIsStrong, boundaryPoints);
            end
            % Find ego lanes
            xOffset = 0;    %  0 meters from the sensor
            distanceToBoundaries = coder.nullcopy(ones(size(boundariesIsStrong,2),1));
            
            for i = 1 : size(boundariesIsStrong, 2)
                distanceToBoundaries(i) = boundariesIsStrong(i).computeBoundaryModel(xOffset);
            end
            
            % Find candidate ego boundaries
            if (numel(distanceToBoundaries(distanceToBoundaries>0)))
                minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
            else
                minLDistance = 0;
            end
            
            if( numel(distanceToBoundaries(distanceToBoundaries< 0)))
                minRDistance = max(distanceToBoundaries(distanceToBoundaries< 0));
            else
                minRDistance = 0;
            end
            % Find left ego boundary
            if (minLDistance ~= 0)
                leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
                leftEgoBoundary = parabolicLaneBoundary(zeros(nnz(leftEgoBoundaryIndex), 3));
                idx = 0;
                for i = 1 : size(leftEgoBoundaryIndex, 1)
                    if( leftEgoBoundaryIndex(i) == 1)
                        idx = idx + 1;
                        leftEgoBoundary(idx) = boundariesIsStrong(i);
                    end
                end
            else
                leftEgoBoundary = parabolicLaneBoundary(zeros(1,3));
            end
            % Find right ego boundary
            if (minRDistance ~= 0)
                rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
                rightEgoBoundary = parabolicLaneBoundary(zeros(nnz(rightEgoBoundaryIndex), 3));
                idx = 0;
                for i = 1 : size(rightEgoBoundaryIndex, 1)
                    if( rightEgoBoundaryIndex(i) == 1)
                        idx = idx + 1;
                        rightEgoBoundary(idx) = boundariesIsStrong(i);
                    end
                end
            else
                rightEgoBoundary = parabolicLaneBoundary(zeros(1,3));
            end
        end        
        
        %------------------------------------------------------------------
        % displaySensorOutputs method displays core information and
        % intermediate results from the monocular camera sensor simulation.
        function isPlayerOpen = ...
                displaySensorOutputs(obj, frame, leftEgoBoundary,rightEgoBoundary, closePlayers)
            
            sensor = obj.Sensor;
            bottomOffset      = obj.LaneDetectionRanges(1);
            distAheadOfSensor = obj.LaneDetectionRanges(2);
            xVehiclePoints = bottomOffset:distAheadOfSensor;
            birdsEyeViewImage = obj.BirdsEyeImage;
            birdsEyeConfig    = obj.BirdsEyeConfig;
            birdsEyeViewBW    = obj.BirdsEyeBW;
            if(~nnz(leftEgoBoundary.Parameters))
                leftEgoBoundary = parabolicLaneBoundary.empty;
            end
             if(~nnz(rightEgoBoundary.Parameters))
                rightEgoBoundary = parabolicLaneBoundary.empty;
            end
            birdsEyeWithOverlays = insertLaneBoundary(birdsEyeViewImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
            birdsEyeWithOverlays = insertLaneBoundary(birdsEyeWithOverlays, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
            
            frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
            frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');    
            
            imageROI = vehicleToImageROI(birdsEyeConfig, obj.VehicleROI);
            ROI = [imageROI(1) imageROI(3) imageROI(2)-imageROI(1) imageROI(4)-imageROI(3)];
            
            % Highlight candidate lane points that include outliers
            birdsEyeViewImage = insertShape(birdsEyeViewImage, 'rectangle', ROI); % show detection ROI
            birdsEyeViewImage = imoverlay(birdsEyeViewImage, birdsEyeViewBW, 'yellow');
            
            % Display the results
            frames = {frameWithOverlays, birdsEyeViewImage, birdsEyeWithOverlays};        
            persistent players;
            if isempty(players)
                frameNames = {'Lane marker detections', 'Raw segmentation', 'Lane marker detections'};
                players = helperVideoPlayerSet(frames, frameNames);
            end
            update(players, frames);
            
            % terminate the loop when the first player is closed
            isPlayerOpen = isOpen(players, 1);
            
            if (~isPlayerOpen || closePlayers) % close down the other players
                clear players;
            end
        end
        
        %------------------------------------------------------------------
        % packLaneBoundaryDetections method packs Pack detections into
        % format expected by LaneFollowingDecisionLogicandControl. 
        function detections = packLaneBoundaryDetections(obj,leftEgoBoundary,rightEgoBoundary)
            
            % Preallocate struct expected by controller
            DefaultLanes = struct('Curvature',{single(0)},...
                'CurvatureDerivative',{single(0)},...
                'HeadingAngle',{single(0)},...
                'LateralOffset',{single(0)},...
                'Strength',{single(0)},...
                'XExtent',{single([0,0])},...
                'BoundaryType',{LaneBoundaryType.Unmarked});
            field1 = 'Left'; field2 = 'Right';
            detections = struct(field1,DefaultLanes,field2,DefaultLanes);           
            zeroStrengthLane = DefaultLanes;
            
            % Pack detections into struct
            detections.Left  = packLaneBoundaryDetection(leftEgoBoundary);
            detections.Right = packLaneBoundaryDetection(rightEgoBoundary);

            % Shift detections to vehicle center as required by controller
            % Note: camera.PositionSim3d(1) represents the X mount location of the
            %       camera sensor with respect to the vehicle center
            if nnz(leftEgoBoundary.Parameters)
                detections.Left.LateralOffset(:) = polyval(...
                    leftEgoBoundary.Parameters, -obj.Camera.PositionSim3d(1));
                % Lane to left should always have positive lateral offset
                if detections.Left.LateralOffset < 0
                    detections.Left = zeroStrengthLane;
                end
            end
            if nnz(rightEgoBoundary.Parameters)
                detections.Right.LateralOffset(:) = polyval(...
                    rightEgoBoundary.Parameters, -obj.Camera.PositionSim3d(1));
                % Lane to right should always have negative lateral offset
                if detections.Right.LateralOffset > 0
                    detections.Right = zeroStrengthLane;
                end
            end
        end
        
        %------------------------------------------------------------------
        % rejectInvalidLanes method reject lane boundaries if they are
        % invalid and replaces using the predicted outputs from Kalman
        % tarckers.
        function [leftEgoBoundary,rightEgoBoundary] = rejectInvalidLanes(obj,leftEgoBoundary,rightEgoBoundary)
            
            % Discard the unreasonable lanes and replace with estimated lanes
            if(obj.FirstInstance && ~isempty(leftEgoBoundary) && ~isempty(rightEgoBoundary))
                if(nnz(leftEgoBoundary.Parameters) && nnz(rightEgoBoundary.Parameters))
                    obj.LeftPredict = leftEgoBoundary.Parameters;
                    obj.RightPredict =  rightEgoBoundary.Parameters;
                    obj.FirstInstance = false;
                end
            end
              
            % Valid lane parameters
            latDeltaMax  = 1.5; % Max lateral change between steps (m)
            laneWidthMin = 2.5; % Minimum valid lane width (m)
            laneWidthMax = 5.5; % Minimum valid lane width (m)
            
            % Initialize flags to reject lanes
            replaceLeft = false;
            replaceRight = false;
            
            % Previous detected lanes
            leftPrev  = obj.LastValidLaneLeft;
            rightPrev = obj.LastValidLaneRight;
            
            % Current detected lanes
            leftCur = leftEgoBoundary;
            rightCur = rightEgoBoundary;
            
            % Longitudinal points to compare lateral distances (1 m increments)
            longDist = obj.Camera.DetectionRanges(1):obj.Camera.DetectionRanges(2);
            
            if (~obj.FirstInstance)
                % Check validity of left lane
                if isempty(leftCur)
                    replaceLeft = true;
                else
                    % Replace lane if lateral distance deviates too much
                    latDistLeftPrev = leftPrev.computeBoundaryModel(longDist);
                    latDistLeftCur  = leftCur.computeBoundaryModel(longDist);
                    latDeltaMaxArray = latDeltaMax*ones(size(latDistLeftCur));
                    if nnz(abs(latDistLeftCur - latDistLeftPrev) > latDeltaMaxArray)
                        replaceLeft = true;
                    end
                end
            
                % Check validity of right lane               
                if isempty(rightCur)
                    replaceRight = true;
                else
                    % Replace lane if lateral distance deviates too much
                    latDistRightPrev = rightPrev.computeBoundaryModel(longDist);
                    latDistRightCur  = rightCur.computeBoundaryModel(longDist);
                    latDeltaMaxArray = latDeltaMax*ones(size(latDistRightCur));
                    if nnz(abs(latDistRightCur - latDistRightPrev) > latDeltaMaxArray)
                        replaceRight = true;
                    end
                end
            end
            
            % Replace lanes
            if replaceLeft
                leftCur = leftPrev;
                if ~nnz(leftCur.Parameters)
                    % Replace left lane parameters with predicted parameters
                    % from Kalman filter
                    leftCur.Parameters = obj.LeftPredict;
                end
            end
            
            if replaceRight
                rightCur = rightPrev;
                if ~nnz(rightCur.Parameters)
                    % Replace right lane parameters with predicted parameters
                    % from Kalman filter
                    rightCur.Parameters = obj.RightPredict;
                end
            end
            
            % Validate that lane widths are expected
            latDistLeftCur  = leftCur.computeBoundaryModel(longDist);
            latDistRightCur = rightCur.computeBoundaryModel(longDist);
            laneWidth = latDistLeftCur - latDistRightCur;
            if ~isempty(laneWidth)
                
                minLanes = laneWidth < laneWidthMin;
                maxLanes = laneWidth > laneWidthMax; 
                if  (nnz(minLanes)) || (nnz(maxLanes))
                    % Lanes are not in a trustworthy state, reset to empty
                    leftCur = parabolicLaneBoundary(zeros(1,3));
                    rightCur = parabolicLaneBoundary(zeros(1,3));
                end
            end
            
            % Update output
            leftEgoBoundary = leftCur;
            rightEgoBoundary = rightCur;
            
            % Update state
            obj.LastValidLaneLeft = leftCur;
            obj.LastValidLaneRight = rightCur;
            
            % Update tracker
            updateTracker(obj,leftCur,rightCur);
        end

        %------------------------------------------------------------------
        % updateTracker intializes Kalman filters for left and right
        % lanes. In case of improper lane detections, replace lanes
        % with the predicted data from Kalman filters. In case of
        % proper lanes, update the filter.
        function updateTracker(obj,leftCur,rightCur)
                       
            detectionsLeft = zeros(1,3);
            detectionsRight = zeros(1,3);
            leftCurParameters = leftCur.Parameters;
            if (nnz(leftCurParameters))
                detectionsLeft = [leftCur.Parameters(1)...
                    leftCur.Parameters(2) ...
                    leftCur.Parameters(3)];
            end
            rightCurParameters = rightCur.Parameters;
            if (nnz(rightCurParameters))
                detectionsRight = [rightCur.Parameters(1)...
                    rightCur.Parameters(2) ...
                    rightCur.Parameters(3)];
            end

              if(~obj.IsLeftTrackInitialized) && nnz(detectionsLeft)
                lenSubState = 3;
                classToUse = 'double';
                numDims = length(detectionsLeft);
                lenState = numDims * lenSubState;
                State = zeros(lenState, 1, classToUse);
                State(1: lenSubState: lenState) = detectionsLeft;
                obj.KalmanLeftFilter.State = State; 
                obj.IsLeftTrackInitialized = true;
             end 
            
              if(~obj.IsRightTrackInitialized) && nnz(detectionsRight)
                lenSubState = 3;
                classToUse = 'double';
                numDims = length(detectionsRight);
                lenState = numDims * lenSubState;
                State = zeros(lenState, 1, classToUse);
                State(1: lenSubState: lenState) = detectionsRight;
                obj.KalmanRightFilter.State = State; 
                obj.IsRightTrackInitialized = true;
             end              
             
            if (obj.IsLeftTrackInitialized && nnz(detectionsLeft))
                % Correct the KalmanLeftFilter with proper left lane detections
                correct(obj.KalmanLeftFilter, detectionsLeft);
            end            
            if (obj.IsRightTrackInitialized && nnz(detectionsRight))
                % Correct the KalmanRightFilter with proper right lane detections
                correct(obj.KalmanRightFilter, detectionsRight);
            end             
            % Update obj.LeftPredict and obj.RightPredict to use them if the lanes
            % are improper in the next sample
            if (obj.IsLeftTrackInitialized)
                obj.LeftPredict =  predict(obj.KalmanLeftFilter);
            end
            if obj.IsRightTrackInitialized
                obj.RightPredict = predict(obj.KalmanRightFilter);
            end             
         end
         
        %------------------------------------------------------------------
        % configKalman intializes Kalman filters 
        function kalmanFilter = configKalman(~,initialLocation)
        % Create a KalmanFilter object
        lenSubState = 3;
        As = [1, 1, 0.5; 0, 1, 1; 0, 0, 1];
        Hs = [1, 0, 0];
        InitialEstimateError = [1 1 1]*1e5;
        MotionNoise = [25, 10, 10];
        MeasurementNoise = 25;
        numDims = length(initialLocation);
        lenState = numDims * lenSubState;
        classToUse = 'double';
        StateTransitionModel = zeros(lenState, lenState, classToUse);
        MeasurementModel     = zeros(numDims,  lenState, classToUse);
        State = zeros(lenState, 1, classToUse);
        StateCovariance  = diag(repmat(InitialEstimateError, [1, numDims]));
        ProcessNoise     = diag(repmat(MotionNoise,          [1, numDims]));
        MeasurementNoise = diag(repmat(MeasurementNoise,     [1, numDims]));
        
        for iDim = 1: numDims
            iFirst = (iDim - 1) * lenSubState + 1;
            iLast = iDim * lenSubState;
            StateTransitionModel(iFirst:iLast, iFirst:iLast) = As;
            MeasurementModel(iDim, iFirst:iLast) = Hs;
        end
        State(1: lenSubState: lenState) = initialLocation;
        kalmanFilter = vision.KalmanFilter(StateTransitionModel,MeasurementModel,...
             'ProcessNoise',ProcessNoise ,'MeasurementNoise',MeasurementNoise,'StateCovariance',StateCovariance );       
        kalmanFilter.State = State;     
    end
    
    
        function [lanes] = getOutputSizeImpl(obj) %#ok<MANU>
            % Return size for each output port
            lanes = 1;
        end
        
        function [lanes] = getOutputDataTypeImpl(obj) %#ok<MANU>
            % Return data type for each output port
            lanes = "LaneSensor";
        end
        
        function [lanes] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            lanes= false;
        end
        
        function [lanes] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            lanes = true;
        end
    end
    
    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(....
                "Title","HelperLaneMarkerDetector",...
                "Text",...
                "Detects lanes from camera image." + newline + newline +...
                "Enable display of debugging visualizations to show intermediate processing for lane detections.");
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end
            
    
end
function detection = packLaneBoundaryDetection(boundary)
% Parameters of parabolicLaneBoundary object = [A B C]
%  corresponds to the three coefficients of a second-degree
%  polynomial equation:
%                y = Ax^2 + Bx + C
% Comparing this equation with lane model using 2nd order
% polynomial approximation:
%  y = (curvature/2)*(x^2) + (headingAngle)*x + lateralOffset
%
% This leads to the following relationship
%   curvature           = 2 * A = 2 * Parameters(1)  (unit: 1/m)
%   headingAngle        = B     = Parameters(2)      (unit: radians)
%   lateralOffset       = C     = Parameters(3)      (unit: meters)
%

% Default lane of zero strength
detection = struct('Curvature',{single(0)},'CurvatureDerivative',...
    {single(0)},'HeadingAngle',{single(0)},'LateralOffset',{single(0)},...
    'Strength',{single(0)},'XExtent',{single([0,0])},...
    'BoundaryType',{LaneBoundaryType.Unmarked});
if nnz(boundary.Parameters)
    detection.Curvature(:)     = 2 * boundary.Parameters(1);
    detection.HeadingAngle(:)  = boundary.Parameters(2); % Coordinate transform
    detection.LateralOffset(:) = boundary.Parameters(3); % Coordinate transform
    detection.Strength(:)      = boundary.Strength;
    detection.XExtent(:)       = boundary.XExtent;
    detection.BoundaryType(:)  = boundary.BoundaryType;
end
end
%--------------------------------------------------------------------------
% Function that's used to reject some of the found curves
function isGood = validateBoundaryFcn(params)

if ~isempty(params)
    a = params(1);
    
    % Reject any curve with a small 'a' coefficient, which makes it highly
    % curved.
    isGood = abs(a) < 0.003; % a from ax^2+bx+c
else
    isGood = false;
end
end
function imageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI)

vehicleROI = double(vehicleROI);

loc2 = (abs(vehicleToImage(birdsEyeConfig, [vehicleROI(2) vehicleROI(4)])));
loc1 = (abs(vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(4)])));
loc4 =     (vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(4)]));
loc3 =     (vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(3)]));

[minRoiX, maxRoiX, minRoiY, maxRoiY] = deal(loc4(1), loc3(1), loc2(2), loc1(2));

imageROI = round([minRoiX, maxRoiX, minRoiY, maxRoiY]);

end
%--------------------------------------------------------------------------
% Determine Lane Marker Types Classify lane boundaries as 'solid',
% 'dashed', etc.
function boundariesIsStrong = classifyLaneTypes(boundariesIsStrong, boundaryPoints)

for bInd = 1 : size(boundariesIsStrong,2)
    
    vehiclePoints = boundaryPoints{bInd};
    % Sort by x
    vehiclePoints = sortrows(vehiclePoints, 1);
    
    xVehicle = vehiclePoints(:,1);
    xVehicleUnique = unique(xVehicle);
    
    % Dashed vs solid
    xdiff  = diff(xVehicleUnique);
    % Sufficiently large threshold to remove spaces between points of a
    % solid line, but not large enough to remove spaces between dashes
    xdifft = mean(xdiff) + 3*std(xdiff);
    largeGaps = xdiff(xdiff > xdifft);
    
    % Safe default
    boundary = boundariesIsStrong(bInd);           % changed according to set/get methods
    boundary.BoundaryType= LaneBoundaryType.Solid;  
    
    if largeGaps>1
        % Ideally, these gaps should be consistent, but you cannot rely
        % on that unless you know that the ROI extent includes at least 3 dashes.
        boundary.BoundaryType= LaneBoundaryType.Dashed;
    end
    boundariesIsStrong(bInd) = boundary;
end

end
