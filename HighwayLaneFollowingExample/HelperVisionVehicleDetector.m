classdef HelperVisionVehicleDetector < matlab.System & matlab.system.mixin.Propagates
%HelperVisionVehicleDetector Provides vehicle detections on image frame.
    % HelperVisionVehicleDetector estimates vehicle positions on the 
    % image frame provided by monoCamera sensor.
    % NOTE: The name of this System Object and it's functionality may 
    % change without notice in a future release, 
    % or the System Object itself may be removed.
    % Copyright 2019 The MathWorks, Inc.
    
    properties(Nontunable, Logical)       
        EnableVehicleDetector = true;
        EnableDisplay = true;
    end
    
    properties
        % Camera sensor parameters
        Camera = struct('ImageSize',[768 1024],'PrincipalPoint',...
        [512 384],'FocalLength',[512 512],'Position',[1.8750 0 1.2000],...
        'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0],...
        'DetectionRanges',[6 30],'MeasurementNoise',diag([6,1,1]));
    end
    
    properties (SetAccess='private', GetAccess='private', Hidden)
       Sensor;
       MonoDetector;
       VehicleDetectionThreshold;
       IsDetectingVehicles;
       DefaultOutVehicles;
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
            
            obj.IsDetectingVehicles = true;
           
            obj.VehicleDetectionThreshold = 20;
            
            % Assign mono camera sensor property
            obj.Sensor = sensor;
          
            % The width of common vehicles is between 1.5 to 2.5 meters.
            vehicleWidth = [1.5, 2.5];
            detector = vehicleDetectorACF();
            obj.MonoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);
            % Default output data structures
            [vehiclesBusName] = getOutputDataTypeImpl(obj);
            obj.DefaultOutVehicles = Simulink.Bus.createMATLABStruct(vehiclesBusName);
            
        end
        
        function [vehicles] = stepImpl(obj,frame,t)
            
            sensorOut = detectVehicles(obj, frame);
            if obj.EnableDisplay
                displaySensorOutputs(obj,frame,sensorOut,false);
            end
            vehicles = packVehicleDetections(obj,sensorOut,t);
            
        end
        
        function [vehicles] = getOutputSizeImpl(obj) %#ok<MANU>
            % Return size for each output port
            vehicles = 1;
            
        end
        
        function [vehicles] = getOutputDataTypeImpl(obj) %#ok<MANU>
            % Return data type for each output port
            vehicles = "BusVision";
          
        end
        
        function [vehicles] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            vehicles = false;
           
        end
        
        function [vehicles] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            vehicles = true;
           
        end
        function sensorOut =  detectVehicles(obj,frame)
             if obj.IsDetectingVehicles
                [bboxes, scores] = detect(obj.MonoDetector, frame);
            else
                bboxes = [];
                scores = [];
            end

            % Remove detections with low classification scores
            if ~isempty(scores)
                ind = scores >= obj.VehicleDetectionThreshold;
                bboxes = bboxes(ind, :);
                scores = scores(ind);
            end
            
            % Compute distance in vehicle coordinates
            sensorOut.vehicleLocations = computeVehicleLocations(bboxes, obj.Sensor);
            sensorOut.vehicleBoxes     = bboxes;

        end
          function isPlayerOpen = ...
                displaySensorOutputs(~, frame, sensorOut, closePlayers)
            
            locations        = sensorOut.vehicleLocations;
            
            frameWithOverlays = insertVehicleDetections(frame, locations, sensorOut.vehicleBoxes);
            
            
            % Display the results
            frames = {frameWithOverlays};
            
            persistent players;
            if isempty(players)
                frameNames = {'Vehicle detections'};
                players = helperVideoPlayerSet(frames, frameNames);
            end
            update(players, frames);
            
            % terminate the loop when the first player is closed
            isPlayerOpen = isOpen(players, 1);
            
            if (~isPlayerOpen || closePlayers) % close down the other players
                clear players;
            end
        end   
        function detections = packVehicleDetections(obj,sensorOut,t)
            % Create a struct of "zeros"
            detections = obj.DefaultOutVehicles;

            locations = sensorOut.vehicleLocations;

            numDetections = size(locations,1);
            detections.NumDetections = numDetections;

            for n = 1:numDetections
                % Bias location measurements as expected by sensor fusion
                % - Sensor fusion is with respect to vehicle rear axle
                % - camera.Position is mount position with respect to rear axle
                locations(n,:) = locations(n,:) + obj.Camera.Position(1:2);

                % Pack to constant velocity measurement format:
                detections.IsValidTime = true;
                detections.Detections(n).Measurement = [...
                    locations(n,1),...
                    locations(n,2),...
                    0]'; % [x,y,z]
                detections.Detections(n).MeasurementNoise = obj.Camera.MeasurementNoise;
                    detections.Detections(n).Time(:) = t;
                detections.Detections(n).SensorIndex = 1;
                detections.Detections(n).ObjectClassID = 1;
                detections.Detections(n).MeasurementParameters.Frame = drivingCoordinateFrameType.Rectangular;
                detections.Detections(n).MeasurementParameters.HasVelocity = false;
                detections.Detections(n).MeasurementParameters.Orientation = eye(3);
            end
        end

    end
    
  
        
    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(....
                "Title","HelperVisionVehicleDetector",...
                "Text",...
                "Detects vehicles from camera image." + newline + newline +...
                "Enable display of debugging visualizations to show intermediate processing for vehicle detections.");
        end
        
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
    
end
function locations = computeVehicleLocations(bboxes, sensor)

locations = zeros(size(bboxes,1),2);
for i = 1:size(bboxes, 1)
    bbox  = bboxes(i, :);
    
    yBottom = bbox(2) + bbox(4) - 1;
    xCenter = bbox(1) + (bbox(3)-1)/2;
    
    locations(i,:) = imageToVehicle(sensor, [xCenter, yBottom]);
end
end
%--------------------------------------------------------------------------
% insertVehicleDetections function inserts bounding boxes and displays
% [x,y] locations corresponding to returned vehicle detections.
function imgOut = insertVehicleDetections(imgIn, locations, bboxes)

imgOut = imgIn;

for i = 1:size(locations, 1)
    location = locations(i, :);
    bbox     = bboxes(i, :);
        
    label = sprintf('X=%0.2f, Y=%0.2f', location(1), location(2));

    imgOut = insertObjectAnnotation(imgOut, ...
        'rectangle', bbox, label, 'Color','g');
end
end
