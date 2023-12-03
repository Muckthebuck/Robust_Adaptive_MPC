% Clean up script for the Highway Lane Following Example
%
% This script cleans up the LF example model. It is triggered by the
% CloseFcn callback.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2019 The MathWorks, Inc.

clear BusActors1
clear BusDetectionConcatenation1
clear BusDetectionConcatenation1Detections
clear BusDetectionConcatenation1DetectionsMeasurementParameters
clear BusLaneBoundaries1
clear BusLaneBoundaries1LaneBoundaries
clear BusLaneDetections1
clear BusLaneDetections1LaneBoundaries
clear BusMultiObjectTracker1
clear BusMultiObjectTracker1Tracks
clear BusObjectDetections1
clear BusObjectDetections1Detections
clear BusRadar
clear BusRadarDetections
clear BusRadarDetectionsMeasurementParameters
clear BusRadarDetectionsObjectAttributes
clear BusVehiclePose
clear BusVision
clear BusVisionDetections
clear BusVisionDetectionsMeasurementParameters
clear BusVisionDetectionsObjectAttributes
clear BusMultiObjectTracker
clear BusMultiObjectTrackerTracks
clear Cf
clear Cr
clear Iz
clear LaneSensor
clear LaneSensorBoundaries
clear M
clear N
clear PredictionHorizon
clear Ts
clear assessment
clear assigThresh
clear camera
clear clusterSize
clear default_spacing
clear egoVehDyn
clear lf
clear lr
clear m
clear max_ac
clear max_steer
clear min_ac
clear min_steer
clear numCoasts
clear numSensors
clear numTracks
clear posSelector
clear radar
clear scenario
clear scenarioFcnName
clear tau
clear time_gap
clear v0_ego
clear v_set
clear vehSim3D
clear velSelector
clear visionVariant
clear forwardCameraOutputFile
clear max_dc
clear tau2
clear BusVehicleToWorldActors1
clear driver_decel
clear FB_decel
clear headwayOffset
clear PB1_decel
clear PB2_decel
clear timeMargin
clear timeToReact
close all
