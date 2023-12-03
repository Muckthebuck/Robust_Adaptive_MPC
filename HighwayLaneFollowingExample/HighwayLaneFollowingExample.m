%% Highway Lane Following
% This example shows how to simulate a highway lane following
% application with controller, sensor fusion, and vision processing
% components. These components are tested in a 3D simulation environment
% that includes camera and radar sensor models.

% Copyright 2019 The MathWorks, Inc.

%% Introduction
% A highway lane-following system steers a vehicle to travel within a
% marked lane. It also maintains a set velocity or safe distance to a
% preceding vehicle in the same lane. The system typically uses vision
% processing algorithms to detect lanes and vehicles from a camera. The
% vehicle detections from the camera are then fused with detections from a
% radar to improve the ability to detect surrounding vehicles. The
% controller uses the lane detections, vehicle detections, and set speed to
% control steering and acceleration.
%
% This example demonstrates how to create a test bench model to test vision
% processing, sensor fusion, and controls in a 3D simulation environment.
% The test bench model can be configured for different scenarios to test
% the ability to follow lanes and avoid collisions with other vehicles. In
% this example, you:
%
% # *Explore the test bench model*: The model contains vision processing,
% sensor fusion, controls, vehicle dynamics, sensors, and metrics to assess
% functionality.
% # *Visualize a test scenario*: The scenario contains a curved road with
% multiple vehicles.
% # *Simulate with a probabilistic detection sensor*: The model is
% configured to test the integration of sensor fusion and controls using a
% probabilistic vision detection sensor model. This is helpful to assess
% baseline behavior before integrating the full vision processing
% algorithm.
% # *Simulate with a vision processing algorithm*:  The test bench model is
% configured to test integration of the vision processing, sensor fusion,
% and controls components.
% # *Explore additional scenarios*: These scenarios test the system under
% additional conditions.
%%
% Testing the integration of the controller and the perception algorithm
% requires a photorealistic simulation environment. In this example, you
% enable system-level simulation through integration with the Unreal
% Engine. The 3D simulation environment requires a Windows(R) 64-bit
% platform.
%%
if ~ispc
    error("The 3D simulation environment requires a Windows 64-bit platform");
end

%%
% To ensure reproducibility of the simulation results, set the random seed.
rng(1)

%% Explore Test Bench Model
% In this example, you use a system-level simulation test bench model to
% explore the behavior of the control and vision processing algorithms for
% the lane following system. Open the system-level simulation test bench
% model.
open_system("HighwayLaneFollowingTestBench")

%%
% The test bench model contains these subsystems: 
%
% # Simulation 3D Scenario: Specifies road, vehicles, camera and radar
% sensors used for simulation
% # Vision Detector Variant: Specifies the fidelity of the two different
% vision detection algorithms to choose from
% # Forward Vehicle Sensor Fusion: Fuses the detections of vehicles in
% front of the ego vehicle that were obtained from vision and radar
% sensors
% # Lane Following Decision and ControllerS: Specifies lateral and
% longitudinal decision logic and the lane following controller
% # Vehicle Dynamic: Specifies the dynamics model for the ego vehicle
% # Metrics Assessment: Assesses system-level behavior
% 
% The Forward Vehicle Sensor Fusion, Lane Following Decision and
% Controller, Vehicle Dynamics, and Metrics Assessment subsystems are
% based on the subsystems used in the
% <docid:driving_examples#mw_2b8d28ea-1921-4c7a-be3b-3c6f72b6ee00 Lane
% Following Control with Sensor Fusion and Lane Detection>. This example
% focuses on the Simulation 3D Scenario and Vision Detector Variant
% subsystems.

%%
% The Simulation 3D Scenario subsystem configures the road network, sets
% vehicle positions, and synthesizes sensors. Open the Simulation 3D
% Scenario subsystem.
open_system("HighwayLaneFollowingTestBench/Simulation 3D Scenario")

%%
% The scene and road network are specified by these parts of the subsystem:
%
% * The <docid:driving_ref#mw_96a521fd-316f-497b-bc01-b2c5f4083563
% Simulation 3D Scene Configuration> block has the *SceneName* parameter
% set to |Curved road|.
% * The <docid:driving_ref#mw_0abe0f52-f25a-4829-babb-d9bafe8fdbf3 Scenario
% Reader> block is configured to use a driving scenario that contains a
% road network that closely matches a section of the road network.
%
% The vehicle positions are specified by these parts of the subsystem:
%
% * The Ego input port controls the position of the ego vehicle, which is
% specified by the Simulation 3D Vehicle with Ground Following 1 block.
% * The <docid:driving_ref#mw_27640406-e319-4cd8-8d2c-6231f1a6a233 Vehicle
% To World> block converts actor poses from the coordinates of the input
% ego vehicle to the world coordinates.
% * The <docid:driving_ref#mw_0abe0f52-f25a-4829-babb-d9bafe8fdbf3 Scenario
% Reader> block outputs actor poses, which control the position of the
% target vehicles. These vehicles are specified by the other
% <docid:driving_ref#mw_32cd8e72-2d69-4c3e-98b0-5b918db383a4 Simulation 3D
% Vehicle with Ground Following> blocks.
% * The <docid:driving_ref#mw_da39544a-b88f-41dc-a641-9457988cebe9 Cuboid
% To 3D Simulation> block converts the ego pose coordinate system (with
% respect to below the center of the vehicle rear axle) to the 3D
% simulation coordinate system (with respect to below the vehicle center).
%
% The sensors attached to the ego vehicle are specified by these parts of
% the subsystem:
%
% * The <docid:driving_ref#mw_e9491451-3198-4988-8ef1-6a3878d29155
% Simulation 3D Camera> block is attached to the ego vehicle to capture
% its front view. The output Image from this block is processed by
% vision processing algorithm to detect lanes and vehicles.
% * The <docid:driving_ref#mw_875dee8d-fc18-4f3d-8eab-bace559a0d66
% Simulation 3D Probabilistic Radar Configuration> block is attached to the
% ego vehicle to detect vehicles in 3D Simulation environment.
% * The Measurement Bias Center to Rear Axle block converts the
% coordinate system of the
% <docid:driving_ref#mw_875dee8d-fc18-4f3d-8eab-bace559a0d66 Simulation 3D
% Probabilistic Radar Configuration> block (with respect to below the
% vehicle center) to the pose coordinates (with respect to below the center
% of the vehicle rear axle).

%%
% The Vision Detector Variant subsystem allows you to select the fidelity
% of the vision detection algorithm based on the types of tests you want
% to run. Open the Vision Detector Variant subsystem.
open_system("HighwayLaneFollowingTestBench/Vision Detector Variant")

%%
% * The Probabilistic Detection Sensor variant enables you to test
% integration of the control algorithm in the 3D simulation environment,
% without also integrating the vision processing algorithm. This variant
% uses a Vision Detection Generator block to synthesize vehicle and lane
% detections based on actor ground truth positions. This configuration
% helps you to verify interactions with vehicles and the radar sensor in
% the 3D simulation environment without vision processing algorithm.
% * The Vision Processing Algorithm variant enables you to test
% integration of the control algorithm and vision processing algorithm
% in the 3D simulation environment. Open the Vision Processing Algorithm
% variant.
open_system("HighwayLaneFollowingTestBench/Vision Detector Variant/Vision Processing Algorithm")

%%
% This variant uses a MATLAB based lane boundary and vehicle detection
% algorithm based on the
% <docid:driving_examples#mw_b13b3ce9-9de9-4d94-a014-13a01f2ca568 Visual
% Perception Using Monocular Camera> example. The primary difference from
% that example is that in this example, lane boundary detection and vehicle
% detection algorithms are segregated into separate components. Lane
% Marker Detector is a reference model that can generate C code. This
% reference model uses a System object&trade;, |HelperLaneMarkerDetector|,
% to detect the lane markers. It also contains a lane tracker to improve
% performance of lane detection in crowded conditions. Vision Vehicle
% Detector uses the |HelperVisionVehicleDetector| System object to detect
% the vehicles. These System objects pack the output data to buses, as
% required in further processing. Since the vision processing algorithm
% operates on an image returned by the camera sensor, the Vision
% Processing Algorithm takes longer to execute than the Probabilistic
% Detection Sensor variant.

%% Visualize a Test Scenario
% The helper function |scenario_LFACC_03_Curve_StopnGo| generates a driving
% scenario that is compatible with the |HighwayLaneFollowingTestBench|
% model. This is an open-loop scenario on a curved road and includes
% multiple target vehicles. The road centers and lane markings closely
% match a section of the curved road scene provided with the 3D simulation
% environment. The scenario has the same number of vehicles as the model
% and they have the same dimensions. In this scenario, a lead vehicle slows
% down in front of the ego vehicle while other vehicles travel in adjacent
% lanes.

%%
% Plot the open-loop scenario to see the interactions of the ego vehicle
% and target vehicles.
hFigScenario = helperPlotLFScenario("scenario_LFACC_03_Curve_StopnGo");

%%
% The ego vehicle is not under closed-loop control, so a collision occurs
% with a slower moving lead vehicle. The goal of the closed-loop system is
% to follow the lane and maintain a safe distance from the lead vehicles.
% In the |HighwayLaneFollowingTestBench| model, the ego vehicle has the
% same initial velocity and initial position as in the open-loop scenario.

%% Simulate with Probabilistic Vision Detection Sensor
% To verify that interactions with vehicles and the radar sensor are
% working properly, test the interactions between the control algorithm and
% the 3D simulation environment using the probabilistic vision detection
% sensor. Doing so enables you to verify baseline system behavior without
% integrating the full vision processing algorithm. Configure the test
% bench model and run the simulation.
helperSLHighwayLaneFollowingSetup(...
    "scenario_LFACC_03_Curve_StopnGo",...
    "ProbabilisticDetectionSensor");
mpcverbosity('off');
sim("HighwayLaneFollowingTestBench")

%%
% Plot the lateral controller performance results.
hFigLatResults = helperPlotLFLateralResults(logsout);

%%
% Examine the simulation results.
%
% * The *Detected lane boundary lateral offsets* plot shows the lateral
% offsets for the detected left-lane and right-lane boundaries. The
% detected values are close to the ground truth of the lane.
% * The *Lateral deviation* plot shows the lateral deviation of the ego
% vehicle from the centerline of the lane. The lateral deviation is close
% to 0, which implies that the ego vehicle closely follows the centerline.
% Small deviations occur when the vehicle is changing velocity to avoid
% collision with another vehicle.
% * The *Relative yaw angle* plot shows the relative yaw angle between ego
% vehicle and the centerline of the lane. The relative yaw angle is very
% close to 0, which implies that the heading angle of the ego vehicle
% matches the yaw angle of the centerline closely.
% * The *Steering angle* plot shows the steering angle of the ego vehicle.
% The steering angle trajectory is smooth.

%%
% Plot the longitudinal controller performance results.
hFigLongResults = helperPlotLFLongitudinalResults(logsout,time_gap,...
    default_spacing);
 
%%
% Examine the simulation results.
%
% * The *Relative longitudinal distance* plot shows the distance between
% the ego vehicle and the Most Important Object (MIO). The MIO represents
% the closest vehicle ahead of and in the same lane as the ego vehicle. In
% this case, the ego vehicle approaches the MIO and gets close to it or
% exceeds the safe distance in some cases.
% * The *Relative longitudinal velocity* plot shows the relative velocity
% between the ego vehicle and the MIO. In this example, the vision
% processing algorithm only detects positions, so the tracker in the
% control algorithm estimates the velocity. The estimated velocity lags the
% actual (ground truth) MIO relative velocity.
% * The *Absolute acceleration* plot shows that the controller commands the
% vehicle to decelerate when it gets too close to the MIO.
% * The *Absolute velocity* plot shows the ego vehicle initially follows
% the set velocity, but when the MIO slows down, to avoid a collision, the
% ego vehicle also slows down.

%%
% During simulation, the model logs signals to the base workspace as
% |logsout| and records the output of the camera sensor to
% |forwardFacingCamera.mp4|. You can use the |plotLFDetectionResults|
% function to visualize the simulated detections similar to how recorded
% data is explored in the
% <docid:driving_examples#mw_7998055b-23be-4950-aa9f-a11b98f2e1de
%  Forward Collision Warning Using Sensor Fusion>
% example. You can also record the visualized detections to a video file 
% to enable review by others who do not have access to MATLAB.

%%
% Plot the detection results from logged data, generate a video, 
% and open the video in the <docid:images_ref#buiat2d-1 Video Viewer> app.
hVideoViewer = helperPlotLFDetectionResults(...
    logsout, "forwardFacingCamera.mp4" , scenario, camera, radar,...
    scenarioFcnName,...
    "RecordVideo", true,...
    "RecordVideoFileName", scenarioFcnName + "_PDS",...
    "OpenRecordedVideoInVideoViewer", true,...
    "VideoViewerJumpToTime", 10.6);

%%
% Play the generated video.
%
% * *Front Facing Camera* shows the image returned by the camera sensor.
% The left lane boundary is plotted in red and the right lane boundary is
% plotted in green. These lanes are returned by the probabilistic detection
% sensor. Tracked detections are also overlaid on the video.
% * *Birds-Eye Plot* shows true vehicle positions, sensor coverage areas,
% probabilistic detections, and track outputs. The plot title includes the
% simulation time so that you can correlate events between the video and
% previous static plots.

%%
% Close the figures.
close(hFigScenario)
close(hFigLatResults)
close(hFigLongResults)
close(hVideoViewer)

%% Simulate with Vision Processing Algorithm
% Now that you verified the control algorithm, test the control algorithm
% and vision processing algorithm together in the 3D simulation
% environment. This enables you to explore the effect of the vision
% processing algorithm on system performance. Configure the test bench
% model to use the same scenario with the vision processing variant.
helperSLHighwayLaneFollowingSetup(...
    "scenario_LFACC_03_Curve_StopnGo",...
    "VisionProcessingAlgorithm");
sim("HighwayLaneFollowingTestBench")

%%
% Plot the lateral controller performance results.
hFigLatResults = helperPlotLFLateralResults(logsout);

%%
% The vision processing algorithm detects the left and right lane
% boundaries but the detections are noisier, which affects the lateral
% deviation. The lateral deviation is still small but is larger than the
% run with the probabilistic detection sensor variant.

%%
% Plot the longitudinal controller performance results.
hFigLongResults = helperPlotLFLongitudinalResults(logsout,time_gap,...
    default_spacing);

%%
% The relative distance and relative velocity have some discontinuities.
% These discontinuities are due to imperfections of the vision processing
% algorithm on system performance. Even with these discontinuities, the
% resulting ego acceleration and velocity are similar to the results using
% the probabilistic detection sensor variant.

%%
% Plot the detection results from logged data, generate a video, and open
% the <docid:images_ref#buiat2d-1 Video Viewer> app.
hVideoViewer = helperPlotLFDetectionResults(...
    logsout, "forwardFacingCamera.mp4" , scenario, camera, radar,...
    scenarioFcnName,...
    "RecordVideo", true,...
    "RecordVideoFileName", scenarioFcnName + "_VPA",...
    "OpenRecordedVideoInVideoViewer", true,...
    "VideoViewerJumpToTime", 10.6);

%%
% Close the figures.
close(hFigLatResults)
close(hFigLongResults)
close(hVideoViewer)

%% Explore Additional Scenarios
% The previous simulations tested the |scenario_LFACC_03_Curve_StopnGo|
% scenario using both the probabilistic vision detection sensor and vision
% processing algorithm variants. This example provides additional
% scenarios that are compatible with the |HighwayLaneFollowingTestBench|
% model:
%
%    scenario_LF_01_Straight_RightLane
%    scenario_LF_02_Straight_LeftLane
%    scenario_LF_03_Curve_LeftLane 
%    scenario_LF_04_Curve_RightLane
%    scenario_LFACC_01_Curve_DecelTarget
%    scenario_LFACC_02_Curve_AutoRetarget 
%    scenario_LFACC_03_Curve_StopnGo
%    scenario_LFACC_04_Curve_CutInOut
%    scenario_LFACC_05_Curve_CutInOut_TooClose
%    scenario_LFACC_06_Straight_StopandGoLeadCar


%%
% These scenarios represent two types of testing.
%
% * Use scenarios with the |scenario_LF_| prefix to test lane-detection and
% lane-following algorithms without obstruction by other vehicles. The
% vehicles still exist in the scenario, but are positioned such that they
% are not seen by the ego vehicle on the road.
% * Use scenarios with the |scenario_LFACC_| prefix
% to test lane-detection and lane-following algorithms with
% other vehicles on the road.

%%
% Examine the comments in each file for more details on the road and
% vehicles in each scenario. You can configure the
% |HighwayLaneFollowingTestBench| model and workspace to simulate these
% scenarios using the |helperSLHighwayLaneFollowingSetup| function.

%%
% For example, while learning about the effects of a camera-based lane
% detection algorithm on closed-loop control, it can be helpful to begin
% with a scenario that has a road but no vehicles. To configure the model
% and workspace for such a scenario, use the following code.
helperSLHighwayLaneFollowingSetup(...
    "scenario_LF_04_Curve_RightLane",...
    "VisionProcessingAlgorithm");

%% Conclusion
% This example shows how to simulate a highway lane following application
% with controller, sensor fusion, and vision processing components.
