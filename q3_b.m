% This script runs Q3(b)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% SLAM is enabled, GPS disabled
configuration.enableGPS = false;
configuration.enableLaser = true;

%Q3b setting odometry reading frequency which also affects laser observation frequency
%the value is set to 0.5
configuration.DT = 0.5;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a');

% Create the localization system
drivebotSLAMSystem = drivebot.KeyFrameSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation evolves
drivebotSLAMSystem.setRecommendOptimizationPeriod(500);

% Set whether the SLAM system should remove prediction edges. If the first
% value is true, the SLAM system should remove the edges. If the second is
% true, the first prediction edge will be retained.
% drivebotSLAMSystem.setRemovePredictionEdges(false, true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
legend('x','y','psi')
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x','y','psi')
hold on

% Plot the chi2 value down here. The chi2 values are logged internally and
% can be plotted directly down here.
% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on

% Q3b:
% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();
NumVertex = length(allVertices); %number of vertices
