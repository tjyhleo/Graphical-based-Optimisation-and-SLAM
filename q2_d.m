% This script runs Q2(d)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Disable the GPS and enable the laser for pure SLAM.
configuration.enableGPS = false;
configuration.enableLaser = true;

% For this part of the coursework, this should be set to zero.
configuration.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
%truncate this value between 1207 and 1208
configuration.maximumStepNumber = 1207;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times.
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
hold on

% Plot the error curves.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance.
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
legend('x','y','psi')
hold on

% Plot errors.
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('x','y','psi')
hold on


%%%%%%%%%%%%%%%%%%%Q2d%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%the step number before loop closure is 1207, after loop closure is 1208
%record the landmark estimate and covariance, plateform estimate and
%covariance
if configuration.maximumStepNumber == 1207
    [x_L_pre, P_L_pre, landmarkIds] = drivebotSLAMSystem.landmarkEstimates();
    [x_V_pre, P_V_pre] = drivebotSLAMSystem.platformEstimate();
elseif configuration.maximumStepNumber == 1208
    [x_L_pos, P_L_pos, landmarkIds] = drivebotSLAMSystem.landmarkEstimates();
    [x_V_pos, P_V_pos] = drivebotSLAMSystem.platformEstimate();
end

%%%%%%uncomment to plot, run after all the data has been recorded%%%%%%%%%
%Plot landmark estimate before and after loop closure
% uncomment to plot
% minislam.graphics.FigureManager.getFigure('Landmark Estimate');
% clf
% plot(x_L_pre(1,:),x_L_pre(2,:),'*','MarkerSize',10)
% hold on
% plot(x_L_pos(1,:),x_L_pos(2,:),'o','MarkerSize',10)
% legend('pre','pos')


% Plot vehicle state before and after loop closure
% uncomment to plot
% minislam.graphics.FigureManager.getFigure('Vehicle Estimate');
% clf
% plot([x_V_pre(1),x_V_pos(1)],'*-','MarkerSize',10)
% hold on
% plot([x_V_pre(2),x_V_pos(2)],'o-','MarkerSize',10)
% plot([x_V_pre(3),x_V_pos(3)],'x-','MarkerSize',10)
% legend('x','y','psi')