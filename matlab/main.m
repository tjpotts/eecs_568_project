% here_map: Nx4

clear;
addpath('./lib');


% Simulation options
sim_options = {};
sim_options.t_start = 780;
%sim_options.t_end = 1000;
sim_options.t_end = 1500;
%sim_options.t_end = 4000;
sim_options.optimization_period = 0.1; % Amount of simulated time in seconds to wait between each slam optimization
sim_options.icp_period = 15; % Amount of simulated time in seconds to wait between each calculation of ICP
sim_options.video_period = 0; % Amount of simulated time in seconds between output video frames (0 = disable video writer)

% Load vehicle data
load('VehicleData3.mat');
vehicle_input = processVehicleInput(testListArray, sim_options.t_start, sim_options.t_end);

% Use original gps data as "ground truth"
ground_truth = {};
ground_truth.t_pos = vehicle_input.t_gps;
ground_truth.pos = vehicle_input.gps;

% Corrupt GPS data
gps_offset = [100; 50];
vehicle_input.gps = vehicle_input.gps + gps_offset';

% Load global map data
load('SignMap_5.mat');
global_map = zeros(size(signDataOut_03.latLong,1),3);

global_map(:,3) = signDataOut_03.TypeArray;
for i = 1:size(signDataOut_03.latLong,1)
    [tempx, tempy] = gpsTransform(signDataOut_03.latLong(i,2), signDataOut_03.latLong(i,1), vehicle_input.ref_long, vehicle_input.ref_lat);
    global_map(i,1:2) = [tempx,tempy];
end

% Run simulation
sim_output = runSimulation(vehicle_input, global_map, ground_truth, sim_options);

% Error post-processing
sicp_idx = find(sim_output.sicp.err < 10000);
sicp_runs = numel(sim_output.sicp.err);
transform_error = zeros(sicp_runs,1);
for i = 1:sicp_runs
    %transform_diff = gps_offset + sim_output.sicp.transform(i,1:2,4);
    transform_diff = gps_offset + sim_output.sicp.transform(i,:);
    transform_error(i) = sqrt(transform_diff(1)^2 + transform_diff(2)^2);
end

% Display the final estimated local map
figure(2);
hold on;
plotLocalMap(sim_output.local_map);
plot(vehicle_input.gps(:,1),vehicle_input.gps(:,2));
plot(sim_output.trajectory(:,1), sim_output.trajectory(:,2));
axis equal;

% Generate error plots
figure(3);
grid on;
scatter(sim_output.sicp.matches(sicp_idx), sim_output.sicp.err(sicp_idx));
xlabel("Number of matched landmarks");
ylabel("SICP error");

figure(4);
grid on;
scatter(sim_output.sicp.matches(sicp_idx), transform_error(sicp_idx));
xlabel("Number of matched landmarks");
ylabel("Trajectory correction error");

figure(5);
grid on;
scatter(sim_output.sicp.err(sicp_idx), transform_error(sicp_idx));
xlabel("SICP error");
ylabel("Trajectory correction error");

