% here_map: Nx4

clear;
addpath('./lib');


% Simulation options
sim_options = {};
sim_options.t_start = 780;
%sim_options.t_end = 1500;
sim_options.t_end = 4200;
%sim_options.t_end = 900;
sim_options.optimization_period = 0.1; % Amount of simulated time in seconds to wait between each slam optimization
sim_options.icp_period = 15; % Amount of simulated time in second to wait between each calculation of ICP
sim_options.plot_period = 15; % Amount of real time in seconds to wait between each update of the trajectory and landmark plot

% Load vehicle data
load('VehicleData3.mat');
vehicle_input = processVehicleInput(testListArray, sim_options.t_start, sim_options.t_end);

% Load global map data
load('SignMap_4.mat');
global_map = zeros(size(signDataOut_03.latLong,1),3);

global_map(:,3) = signDataOut_03.TypeArray;
for i = 1:size(signDataOut_03.latLong,1)
    [tempx, tempy] = gpsTransform(signDataOut_03.latLong(i,2), signDataOut_03.latLong(i,1), vehicle_input.ref_long, vehicle_input.ref_lat);
    global_map(i,1:2) = [tempx,tempy];
end

% Run simulation
sim_output = runSimulation(vehicle_input, global_map, sim_options);

% Display the final estimated local map
figure(2);
hold on;
plotLocalMap(sim_output.local_map);
plot(vehicle_input.gps(:,1),vehicle_input.gps(:,2));
plot(sim_output.trajectory(:,1), sim_output.trajectory(:,2));
axis equal;

