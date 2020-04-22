% t_odometry: Nx1
% odometry: Nx3x3

% t_gps: Nx1
% gps: Nx2

% t_landmarks: Nx1
% landmarks: Nx4

% here_map: Nx4

% TODO: Add input data listed above

clear;

START_TIME = 780;
END_TIME = 1500;

% Load vehicle data
load('VehicleData3.mat');
addpath('./lib');

t_odometry = testListArray.Odometry.YawRateTime;
odometry = zeros(numel(t_odometry),1);
odometry_index = 1;
for i = 1:numel(t_odometry)
    t = t_odometry(i);
    if (t < testListArray.Odometry.VehicleSpeedTime(1))
        odometry(i,1) = testListArray.Odometry.VehicleSpeed(1);
    elseif (t > testListArray.Odometry.VehicleSpeedTime(end))
        odometry(i,1) = testListArray.Odometry.VehicleSpeed(end);
    else
        while (t > testListArray.Odometry.VehicleSpeedTime(odometry_index+1))
            odometry_index = odometry_index+1;
        end
        t1 = testListArray.Odometry.VehicleSpeedTime(odometry_index);
        t2 = testListArray.Odometry.VehicleSpeedTime(odometry_index+1);
        v1 = testListArray.Odometry.VehicleSpeed(odometry_index);
        v2 = testListArray.Odometry.VehicleSpeed(odometry_index+1);
        slope = (v2 - v1) / (t2 - t1);
        odometry(i,1) = v1 + slope * (t - t1);
    end

    odometry(i,1) = odometry(i,1) * 0.27777;

    % Rotational velocity (converted from deg/s to rad/s)
    %odometry(i,3) = testListArray.Odometry.YawRate(i) * pi / 180;
    odometry(i,3) = testListArray.Odometry.YawRate(i);
    %odometry(i,3) = testListArray.Odometry.YawRate(i) * 180 / pi;
end

t_gps = testListArray.GPS.Time;
gps = zeros(numel(t_gps),2);
i_gps = find(t_gps > START_TIME, 1);
lat0 = testListArray.GPS.Lat(i_gps);
lon0 = testListArray.GPS.Long(i_gps);
gps(:,2) = (testListArray.GPS.Lat - lat0) .* (pi / 180 * 6378100);
gps(:,1) = -(testListArray.GPS.Long - lon0) .* (pi / 180 * 6378100) .* cos(lat0);
for i = 1:numel(t_gps)
    gps(i,1) = distance(lat0,lon0,lat0,testListArray.GPS.Long(i)) * (pi / 180 * 6378100);
    if (testListArray.GPS.Long(i) < lon0)
        gps(i,1) = -gps(i,1);
    end
    gps(i,2) = distance(lat0,lon0,testListArray.GPS.Lat(i),lon0) * (pi / 180 * 6378100);
    if (testListArray.GPS.Lat(i) < lat0)
        gps(i,2) = -gps(i,2);
    end
end

landmark_count = min([
    size(testListArray.Landmark.Time,1),
    size(testListArray.Landmark.ID,1),
    size(testListArray.Landmark.X,1),
    size(testListArray.Landmark.Y,1),
    size(testListArray.Landmark.Type,1)
]);

t_landmarks = testListArray.Landmark.Time(1:landmark_count,:);
t_landmarks = reshape(t_landmarks,[numel(t_landmarks),1]);

landmarks = zeros(numel(t_landmarks),4);
landmarks(:,1) = reshape(testListArray.Landmark.ID(1:landmark_count,:), [numel(t_landmarks),1]);
landmarks(:,2) = reshape(testListArray.Landmark.X(1:landmark_count,:), [numel(t_landmarks),1]);
landmarks(:,3) = reshape(testListArray.Landmark.Y(1:landmark_count,:), [numel(t_landmarks),1]);
landmarks(:,4) = reshape(testListArray.Landmark.Type(1:landmark_count,:), [numel(t_landmarks),1]);

% Sort by time
[t_landmarks, landmarks_sorted_index] = sort(t_landmarks);
landmarks = landmarks(landmarks_sorted_index,:);

% Filter out empty landmark measurements
landmarks_filtered_index = landmarks(:,1) > 0;
t_landmarks = t_landmarks(landmarks_filtered_index);
landmarks = landmarks(landmarks_filtered_index,:);

% Create unique ids
landmark_id_times = zeros(0,3);
next_landmark_id = max(landmarks(:,1)) + 1
for i = 1:numel(t_landmarks)
    id = landmarks(i,1);
    idx = find(landmark_id_times(:,1) == id,1);
    if (numel(idx) == 0)
        % First time seeing landmark
        landmark_id_times = [landmark_id_times; id t_landmarks(i) id];
    elseif (t_landmarks(i) - landmark_id_times(idx,2) < 10)
        % Consider this the same landmark as previously seen
        landmark_id_times(idx,2) = t_landmarks(i);
        landmarks(i,1) = landmark_id_times(idx,3);
    else
        % Consider this a new landmark
        landmark_id_times(idx,2) = t_landmarks(i);
        landmark_id_times(idx,3) = next_landmark_id;
        landmarks(i,1) = next_landmark_id;
        next_landmark_id = next_landmark_id + 1;
    end
end

% Initialization

slam = LocalizerSlam();
slam.initialize();

% Main loop

done = 0;
t = START_TIME;
figure(1);
axis equal;
hold on;
trajectory_line = animatedline('Color','b');
gps_line = animatedline('Color','r');
local_map_scatter = scatter([],[]);

i_odometry = find(t_odometry > START_TIME, 1);
i_gps = find(t_gps > START_TIME, 1);
i_landmarks = find(t_landmarks > START_TIME, 1);;

tic;
t_plot_update = toc;
while(~done)
    display(t);

    % Add odometry measurements at the current time
    if (t >= t_odometry(i_odometry))
        slam.addOdometry(t_odometry(i_odometry), odometry(i_odometry,:));
        i_odometry = i_odometry + 1;
    end

    % Add GPS measurements at the current time
    if (t >= t_gps(i_gps))
        %slam.addGps(t_gps(i_gps), gps(i_gps,:));
        addpoints(gps_line, gps(i_gps,1), gps(i_gps,2));
        i_gps = i_gps + 1;
    end

    % Add landmark observations at the current time
    if (t >= t_landmarks(i_landmarks))
        slam.addLandmark(t_landmarks(i_landmarks), landmarks(i_landmarks,:));
        i_landmarks = i_landmarks + 1;
    end

    % Run optimization
    slam.optimize();

    t_real = toc;
    if (t_real - t_plot_update > 1)
        % Get results
        local_map = slam.getMapEstimate();
        %scatter(local_map(:,2),local_map(:,3));
        local_map_scatter.XData = local_map(:,2);
        local_map_scatter.YData = local_map(:,3);
        trajectory = slam.getTrajectoryEstimate();
        t_plot_update = toc;
    end

    % TODO: Perform icp on here_map and local_map to calculate map_transform

    % Visualize trajectory
    pose = slam.getPoseEstimate();
    addpoints(trajectory_line, pose.x, pose.y);
    drawnow limitrate nocallbacks;
    % TODO: Visualize local_map and here_map

    % Get the next time value
    next_t_odometry = t_odometry(i_odometry);
    next_t_gps = [];
    next_t_landmarks = t_landmarks(i_landmarks);
    t = min([next_t_odometry; next_t_gps; next_t_landmarks]);
    if (t >= END_TIME)
        done = 1;
    end
end

