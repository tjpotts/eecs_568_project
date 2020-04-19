% t_odometry: Nx1
% odometry: Nx3x3

% t_gps: Nx1
% gps: Nx2

% t_landmarks: Nx1
% landmarks: Nx4

% here_map: Nx4

% TODO: Add input data listed above

slam = LocalizerSlam;
slam.initialize();

done = 0;
t = 0;
while(!done)
    % Add odometry measurements at the current time
    i_odometry = find(t_odometry == t)
    if (size(i_odometry,2) > 0)
        slam.addOdometry(odometry(i_odometry))
    end

    % Add gps measurements at the current time
    i_gps = find(t_gps == t)
    if (size(i_gps,2) > 0)
        slam.addOdometry(gps(i_gps))
    end

    % Add landmark observations at the current time
    i_odometry = find(t_odometry == t)
    if (size(i_odometry,2) > 0)
        slam.addOdometry(odometry(i_odometry))
    end

    % Run optimization
    slam.optimize()

    % Get results
    local_map = slam.getMapEstimate()
    trajectory = slam.getTrajectoryEstimate()

    % TODO: Perform icp on here_map and local_map to calculate map_transform

    % TODO: Visualize trajectory estimate, local_map and here_map

    % Get the next time value
    next_t_odometry = find(t_odometry>t,1)
    next_t_gps = find(t_gps>t,1)
    next_t_landmarks = find(t_landmarks>t,1)
    t = min([next_t_odometry, next_t_gps, next_t_landmarks])
    if (size(t,1) == 0)
        done = 1;
    end
end

