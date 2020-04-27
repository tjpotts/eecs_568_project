function [output] = runSimulation(vehicle_input, global_map, options)
    local_map = [];
    trajectory = [];

    % Initialize slam
    slam = LocalizerSlam();
    slam.initialize();

    % Setup plots
    figure(1);
    clf;
    axis equal;
    hold on;
    trajectory_line = animatedline('Color','b');
    gps_line = animatedline('Color','r');
    trajectory_plot = plot([0],[0]);
    local_map_scatter = scatter([],[]);

    % Main loop
    tic;
    t = options.t_start;
    t_last_optimization = t;
    t_plot_update = toc;
    i_odometry = find(vehicle_input.t_odometry > options.t_start, 1);
    i_gps = find(vehicle_input.t_gps > options.t_start, 1);
    i_landmarks = find(vehicle_input.t_landmarks > options.t_start, 1);;
    done = 0;
    while(~done)
        display(t);

        % Add new odometry measurements at the current time
        while (t >= vehicle_input.t_odometry(i_odometry))
            slam.addOdometry(vehicle_input.t_odometry(i_odometry), vehicle_input.odometry(i_odometry,:));
            i_odometry = i_odometry + 1;
        end

        % Add new GPS measurements at the current time
        while (t >= vehicle_input.t_gps(i_gps))
            slam.addGps(vehicle_input.t_gps(i_gps), vehicle_input.gps(i_gps,:));
            addpoints(gps_line, vehicle_input.gps(i_gps,1), vehicle_input.gps(i_gps,2));
            i_gps = i_gps + 1;
        end

        % Add new landmark observations at the current time
        while (t >= vehicle_input.t_landmarks(i_landmarks))
            slam.addLandmark(vehicle_input.t_landmarks(i_landmarks), vehicle_input.landmarks(i_landmarks,:));
            i_landmarks = i_landmarks + 1;
        end

        % Run optimization every 100ms simulated time
        if (t - t_last_optimization > options.optimization_period)
            slam.optimize();
            t_last_optimization = t;

            % Update the plot every 10s real time
            % NOTE: Currently this needs to be run directly after slam.optimize() in order to avoid issues
            % in slam.getTrajectoryEstimate()
            t_real = toc;
            if (t_real - t_plot_update > options.plot_period)
                % Get results
                local_map = slam.getMapEstimate();
                %scatter(local_map(:,2),local_map(:,3));
                local_map_scatter.XData = local_map(:,2);
                local_map_scatter.YData = local_map(:,3);
                trajectory = slam.getTrajectoryEstimate();
                trajectory_plot.XData = trajectory(:,1);
                trajectory_plot.YData = trajectory(:,2);
                t_plot_update = toc;
            end
        end


        % TODO: Perform icp on here_map and local_map to calculate map_transform

        % Visualize trajectory
        pose = slam.getPoseEstimate();
        addpoints(trajectory_line, pose.x, pose.y);
        drawnow limitrate;
        % TODO: Visualize local_map and here_map

        % Get the next time value
        next_t_odometry = vehicle_input.t_odometry(i_odometry);
        next_t_gps = vehicle_input.t_gps(i_gps);
        next_t_landmarks = vehicle_input.t_landmarks(i_landmarks);
        t = min([next_t_odometry; next_t_gps; next_t_landmarks]);
        if (t >= options.t_end)
            done = 1;
        end
    end

    output = {};
    output.trajectory = slam.getTrajectoryEstimate();
    output.local_map = slam.getMapEstimate();
end

