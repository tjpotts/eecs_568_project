function [output] = runSimulation(vehicle_input, global_map, ground_truth, options)
    local_map = [];
    trajectory = [];

    % Initialize slam
    slam = LocalizerSlam();
    %TODO: Set initial position estimate based on first GPS data
    slam.initialize();

    % Setup plots
    figure(1);
    clf;
    axis equal;
    hold on;
    trajectory_line = animatedline('Color','b');
    ground_truth_line = animatedline('Color','r');
    trajectory_plot = plot([0],[0]);
    local_map_scatter = scatter([],[]);

    % Main loop
    tic;
    t = options.t_start;
    t_last_optimization = t;
    t_last_icp = t;
    t_plot_update = toc;
    i_odometry = find(vehicle_input.t_odometry > options.t_start, 1);
    i_gps = find(vehicle_input.t_gps > options.t_start, 1);
    i_landmarks = find(vehicle_input.t_landmarks > options.t_start, 1);;
    done = 0;
    while(~done)

        % Add new odometry measurements at the current time
        while (t >= vehicle_input.t_odometry(i_odometry))
            slam.addOdometry(vehicle_input.t_odometry(i_odometry), vehicle_input.odometry(i_odometry,:));
            i_odometry = i_odometry + 1;
        end

        % Add new GPS measurements at the current time
        while (t >= vehicle_input.t_gps(i_gps))
            slam.addGps(vehicle_input.t_gps(i_gps), vehicle_input.gps(i_gps,:));

            % Plot the "ground truth" vehicle position
            % TODO: Update so that this does not rely on having the same time points as gps
            addpoints(ground_truth_line, ground_truth.pos(i_gps,1), ground_truth.pos(i_gps,2));

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
                local_map_scatter.XData = local_map(:,2);
                local_map_scatter.YData = local_map(:,3);
                t_plot_update = toc;
            end
        end


        % Perform ICP on local_map and global_map
        if (t - t_last_icp > options.icp_period)
            local_map = slam.getMapEstimate();

            initial_transform = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            target = {};
            target.Location = zeros(size(global_map,1),3);
            target.Location(:,1:2) = global_map(:,1:2);
            target.Label = global_map(:,3);
            source = {};
            source.Location = zeros(size(local_map,1),3);
            source.Location(:,1:2) = local_map(:,2:3);
            source.Label = local_map(:,4);

            sicp_obj = sicp(target, source, initial_transform);
            sicp_obj.getCorrectedPose();
            sicp_obj.calculateError();

            display(size(sicp_obj.correlation,1));
            display(sicp_obj.converged);
            display(sicp_obj.avgError);
            display(sicp_obj.T);

            x_offset = sicp_obj.T(1,4);
            y_offset = sicp_obj.T(2,4);

            trajectory = slam.getTrajectoryEstimate();
            trajectory_plot.XData = trajectory(:,1) + x_offset;
            trajectory_plot.YData = trajectory(:,2) + y_offset;

            t_last_icp = t;
        end

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

