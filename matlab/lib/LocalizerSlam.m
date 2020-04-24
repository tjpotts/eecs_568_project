
classdef LocalizerSlam < handle
    properties
        % ISAM2 optimizer
        isam

        % Objects to hold new factors to add to the factor graph and their initial estimates
        new_factors
        initial_estimates

        % Index and timestamp of the last pose added to the factor graph
        pose_index = 0
        pose_t = 0

        % Current pose, trajectory and map estimate
        current_pose
        trajectory
        local_map

        % Holds a list of already observed landmarks and their semantic types
        % landmarks(n,:) = [landmark_id, landmark_type]
        landmarks = zeros(0,2);

        % TODO: Create noise models
        odometry_noise
        gps_noise
        landmark_noise

        % Holds the output from ISAM
        result
    end

    methods
        % Initialize the SLAM algorithm
        % initialPose: SE2 pose
        function initialize(obj)
            import gtsam.*

            obj.isam = ISAM2();
            obj.new_factors = NonlinearFactorGraph();
            obj.initial_estimates = Values();

            % Set the initial pose estimate at the origin and create a prior factor
            %obj.current_pose = Pose2(0,0,pi);
            obj.current_pose = Pose2(0,0,0);
            obj.pose_index = 0;
            obj.pose_t = 0;
            obj.new_factors.add(PriorFactorPose2(0, obj.current_pose, noiseModel.Diagonal.Sigmas([1e-5; 1e-5; 0.01])));
            obj.initial_estimates.insert(0, Pose2(0,0,0));

            % Create noise models
            obj.odometry_noise = noiseModel.Diagonal.Sigmas([1; 1; 0.1]);
            obj.gps_noise = noiseModel.Diagonal.Sigmas([1e4; 1e4]);
            obj.landmark_noise = noiseModel.Diagonal.Sigmas([100; 100]);
        end

        % Adds a new odometry measurement to the factor graph
        % t: timestamp of measurement
        % meas: [3x3] SE2 transformation
        function addOdometry(obj, t, meas)
            dt = t - obj.pose_t;

            % Add odometry factor the the factor graph
            i = obj.pose_index + 1; % Index for new pose
            odometry_pose = gtsam.Pose2(meas(1)*dt, meas(2)*dt, meas(3)*dt);
            odometry_factor = gtsam.BetweenFactorPose2(i-1, i, odometry_pose, obj.odometry_noise);
            obj.new_factors.add(odometry_factor);

            % Calculate initial estimate of new pose based on previous estimate and odometry
            % Update the estimate of our current pose based on previous estimate and new odometry information
            % and use it as our initial estimate for the new pose
            obj.current_pose = obj.current_pose.compose(odometry_pose);
            obj.initial_estimates.insert(i, obj.current_pose);

            % Store the index and timestamp of the pose added
            obj.pose_index = i;
            obj.pose_t = t;
        end

        % Adds a new GPS measurement to the factor graph
        % t: timestamp of measurement
        % meas: [X,Y] coordinates of GPS measurement
        function addGps(obj, t, meas)
            % TODO: Incorporate t and last odometry measurement into factor to account for distance travelled
            %       since corresponding pose was added?
            % Create the GPS factor and add it to the factor graph
            gps_pose = gtsam.Pose2(meas(1),meas(2),0);
            gps_factor = gtsam.PoseTranslationPrior2D(obj.pose_index, gps_pose, obj.gps_noise);
            obj.new_factors.add(gps_factor);
        end

        % Adds a new landmark observation to the factor graph
        % t: timestamp of measurement
        % meas: [Nx4] data of landmark observations at current timestep
        % meas(n,:) [ID,X,Y,Type] data for landmark observation n
        function addLandmark(obj, t, meas)
            import gtsam.*

            % TODO: Incorporate t and alst odometry measurement into factor to account for distance travelled
            %       since corresponding pose was added?
            % Add the landmark observation factor to the pose graph
            landmark_id = symbol('L',meas(1));
            landmark_point = Point2(meas(2),meas(3));
            landmark_factor = DeltaFactor(obj.pose_index, landmark_id, landmark_point, obj.landmark_noise);
            obj.new_factors.add(landmark_factor);

            % Check if this the first time seeing this landmark
            if (~any(obj.landmarks(:,1) == meas(1)))
                % Add the landmark to the list of observerd landmarks
                obj.landmarks = [obj.landmarks; meas(1) meas(4)];

                % Calculate the initial estimate based on the current pose estimate
                landmark_initial = obj.current_pose.transformFrom(landmark_point);
                obj.initial_estimates.insert(landmark_id, landmark_initial);
                obj.local_map = [obj.local_map; meas(1) landmark_initial.x landmark_initial.y meas(4)];
            end
        end

        % Run the optimizer
        function optimize(obj)
            import gtsam.*

            % TODO: Will probably need to avoid running until we have enough information to
            %       have a well-defined linear system
            % Run ISAM2 and get our new estimate of our current pose and map
            obj.isam.update(obj.new_factors,obj.initial_estimates);
            results = obj.isam.calculateEstimate();
            obj.current_pose = results.atPose2(obj.pose_index);
            for i = 1:obj.pose_index
                trajectory_pose = results.atPose2(i);
                obj.trajectory(i,1) = trajectory_pose.x;
                obj.trajectory(i,2) = trajectory_pose.y;
            end
            for i = 1:size(obj.local_map,1)
                landmark_id = obj.local_map(i,1);
                landmark_point = results.atPoint2(symbol('L',landmark_id));
                obj.local_map(i,2) = landmark_point.x;
                obj.local_map(i,3) = landmark_point.y;
            end

            % Reset the new_factors and initial_estimates objects
            obj.new_factors.resize(0);
            obj.initial_estimates.clear();
        end

        % Get the current pose estimate of the vehicle
        % returns: SE2 pose
        function pose = getPoseEstimate(obj)
            % TODO: Convert from gtsam type to 3x3 matrix
            pose = obj.current_pose;
        end

        % Get the current estimated trajectory of the vehicle
        % returns: [Nx3x3] matrics of stacked SE2 poses
        function traj = getTrajectoryEstimate(obj)
            traj = obj.trajectory;
        end

        % Get the current estimate of the landmark map
        % returns: Nx3 matrix of landmark estimates
        % map(n,:) = [X,Y,Type] for landmark n
        function map = getMapEstimate(obj)
            map = obj.local_map;
        end
    end
end

